"""
Single-owner hardware controller for myCobot 280 M5: /joint_states + FollowJointTrajectory.

SAFETY RULES:
- ``allow_motion`` defaults to ``False``. When False, the node behaves strictly
  as a read-only monitor while still advertising the FollowJointTrajectory
  action server (which rejects all incoming motion goals).
- Never calls ``release_all_servos``, ``release_servo``, ``power_on``, or any
  motion command on startup.
- Owns the single serial connection guarded by a reentrant lock so polling and
  trajectory execution never collide on the USB bus.
"""

from dataclasses import dataclass
import inspect
import math
import threading
import time
from typing import Callable, List, Optional, Sequence

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from mycobot_280_hardware.gripper_conversion import (
    GripperCalibration, device_value_to_radians, GRIPPER_JOINT,
)
from mycobot_280_hardware.gripper_execution import GripperTrajectoryExecutor

from mycobot_280_hardware.joint_conversion import (
    check_signs,
    DEFAULT_JOINT_NAMES,
    DEFAULT_LOWER,
    DEFAULT_UPPER,
    degrees_to_joint_positions,
    find_limit_violations,
    joint_positions_to_degrees,
    NUM_JOINTS,
    validate_angles,
)
from mycobot_280_hardware.trajectory_validator import (
    coalesce_waypoints,
    duration_to_seconds,
    ERROR_GOAL_TOLERANCE_VIOLATED,
    ERROR_INVALID_GOAL,
    ERROR_NONE,
    interpolate_desired_waypoint,
    validate_trajectory,
    WaypointSpec,
)

DEFAULT_ACTION_NAME = 'mycobot280m5_group_controller/follow_joint_trajectory'


@dataclass(frozen=True)
class SegmentExecutionRecord:
    """Instrumentation record for one dispatched hardware trajectory segment."""

    cmd_index: int
    total_commands: int
    sent_time_s: float
    elapsed_at_send_s: float
    planned_time_from_start_s: float
    planned_segment_duration_s: float
    actual_segment_duration_s: float
    pymycobot_speed: int
    target_positions_rad: List[float]
    target_angles_deg: List[float]
    measured_start_rad: List[float]
    measured_end_rad: List[float]
    max_error_rad: float
    poll_count: int


class ControlledArm:
    """
    Thread-safe wrapper around a pymycobot handle with explicit motion gating.

    Only exposes ``get_angles``, ``send_angles``, ``stop``, ``is_moving``,
    ``reset_buffers``, and ``close``. Never exposes ``release_all_servos`` or
    ``power_on``.
    """

    def __init__(
        self,
        device,
        allow_motion: bool = False,
        min_io_interval_s: float = 0.015,
        gripper_stop_strategy: str = 'torque',
        gripper_command_mode: str = 'sdk_ack',
    ):
        self._device = device
        self._allow_motion = bool(allow_motion)
        self._min_io_interval_s = max(0.0, float(min_io_interval_s))
        self._lock = threading.RLock()
        self.configure_gripper_stop(gripper_stop_strategy)
        if gripper_command_mode not in ('sdk_ack', 'feedback'):
            raise ValueError('Unknown gripper command mode')
        self._gripper_command_mode = gripper_command_mode
        self._gripper_feedback_pending = False
        self._session_authorizer = None
        self._last_io_mono = 0.0
        self._send_accepts_async = self._check_async_param(
            getattr(device, 'send_angles', None)
        )

    @staticmethod
    def _check_async_param(fn) -> bool:
        if fn is None:
            return False
        try:
            sig = inspect.signature(fn)
            return '_async' in sig.parameters
        except (TypeError, ValueError):
            return False

    def _pace_io(self) -> None:
        if self._min_io_interval_s <= 0.0:
            return
        now = time.monotonic()
        wait = self._min_io_interval_s - (now - self._last_io_mono)
        if wait > 0.0:
            time.sleep(wait)

    def _mark_io(self) -> None:
        self._last_io_mono = time.monotonic()

    @property
    def allow_motion(self) -> bool:
        return self._allow_motion

    def get_angles(self):
        with self._lock:
            self._pace_io()
            try:
                # On the tested M5, GET_ANGLES stops replying after a gripper
                # write until GET_GRIPPER_VALUE is queried. Consume that fresh
                # feedback under the same USB lock before polling the arm.
                if self._gripper_feedback_pending:
                    value = self._device.get_gripper_value(gripper_type=1)
                    if (isinstance(value, bool) or not isinstance(value, (int, float))
                            or not math.isfinite(value) or not 0 <= value <= 100):
                        raise ValueError('Invalid pending gripper feedback')
                    self._gripper_feedback_pending = False
                    self._mark_io()
                    self._pace_io()
                return self._device.get_angles()
            finally:
                self._mark_io()

    def send_angles(self, angles_deg: Sequence[float], speed: int):
        if not self._allow_motion:
            raise PermissionError(
                'Physical motion is disabled (allow_motion=False); '
                'refusing to call send_angles().'
            )
        clamped_speed = max(1, min(100, int(speed)))
        target = [round(float(a), 2) for a in angles_deg]
        with self._lock:
            self._pace_io()
            try:
                if self._session_authorizer is not None and not self._session_authorizer():
                    raise PermissionError('Physical session revoked before USB dispatch')
                # On myCobot 280 M5, SEND_ANGLES has no serial reply frame.
                # Calling send_angles(..., _async=False) blocks for 3x0.5s=1.5s
                # waiting for a reply that never arrives. Using _async=True
                # writes the command once under lock without blocking the bus.
                if self._send_accepts_async:
                    return self._device.send_angles(
                        target, clamped_speed, _async=True
                    )
                return self._device.send_angles(target, clamped_speed)
            finally:
                self._mark_io()

    def set_fresh_mode(self, mode):
        return self._gripper_call('set_fresh_mode', mode, motion=True)

    def get_fresh_mode(self):
        return self._gripper_call('get_fresh_mode')

    @property
    def supports_gripper_stop(self):
        methods = ('gripper_stop', 'is_gripper_moving')
        if self._gripper_stop_strategy == 'hold_position':
            methods = ('get_gripper_value', 'set_gripper_value', 'is_gripper_moving')
        return all(callable(getattr(self._device, name, None)) for name in methods)

    def configure_gripper_stop(self, strategy):
        if strategy not in ('torque', 'hold_position'):
            raise ValueError('Unknown gripper stop strategy')
        self._gripper_stop_strategy = strategy

    def _gripper_call(self, name, *args, motion=False, **kwargs):
        if motion and not self._allow_motion:
            raise PermissionError('Physical gripper motion is disabled')
        with self._lock:
            self._pace_io()
            try:
                if (motion and name != 'gripper_stop' and self._session_authorizer is not None
                        and not self._session_authorizer()):
                    raise PermissionError('Physical session revoked before USB dispatch')
                if name == 'set_gripper_value':
                    return self._device_gripper_write(*args)
                return getattr(self._device, name)(*args, **kwargs)
            finally:
                self._mark_io()

    def get_gripper_value(self):
        return self._gripper_call('get_gripper_value', gripper_type=1)

    def _device_gripper_write(self, value, speed):
        if self._gripper_command_mode == 'feedback':
            from pymycobot.common import ProtocolCode
            from .serial_transport import GripperDispatch
            result = self._device._mesg(ProtocolCode.SET_GRIPPER_VALUE, value, speed, 1,
                                        _async=True)
            if result is not None:
                raise RuntimeError('Unexpected asynchronous gripper transport response')
            self._gripper_feedback_pending = True
            return GripperDispatch.SENT
        return self._device.set_gripper_value(value, speed, gripper_type=1)

    def set_gripper_value(self, value, speed):
        if (type(value) is not int or type(speed) is not int
                or not 0 <= value <= 100 or not 1 <= speed <= 30):
            raise ValueError('Invalid gripper device command')
        return self._gripper_call('set_gripper_value', value, speed,
                                  gripper_type=1, motion=True)

    def gripper_stop(self):
        if self._gripper_stop_strategy == 'torque':
            return self._gripper_call('gripper_stop', motion=True)
        if not self._allow_motion:
            raise PermissionError('Physical gripper motion is disabled')
        # A stop must remain possible after session revocation. Serialize the read
        # and hold target with every other USB operation; never substitute a
        # model-clamped or cached position. The executor still verifies stopping.
        with self._lock:
            value = self.get_gripper_value()
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or not 0 <= value <= 100):
                raise ValueError('A valid measured gripper value is required to hold')
            self._pace_io()
            try:
                # Three-argument position command verified on M5 firmware 7.2 /
                # basic 2.4. Sending is_torque=0 behaved differently on this unit.
                return self._device_gripper_write(round(value), 10)
            finally:
                self._mark_io()

    def is_gripper_moving(self):
        return self._gripper_call('is_gripper_moving')

    def stop(self):
        if not self._allow_motion:
            return None
        with self._lock:
            self._pace_io()
            try:
                if hasattr(self._device, 'stop'):
                    return self._device.stop()
                return None
            finally:
                self._mark_io()

    def is_moving(self) -> Optional[int]:
        with self._lock:
            self._pace_io()
            try:
                if hasattr(self._device, 'is_moving'):
                    return self._device.is_moving()
                return None
            finally:
                self._mark_io()

    def reset_buffers(self) -> None:
        """Flush stale serial bytes without closing or toggling the USB-CDC port."""
        with self._lock:
            ser = getattr(self._device, '__dict__', {}).get('_serial_port')
            if ser is not None:
                try:
                    if hasattr(ser, 'reset_input_buffer'):
                        ser.reset_input_buffer()
                    if hasattr(ser, 'reset_output_buffer'):
                        ser.reset_output_buffer()
                except Exception:
                    pass

    def close(self) -> None:
        """Close the underlying serial port cleanly so no duplicate FDs leak."""
        with self._lock:
            try:
                close_fn = getattr(type(self._device), 'close', None)
                if close_fn is not None:
                    self._device.close()
                    return
                ser = getattr(self._device, '__dict__', {}).get('_serial_port')
                if ser is not None and hasattr(ser, 'close'):
                    ser.close()
            except Exception:
                pass


def open_controlled_mycobot(
    port: str, baudrate: int, allow_motion: bool
) -> ControlledArm:
    """Open the serial connection and wrap it in ``ControlledArm``."""
    from pymycobot import MyCobot280  # lazy import so unit tests run without hardware
    from .serial_transport import bound_state_read_deadline
    device = MyCobot280(port, baudrate)
    bound_state_read_deadline(device)
    # The M5/Atom shares the servo bus with the gripper. Back-to-back queries
    # after gripper commands dropped replies in physical tests at 15 ms.
    arm = ControlledArm(device, allow_motion=allow_motion, min_io_interval_s=0.05,
                        gripper_command_mode='feedback')
    arm.reset_buffers()
    return arm


def _float_to_duration(seconds: float) -> Duration:
    seconds = max(0.0, float(seconds))
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return Duration(sec=sec, nanosec=nanosec)


class RobotController(Node):
    """Single-owner hardware node: /joint_states + FollowJointTrajectory."""

    def __init__(
        self,
        arm_factory: Optional[Callable[[str, int, bool], ControlledArm]] = None,
        sleep_fn: Callable[[float], None] = time.sleep,
        clock_fn: Optional[Callable[[], float]] = None,
        parameter_overrides=None,
    ):
        super().__init__('robot_control', parameter_overrides=parameter_overrides)
        self.declare_parameter('adaptive_gripper', False)
        self.declare_parameter('gripper_stop_confirmed', False)
        self.declare_parameter('gripper_stop_strategy', 'torque')
        self._gripper_stop_strategy = str(self.get_parameter('gripper_stop_strategy').value)
        if self._gripper_stop_strategy not in ('torque', 'hold_position'):
            raise ValueError('Unknown gripper stop strategy')
        self.declare_parameter('gripper_speed', 15)
        self.declare_parameter('gripper_tolerance_rad', 0.03)
        self.declare_parameter('gripper_timeout_margin_s', 3.0)
        self.declare_parameter('gripper_closed_rad', -0.74)
        self.declare_parameter('gripper_open_rad', 0.15)
        self.declare_parameter('gripper_closed_value', 0.0)
        self.declare_parameter('gripper_open_value', 100.0)
        self._adaptive_gripper = bool(self.get_parameter('adaptive_gripper').value)
        self._gripper_calibration = GripperCalibration(
            closed_rad=float(self.get_parameter('gripper_closed_rad').value),
            open_rad=float(self.get_parameter('gripper_open_rad').value),
            closed_value=float(self.get_parameter('gripper_closed_value').value),
            open_value=float(self.get_parameter('gripper_open_value').value))
        self._gripper = None
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('allow_motion', False)
        self.declare_parameter('require_session', False)
        self.declare_parameter('live_command_period_s', 0.1)
        self.declare_parameter('physical_state_timeout_s', 0.5)
        self.declare_parameter('action_name', DEFAULT_ACTION_NAME)
        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        self.declare_parameter('joint_offsets', [0.0] * NUM_JOINTS)
        self.declare_parameter('joint_signs', [1.0] * NUM_JOINTS)
        self.declare_parameter('joint_lower_limits', DEFAULT_LOWER)
        self.declare_parameter('joint_upper_limits', DEFAULT_UPPER)
        self.declare_parameter('max_joint_velocity_rad_s', 1.0)
        self.declare_parameter('default_pymycobot_speed', 15)
        self.declare_parameter('min_pymycobot_speed', 5)
        self.declare_parameter('max_pymycobot_speed', 30)
        self.declare_parameter('start_state_tolerance_rad', 0.15)
        self.declare_parameter('goal_tolerance_rad', 0.08)
        self.declare_parameter('waypoint_tolerance_rad', 0.10)
        self.declare_parameter('goal_time_tolerance_s', 2.0)
        self.declare_parameter('trajectory_timeout_scaling', 2.0)
        self.declare_parameter('min_segment_wait_s', 0.08)
        self.declare_parameter('collinear_tolerance_rad', 0.02)
        self.declare_parameter('min_waypoint_separation_rad', 0.035)
        self.declare_parameter('min_segment_duration_s', 0.20)
        self.declare_parameter('max_state_age_s', 1.5)
        self.declare_parameter('max_consecutive_failures', 15)
        self.declare_parameter('reconnect_period_s', 2.0)
        self.declare_parameter('stop_retries', 3)

        self._port = str(self.get_parameter('port').value)
        self._baudrate = int(self.get_parameter('baudrate').value)
        self._rate = float(self.get_parameter('update_rate_hz').value)
        self._allow_motion = bool(self.get_parameter('allow_motion').value)
        self._action_name = str(self.get_parameter('action_name').value)
        self._names = list(self.get_parameter('joint_names').value)
        self._offsets = [float(v) for v in self.get_parameter('joint_offsets').value]
        self._signs = [float(v) for v in self.get_parameter('joint_signs').value]
        self._lower = [float(v) for v in self.get_parameter('joint_lower_limits').value]
        self._upper = [float(v) for v in self.get_parameter('joint_upper_limits').value]
        self._max_vel = float(self.get_parameter('max_joint_velocity_rad_s').value)
        self._default_speed = int(self.get_parameter('default_pymycobot_speed').value)
        self._min_speed = int(self.get_parameter('min_pymycobot_speed').value)
        self._max_speed = int(self.get_parameter('max_pymycobot_speed').value)
        self._start_tol = float(self.get_parameter('start_state_tolerance_rad').value)
        self._goal_tol = float(self.get_parameter('goal_tolerance_rad').value)
        self._waypoint_tol = float(self.get_parameter('waypoint_tolerance_rad').value)
        self._goal_time_tol = float(self.get_parameter('goal_time_tolerance_s').value)
        self._timeout_scaling = float(
            self.get_parameter('trajectory_timeout_scaling').value
        )
        self._min_segment_wait = float(self.get_parameter('min_segment_wait_s').value)
        self._collinear_tol = float(
            self.get_parameter('collinear_tolerance_rad').value
        )
        self._min_wp_sep = float(
            self.get_parameter('min_waypoint_separation_rad').value
        )
        self._min_seg_dur = float(
            self.get_parameter('min_segment_duration_s').value
        )
        self._max_state_age = float(self.get_parameter('max_state_age_s').value)
        self._max_failures = int(self.get_parameter('max_consecutive_failures').value)
        self._reconnect_period = float(self.get_parameter('reconnect_period_s').value)
        self._stop_retries = max(1, int(self.get_parameter('stop_retries').value))

        self._validate_config()

        # Real wall-clock timestamps only.
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, False)])

        self._arm_factory = arm_factory or open_controlled_mycobot
        self._sleep = sleep_fn
        self._clock_fn = clock_fn
        self._state_lock = threading.RLock()
        self._goal_lock = threading.RLock()
        self._active_goal = False
        self._motion_fault = False
        self._hardware_live = None
        self._arm: Optional[ControlledArm] = None
        self._failures = 0
        self._last_connect_attempt: Optional[float] = None
        self._last_positions_rad: Optional[List[float]] = None
        self._last_positions_time_s: Optional[float] = None
        self._last_execution_trace: List[SegmentExecutionRecord] = []

        # Separate mutually-exclusive groups prevent concurrent timer executions
        # while still allowing the action callback to run alongside the timer.
        self._timer_cb_group = MutuallyExclusiveCallbackGroup()
        self._action_cb_group = MutuallyExclusiveCallbackGroup()
        self._pub = self.create_publisher(
            JointState,
            self.get_parameter('joint_states_topic').value,
            10,
        )

        self._connect()
        self._timer = self.create_timer(
            1.0 / self._rate, self._on_timer, callback_group=self._timer_cb_group
        )
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self._action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_cb_group,
        )

        if self._adaptive_gripper:
            self._gripper = GripperTrajectoryExecutor(
                self, self._gripper_calibration,
                speed=int(self.get_parameter('gripper_speed').value),
                tolerance=float(self.get_parameter('gripper_tolerance_rad').value),
                timeout_margin=float(self.get_parameter('gripper_timeout_margin_s').value),
                stop_confirmed=bool(self.get_parameter('gripper_stop_confirmed').value))

        if self.get_parameter('require_session').value:
            from .hardware_live import HardwareLiveController
            self._hardware_live = HardwareLiveController(self)

        mode_str = 'MOTION ENABLED' if self._allow_motion else 'READ-ONLY (allow_motion=False)'
        self.get_logger().info(
            f'RobotController started [{mode_str}] on {self._port} @ {self._rate:.1f} Hz, '
            f'action={self._action_name}, max_speed={self._max_speed}. '
            'No startup motor commands were sent.'
        )

    @property
    def last_execution_trace(self) -> List[SegmentExecutionRecord]:
        """Return instrumentation records for the most recent trajectory execution."""
        with self._state_lock:
            return list(self._last_execution_trace)

    def _validate_config(self) -> None:
        if self._rate <= 0.0:
            raise ValueError('update_rate_hz must be > 0')
        lists = {
            'joint_names': self._names,
            'joint_offsets': self._offsets,
            'joint_signs': self._signs,
            'joint_lower_limits': self._lower,
            'joint_upper_limits': self._upper,
        }
        for key, value in lists.items():
            if len(value) != NUM_JOINTS:
                raise ValueError(f'{key} must have {NUM_JOINTS} entries')
        if not check_signs(self._signs):
            raise ValueError('joint_signs entries must be +1 or -1')
        if not (1 <= self._min_speed <= self._default_speed <= self._max_speed <= 100):
            raise ValueError(
                'Speed bounds must satisfy 1 <= min_pymycobot_speed <= '
                'default_pymycobot_speed <= max_pymycobot_speed <= 100'
            )
        if self._max_vel <= 0.0:
            raise ValueError('max_joint_velocity_rad_s must be > 0')

    def _now_s(self) -> float:
        if self._clock_fn is not None:
            return float(self._clock_fn())
        return self.get_clock().now().nanoseconds * 1e-9

    def _close_arm_locked(self) -> None:
        if self._hardware_live is not None:
            self._hardware_live.connection_lost()
        if self._arm is not None:
            try:
                self._arm.close()
            except Exception:
                pass
            self._arm = None

    def _connect(self) -> bool:
        with self._state_lock:
            self._close_arm_locked()
            self._last_connect_attempt = self._now_s()
            try:
                arm = self._arm_factory(
                    self._port, self._baudrate, self._allow_motion
                )
                if not isinstance(arm, ControlledArm):
                    arm = ControlledArm(arm, allow_motion=self._allow_motion)
                arm.configure_gripper_stop(self._gripper_stop_strategy)
                self._arm = arm
                if self.get_parameter('require_session').value:
                    arm._session_authorizer = lambda: (
                        self._hardware_live is not None and
                        self._hardware_live.session.selected and not self._motion_fault and
                        not self._hardware_live.session.watchdog(
                            time.monotonic(), self._hardware_live.measured_time))
                self._failures = 0
                return True
            except Exception as exc:
                self._arm = None
                self.get_logger().error(
                    f'Cannot open {self._port}: {exc}',
                    throttle_duration_sec=5.0,
                )
                return False

    def destroy_node(self):
        if self._hardware_live is not None:
            self._hardware_live.shutdown()
            self._hardware_live.session.revoke()
        if self._gripper is not None:
            self._gripper.server.destroy()
        with self._state_lock:
            self._close_arm_locked()
        return super().destroy_node()

    def _on_timer(self) -> None:
        with self._state_lock:
            ts = self._last_positions_time_s
        if ts is not None and (self._now_s() - ts) < (0.5 / self._rate):
            return
        self.poll_once()

    def poll_once(self) -> Optional[List[float]]:
        """Read joint angles once, publish ``/joint_states`` if valid, and return radians."""
        with self._state_lock:
            arm = self._arm
            last_attempt = self._last_connect_attempt

        if arm is None:
            if (
                last_attempt is None
                or self._now_s() - last_attempt >= self._reconnect_period
            ):
                self._connect()
            return None

        try:
            raw = arm.get_angles()
        except Exception as exc:
            self.get_logger().warn(
                f'Serial exception in get_angles(): {exc}; closing port for reconnect.',
                throttle_duration_sec=5.0,
            )
            with self._state_lock:
                self._close_arm_locked()
                self._last_connect_attempt = self._now_s()
            return None

        return self.process_reading(raw)

    def process_reading(self, raw) -> Optional[List[float]]:
        """Validate one raw degree list, publish ``/joint_states``, and update cache."""
        angles = validate_angles(raw)
        if angles is None:
            with self._state_lock:
                self._failures += 1
                failures = self._failures
                arm = self._arm
                if arm is not None:
                    arm.reset_buffers()
                if failures >= self._max_failures:
                    self.get_logger().error(
                        f'{failures} consecutive invalid readings; '
                        'closing and resetting serial connection.',
                        throttle_duration_sec=5.0,
                    )
                    self._close_arm_locked()
                    self._last_connect_attempt = self._now_s()
            self.get_logger().warn(
                f'Invalid get_angles() reading dropped: {raw!r}',
                throttle_duration_sec=5.0,
            )
            return None

        positions = degrees_to_joint_positions(angles, self._offsets, self._signs)
        violations = find_limit_violations(positions, self._lower, self._upper)
        if violations:
            detail = ', '.join(
                f'{self._names[i]}={p:.3f}' for i, p in violations
            )
            self.get_logger().warn(
                f'Joint(s) outside URDF limits (published as read): {detail}',
                throttle_duration_sec=5.0,
            )

        now_msg = self.get_clock().now().to_msg()
        now_s = self._now_s()
        with self._state_lock:
            self._failures = 0
            self._last_positions_rad = list(positions)
            self._last_positions_time_s = now_s

        msg = JointState()
        msg.header.stamp = now_msg
        msg.header.frame_id = 'g_base'
        msg.name = list(self._names)
        msg.position = list(positions)
        if self._adaptive_gripper:
            gripper_position = self.read_gripper_position()
            if gripper_position is not None:
                msg.name.append(GRIPPER_JOINT)
                msg.position.append(gripper_position)
        if self._hardware_live is not None:
            with self._state_lock:
                self._hardware_live.measured_state(msg.position)
        self._pub.publish(msg)
        return positions

    def read_gripper_position(self):
        with self._state_lock:
            arm = self._arm
        if arm is None:
            return None
        try:
            return device_value_to_radians(arm.get_gripper_value(), self._gripper_calibration)
        except Exception as exc:
            self.get_logger().warn(f'Invalid gripper measurement: {exc}', throttle_duration_sec=5.0)
            return None

    def _get_fresh_positions(self) -> Optional[List[float]]:
        """Return a fresh measured joint state, polling the arm if needed."""
        with self._state_lock:
            pos = self._last_positions_rad
            ts = self._last_positions_time_s
        if pos is not None and ts is not None and (self._now_s() - ts) <= self._max_state_age:
            return list(pos)
        return self.poll_once()

    def _extract_points(self, trajectory):
        points = []
        for pt in trajectory.points:
            t_sec = duration_to_seconds(
                pt.time_from_start.sec, pt.time_from_start.nanosec
            )
            points.append((list(pt.positions), list(pt.velocities), t_sec))
        return points

    def validate_goal_request(self, goal_request: FollowJointTrajectory.Goal):
        """Run all safety, concurrency, and trajectory validations on a goal."""
        if self._hardware_live is not None and not self._hardware_live.authorize_trajectory():
            return False, 'Physical session is inactive, expired or following live.', ERROR_INVALID_GOAL, []
        if self._motion_fault:
            return False, 'Motion fault latched: verify the robot is stopped before restarting the controller.', ERROR_INVALID_GOAL, []
        if not self._allow_motion:
            return (
                False,
                'Physical motion is disabled (allow_motion=False). '
                'Pass allow_motion:=true to enable execution.',
                ERROR_INVALID_GOAL,
                [],
            )

        with self._state_lock:
            arm_connected = self._arm is not None
        if not arm_connected and not self._connect():
            return (
                False,
                f'Hardware serial connection on {self._port} is not available.',
                ERROR_INVALID_GOAL,
                [],
            )

        current_pos = self._get_fresh_positions()
        if current_pos is None:
            return (
                False,
                'Cannot obtain a valid measured joint state from the robot.',
                ERROR_INVALID_GOAL,
                [],
            )

        traj = goal_request.trajectory
        res = validate_trajectory(
            traj_joint_names=list(traj.joint_names),
            traj_points=self._extract_points(traj),
            canonical_joint_names=self._names,
            current_positions_rad=current_pos,
            lower_limits_rad=self._lower,
            upper_limits_rad=self._upper,
            max_joint_velocity_rad_s=self._max_vel,
            start_state_tolerance_rad=self._start_tol,
            default_pymycobot_speed=self._default_speed,
            min_pymycobot_speed=self._min_speed,
            max_pymycobot_speed=self._max_speed,
        )
        return res.ok, res.reason, res.error_code, res.waypoints

    def _goal_callback(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        with self._goal_lock:
            if self._active_goal:
                self.get_logger().warn(
                    'Rejecting FollowJointTrajectory goal: another goal is active.'
                )
                return GoalResponse.REJECT
            ok, reason, _, _ = self.validate_goal_request(goal_request)
            if not ok:
                self.get_logger().warn(
                    f'Rejecting FollowJointTrajectory goal: {reason}'
                )
                return GoalResponse.REJECT
            self._active_goal = True
        self.get_logger().info(
            f'Accepted FollowJointTrajectory goal with '
            f'{len(goal_request.trajectory.points)} points.'
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().warn('Cancel requested for active trajectory; stopping arm.')
        return CancelResponse.ACCEPT

    def _verified_stop(self) -> bool:
        """
        Issue ``stop()`` with retries and verify that the arm stops moving.

        Returns True if a stop command was sent and a subsequent angle reading
        succeeded (or ``is_moving()`` reported 0).
        """
        if self._hardware_live is not None:
            return self._hardware_live.stop_motion()
        with self._state_lock:
            arm = self._arm
        if arm is None:
            return False

        stopped = False
        for _ in range(self._stop_retries):
            try:
                arm.stop()
                stopped = True
            except Exception as exc:
                self.get_logger().error(f'stop() failed: {exc}')
            self._sleep(0.05)
            try:
                moving = arm.is_moving()
                if moving in (0, None):
                    break
            except Exception:
                pass

        # Read and publish final measured state after stopping.
        self.poll_once()
        return stopped

    def _publish_feedback(
        self,
        goal_handle,
        wp: WaypointSpec,
        actual_positions_rad: Sequence[float],
        elapsed_s: float,
    ) -> None:
        if not hasattr(goal_handle, 'publish_feedback'):
            return
        fb = FollowJointTrajectory.Feedback()
        fb.header.stamp = self.get_clock().now().to_msg()
        fb.joint_names = list(self._names)
        fb.desired = JointTrajectoryPoint(
            positions=list(wp.positions_rad),
            velocities=list(wp.velocities_rad_s),
            time_from_start=_float_to_duration(wp.time_from_start_s),
        )
        fb.actual = JointTrajectoryPoint(
            positions=list(actual_positions_rad),
            time_from_start=_float_to_duration(elapsed_s),
        )
        fb.error = JointTrajectoryPoint(
            positions=[
                d - a for d, a in zip(wp.positions_rad, actual_positions_rad)
            ],
            time_from_start=_float_to_duration(elapsed_s),
        )
        goal_handle.publish_feedback(fb)

    def _execute_callback(self, goal_handle) -> FollowJointTrajectory.Result:
        try:
            return self._run_trajectory(goal_handle)
        finally:
            with self._goal_lock:
                self._active_goal = False

    def _run_trajectory(self, goal_handle) -> FollowJointTrajectory.Result:
        result = FollowJointTrajectory.Result()
        with self._state_lock:
            self._last_execution_trace = []

        ok, reason, err_code, waypoints = self.validate_goal_request(
            goal_handle.request
        )
        if not ok or not waypoints:
            result.error_code = err_code if err_code != ERROR_NONE else ERROR_INVALID_GOAL
            result.error_string = reason
            goal_handle.abort()
            return result

        start_positions = self._get_fresh_positions()
        if start_positions is None:
            result.error_code = ERROR_INVALID_GOAL
            result.error_string = 'Cannot obtain initial joint state before execution.'
            goal_handle.abort()
            return result

        cmd_waypoints = coalesce_waypoints(
            start_positions_rad=start_positions,
            waypoints=waypoints,
            max_joint_velocity_rad_s=self._max_vel,
            default_pymycobot_speed=self._default_speed,
            min_pymycobot_speed=self._min_speed,
            max_pymycobot_speed=self._max_speed,
            collinear_tolerance_rad=self._collinear_tol,
            min_waypoint_separation_rad=self._min_wp_sep,
            min_segment_duration_s=self._min_seg_dur,
        )

        goal_time_tol = self._goal_time_tol
        req_tol = duration_to_seconds(
            goal_handle.request.goal_time_tolerance.sec,
            goal_handle.request.goal_time_tolerance.nanosec,
        )
        if req_tol > 0.0:
            goal_time_tol = max(goal_time_tol, req_tol)

        planned_duration = max(waypoints[-1].time_from_start_s, 0.5)
        overall_timeout = (
            planned_duration * self._timeout_scaling + goal_time_tol + 1.0
        )

        start_wall = self._now_s()
        poll_period = max(0.05, 1.0 / self._rate)
        total_cmds = len(cmd_waypoints)

        self.get_logger().info(
            f'Starting trajectory execution: {len(waypoints)} validated points '
            f'coalesced into {total_cmds} hardware command(s), '
            f'planned_duration={planned_duration:.2f}s, '
            f'start_rad={[round(v, 3) for v in start_positions]}'
        )

        for idx, wp in enumerate(cmd_waypoints):
            if self._hardware_live is not None and not self._hardware_live.authorize_trajectory():
                result.error_code = ERROR_INVALID_GOAL
                result.error_string = 'Physical session revoked'
                goal_handle.abort()
                return result
            is_last = idx == total_cmds - 1
            if goal_handle.is_cancel_requested:
                self._verified_stop()
                result.error_code = ERROR_NONE
                result.error_string = 'Trajectory canceled by client; robot stopped.'
                goal_handle.canceled()
                return result

            with self._state_lock:
                arm = self._arm
                measured_before = list(self._last_positions_rad or start_positions)
            if arm is None:
                self._verified_stop()
                result.error_code = ERROR_INVALID_GOAL
                result.error_string = 'Serial connection lost before waypoint command.'
                goal_handle.abort()
                return result

            target_deg = joint_positions_to_degrees(
                wp.positions_rad, self._offsets, self._signs
            )
            seg_send_s = self._now_s()
            elapsed_at_send = seg_send_s - start_wall
            try:
                arm.send_angles(target_deg, wp.pymycobot_speed)
            except Exception as exc:
                self.get_logger().error(f'send_angles() failed at command {idx}: {exc}')
                self._verified_stop()
                result.error_code = ERROR_INVALID_GOAL
                result.error_string = f'send_angles failed: {exc}'
                goal_handle.abort()
                return result

            self.get_logger().info(
                f'[cmd {idx + 1}/{total_cmds}] sent_at=+{elapsed_at_send:.3f}s '
                f'(planned_t={wp.time_from_start_s:.3f}s, seg_dt={wp.segment_duration_s:.3f}s) '
                f'speed={wp.pymycobot_speed} target_deg={[round(d, 2) for d in target_deg]} '
                f'measured_start_rad={[round(p, 3) for p in measured_before]}'
            )

            seg_wait = max(self._min_segment_wait, wp.segment_duration_s)
            wp_deadline = max(
                start_wall + wp.time_from_start_s,
                seg_send_s + self._min_segment_wait,
                start_wall + seg_wait,
            )
            if is_last:
                wp_deadline = start_wall + planned_duration + goal_time_tol

            tol = self._goal_tol if is_last else self._waypoint_tol
            seg_polls = 0
            last_seg_measured = measured_before
            last_seg_err = max(
                abs(d - a) for d, a in zip(wp.positions_rad, measured_before)
            )

            while True:
                if self._hardware_live is not None and not self._hardware_live.authorize_trajectory():
                    result.error_code = ERROR_INVALID_GOAL
                    result.error_string = 'Physical session revoked'
                    goal_handle.abort()
                    return result
                if goal_handle.is_cancel_requested:
                    self._verified_stop()
                    result.error_code = ERROR_NONE
                    result.error_string = 'Trajectory canceled by client; robot stopped.'
                    goal_handle.canceled()
                    return result

                measured = self.poll_once()
                seg_polls += 1
                now_s = self._now_s()
                elapsed = now_s - start_wall

                if self._arm is None:
                    self._verified_stop()
                    result.error_code = ERROR_INVALID_GOAL
                    result.error_string = (
                        'Lost communication with robot during trajectory execution.'
                    )
                    goal_handle.abort()
                    return result

                if measured is not None:
                    last_seg_measured = list(measured)
                    desired_wp = interpolate_desired_waypoint(
                        waypoints, start_positions, elapsed
                    )
                    self._publish_feedback(goal_handle, desired_wp, measured, elapsed)
                    if goal_handle.is_cancel_requested:
                        self._verified_stop()
                        result.error_code = ERROR_NONE
                        result.error_string = (
                            'Trajectory canceled by client; robot stopped.'
                        )
                        goal_handle.canceled()
                        return result

                    max_err = max(
                        abs(d - a) for d, a in zip(wp.positions_rad, measured)
                    )
                    last_seg_err = max_err
                    # Break as soon as the arm reaches tolerance so intermediate
                    # via-points transition smoothly without stopping to wait.
                    if max_err <= tol:
                        # A final target inside tolerance can precede actual
                        # motion on the device. Do not declare completion while
                        # firmware still reports motion or cannot confirm rest.
                        if not is_last or arm.is_moving() == 0:
                            break

                if elapsed > overall_timeout:
                    self.get_logger().error(
                        f'Trajectory timed out after {elapsed:.2f}s '
                        f'(limit {overall_timeout:.2f}s); stopping arm.'
                    )
                    self._verified_stop()
                    result.error_code = ERROR_GOAL_TOLERANCE_VIOLATED
                    result.error_string = (
                        f'Trajectory execution timed out after {elapsed:.2f}s.'
                    )
                    goal_handle.abort()
                    return result

                if now_s >= wp_deadline:
                    break

                sleep_dt = min(poll_period, max(0.01, wp_deadline - now_s))
                self._sleep(sleep_dt)

            actual_seg_dt = self._now_s() - seg_send_s
            rec = SegmentExecutionRecord(
                cmd_index=idx,
                total_commands=total_cmds,
                sent_time_s=seg_send_s,
                elapsed_at_send_s=elapsed_at_send,
                planned_time_from_start_s=wp.time_from_start_s,
                planned_segment_duration_s=wp.segment_duration_s,
                actual_segment_duration_s=actual_seg_dt,
                pymycobot_speed=wp.pymycobot_speed,
                target_positions_rad=list(wp.positions_rad),
                target_angles_deg=[round(float(d), 2) for d in target_deg],
                measured_start_rad=list(measured_before),
                measured_end_rad=list(last_seg_measured),
                max_error_rad=last_seg_err,
                poll_count=seg_polls,
            )
            with self._state_lock:
                self._last_execution_trace.append(rec)

            self.get_logger().info(
                f'[cmd {idx + 1}/{total_cmds} done] actual_dt={actual_seg_dt:.3f}s '
                f'(planned_dt={wp.segment_duration_s:.3f}s) polls={seg_polls} '
                f'max_err={last_seg_err:.4f} rad '
                f'measured_end_rad={[round(p, 3) for p in last_seg_measured]}'
            )

        # Final position verification against the last waypoint.
        final_measured = self.poll_once() or self._get_fresh_positions()
        if final_measured is None:
            self._verified_stop()
            result.error_code = ERROR_INVALID_GOAL
            result.error_string = 'Could not read final robot position after motion.'
            goal_handle.abort()
            return result

        final_wp = cmd_waypoints[-1]
        final_err = max(
            abs(d - a) for d, a in zip(final_wp.positions_rad, final_measured)
        )
        try:
            final_moving = arm.is_moving()
        except Exception:
            final_moving = None
        if final_err > self._goal_tol or final_moving != 0:
            self.get_logger().error(
                f'Final verification failed: error={final_err:.3f} rad, '
                f'tolerance={self._goal_tol:.3f} rad, moving={final_moving}; stopping arm.'
            )
            self._verified_stop()
            result.error_code = ERROR_GOAL_TOLERANCE_VIOLATED
            result.error_string = (
                f'Final joint error {final_err:.3f} rad exceeds '
                f'tolerance {self._goal_tol:.3f} rad.'
                if final_err > self._goal_tol else
                'Final robot standstill could not be verified.'
            )
            goal_handle.abort()
            return result

        total_elapsed = self._now_s() - start_wall
        result.error_code = ERROR_NONE
        result.error_string = 'Trajectory executed and verified.'
        goal_handle.succeed()
        self.get_logger().info(
            f'Trajectory succeeded in {total_elapsed:.2f}s '
            f'(planned {planned_duration:.2f}s, {total_cmds} command(s), '
            f'max final error {final_err:.4f} rad).'
        )
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
