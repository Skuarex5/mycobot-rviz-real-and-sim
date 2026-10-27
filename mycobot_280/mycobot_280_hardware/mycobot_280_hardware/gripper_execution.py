"""One-joint trajectory action using the existing controller's single USB owner."""

import math
import time

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from trajectory_msgs.msg import JointTrajectoryPoint

from .gripper_conversion import GRIPPER_ACTION, GRIPPER_JOINT, radians_to_device_value
from .serial_transport import gripper_write_sent


def coalesce_gripper_points(start, points):
    """Keep endpoints of monotonic runs, including every direction reversal.

    The device executes position commands with its own speed profile. Dense
    planner samples would repeatedly restart it. All original points must be
    validated before calling this function; retained timestamps stay unchanged.
    """
    result = []
    previous = start
    direction = 0
    for index, (target, arrival) in enumerate(points):
        step = target - previous
        next_direction = (step > 0) - (step < 0)
        if direction and next_direction and next_direction != direction:
            result.append(points[index - 1])
        if next_direction:
            direction = next_direction
        previous = target
    if points:
        result.append(points[-1])
    return result


class GripperTrajectoryExecutor:
    def __init__(
        self,
        node,
        calibration,
        speed=15,
        tolerance=0.03,
        max_velocity=0.5,
        timeout_margin=3.0,
        stop_confirmed=False,
    ):
        self.node = node
        self.calibration = calibration
        self.speed = speed
        self.tolerance = tolerance
        self.max_velocity = max_velocity
        self.timeout_margin = timeout_margin
        self.stop_confirmed = stop_confirmed
        if (
            not 1 <= speed <= 30
            or not all(
                math.isfinite(x) and x > 0
                for x in (tolerance, max_velocity, timeout_margin)
            )
            or max_velocity > 0.5
        ):
            raise ValueError("Invalid gripper execution limits")
        self.server = ActionServer(
            node,
            FollowJointTrajectory,
            GRIPPER_ACTION,
            execute_callback=self.execute,
            goal_callback=self.goal_callback,
            cancel_callback=lambda handle: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )

    def validate(self, goal):
        node = self.node
        if node._hardware_live is not None and not node._hardware_live.authorize_trajectory():
            raise ValueError('Physical session inactive or following live')
        if node._motion_fault:
            raise ValueError(
                "Motion fault latched: verify stopping before restarting the controller"
            )
        if not node._allow_motion:
            raise ValueError("Physical motion is disabled")
        if (
            not self.stop_confirmed
            or node._arm is None
            or not node._arm.supports_gripper_stop
        ):
            raise ValueError(
                "Specific gripper stopping must be supported and confirmed on the actual firmware"
            )
        current = node.read_gripper_position()
        if (
            current is None
            or not self.calibration.lower_rad <= current <= self.calibration.upper_rad
        ):
            raise ValueError(
                "A valid measured gripper position within model limits is required"
            )
        traj = goal.trajectory
        if list(traj.joint_names) != [GRIPPER_JOINT] or not traj.points:
            raise ValueError("Expected a trajectory for gripper_controller only")
        if traj.header.stamp.sec or traj.header.stamp.nanosec:
            raise ValueError("Scheduled gripper trajectories are not supported")
        if goal.path_tolerance or goal.goal_tolerance:
            raise ValueError(
                "Custom gripper tolerances are not supported; use configured tolerance"
            )
        if goal.goal_time_tolerance.sec or goal.goal_time_tolerance.nanosec:
            raise ValueError("Custom goal time tolerance is not supported")
        result = []
        last_q, last_t = current, 0.0
        for idx, p in enumerate(traj.points):
            if len(p.positions) != 1 or p.effort:
                raise ValueError("Each point must contain one position and no effort")
            q = p.positions[0]
            radians_to_device_value(q, self.calibration)
            if (
                p.time_from_start.sec < 0
                or not 0 <= p.time_from_start.nanosec < 1000000000
            ):
                raise ValueError("Invalid trajectory timestamp")
            t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            for values, limit in (
                (p.velocities, self.max_velocity),
                (p.accelerations, 1.0),
            ):
                if values and (
                    len(values) != 1
                    or not math.isfinite(values[0])
                    or abs(values[0]) > limit
                ):
                    raise ValueError("Invalid gripper derivative")
            if idx == 0 and t == 0 and abs(q - current) <= self.tolerance:
                last_q = q
                continue
            if t <= last_t or abs(q - last_q) / (t - last_t) > self.max_velocity + 1e-9:
                raise ValueError(
                    "Non-monotonic timestamps or excessive gripper velocity"
                )
            result.append((q, t))
            last_q, last_t = q, t
        if not result:
            raise ValueError("Trajectory must contain a positive-duration segment")
        return result

    def goal_callback(self, goal):
        with self.node._goal_lock:
            if self.node._active_goal:
                return GoalResponse.REJECT
            try:
                self.validate(goal)
            except (ValueError, RuntimeError) as exc:
                self.node.get_logger().warn(f"Rejecting gripper trajectory: {exc}")
                return GoalResponse.REJECT
            self.node._active_goal = True
        return GoalResponse.ACCEPT

    def _now_s(self):
        return (
            float(self.node._clock_fn())
            if self.node._clock_fn is not None
            else time.monotonic()
        )

    def stop(self):
        arm = self.node._arm
        verified = False
        try:
            if arm is None or not gripper_write_sent(arm.gripper_stop()):
                return False
            deadline = self._now_s() + 1.0
            while self._now_s() < deadline:
                if arm.is_gripper_moving() == 0:
                    measured = arm.get_gripper_value()
                    if (not isinstance(measured, bool)
                            and isinstance(measured, (int, float))
                            and math.isfinite(measured) and 0 <= measured <= 100):
                        verified = True
                        return True
                self.node._sleep(0.05)
        except Exception as exc:
            self.node.get_logger().error(f"Gripper stop failed: {exc}")
        finally:
            if not verified:
                with self.node._goal_lock:
                    self.node._motion_fault = True
                self.node.get_logger().error(
                    "Gripper stop NOT verified: both arm and gripper motion are latched off. "
                    "Verify the physical robot is stopped before restarting the controller."
                )
        return False

    def _feedback(self, handle, desired, measured, elapsed):
        fb = FollowJointTrajectory.Feedback()
        fb.header.stamp = self.node.get_clock().now().to_msg()
        fb.joint_names = [GRIPPER_JOINT]
        for name, q in (
            ("desired", desired),
            ("actual", measured),
            ("error", desired - measured),
        ):
            p = JointTrajectoryPoint()
            p.positions = [q]
            p.time_from_start.sec = int(elapsed)
            p.time_from_start.nanosec = int((elapsed - int(elapsed)) * 1e9)
            setattr(fb, name, p)
        handle.publish_feedback(fb)

    def execute(self, handle):
        result = FollowJointTrajectory.Result()
        started_motion = False
        try:
            points = self.validate(handle.request)  # Re-read after acceptance.
            start = self._now_s()
            previous = self.node.read_gripper_position()
            points = coalesce_gripper_points(previous, points)
            previous_time = 0.0
            arm = self.node._arm
            for index, (target, arrival) in enumerate(points):
                if handle.is_cancel_requested:
                    stopped = self.stop()
                    result.error_code = result.PATH_TOLERANCE_VIOLATED
                    result.error_string = (
                        "Canceled; gripper stop verified"
                        if stopped
                        else "Cancel requested; gripper stop NOT verified"
                    )
                    handle.canceled() if stopped else handle.abort()
                    return result
                # Serial connection must remain the same throughout execution.
                if arm is not self.node._arm:
                    raise RuntimeError(
                        "Serial connection changed during gripper execution"
                    )
                started_motion = True
                reply = arm.set_gripper_value(
                    radians_to_device_value(target, self.calibration), self.speed
                )
                if not gripper_write_sent(reply):
                    raise RuntimeError("Gripper command was not dispatched")
                self.node.get_logger().info(
                    f'Gripper command {index + 1}/{len(points)}: '
                    f'target={target:.4f} rad, device='
                    f'{radians_to_device_value(target, self.calibration)}, '
                    f'planned_t={arrival:.3f}s'
                )
                deadline = start + arrival + self.timeout_margin
                while True:
                    if self.node._hardware_live is not None and not self.node._hardware_live.authorize_trajectory():
                        raise RuntimeError('Physical session revoked')
                    if handle.is_cancel_requested:
                        stopped = self.stop()
                        result.error_code = result.PATH_TOLERANCE_VIOLATED
                        result.error_string = (
                            "Canceled; gripper stop verified"
                            if stopped
                            else "Cancel requested; gripper stop NOT verified"
                        )
                        handle.canceled() if stopped else handle.abort()
                        return result
                    if arm is not self.node._arm:
                        raise RuntimeError(
                            "Serial connection lost during gripper execution"
                        )
                    measured = self.node.read_gripper_position()
                    if measured is None:
                        raise RuntimeError(
                            "Missing gripper measurement during execution"
                        )
                    elapsed = self._now_s() - start
                    fraction = min(
                        1.0,
                        max(0.0, (elapsed - previous_time) / (arrival - previous_time)),
                    )
                    self._feedback(
                        handle,
                        previous + fraction * (target - previous),
                        measured,
                        elapsed,
                    )
                    if ((index == len(points) - 1 or elapsed >= arrival)
                            and abs(measured - target) <= self.tolerance
                            and arm.is_gripper_moving() == 0):
                        break
                    if self._now_s() >= deadline:
                        raise RuntimeError(
                            f"Gripper did not converge before the deadline: "
                            f"target={target:.4f}, measured={measured:.4f} rad"
                        )
                    self.node._sleep(0.05)
                previous, previous_time = measured, arrival
            handle.succeed()
            result.error_code = result.SUCCESSFUL
            return result
        except Exception as exc:
            # Reads may fail before command dispatch; stopping is still required once
            # a goal is active, but never from the constructor/read-only monitor.
            stopped = self.stop() if self.node._allow_motion else False
            result.error_code = (
                result.GOAL_TOLERANCE_VIOLATED
                if started_motion
                else result.INVALID_GOAL
            )
            result.error_string = f"{exc}; gripper stop verified={stopped}"
            handle.abort()
            return result
        finally:
            with self.node._goal_lock:
                self.node._active_goal = False
