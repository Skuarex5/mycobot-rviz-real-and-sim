"""
Read-only monitor: publishes the real myCobot 280 M5 pose as JointState.

SAFETY: this node only ever calls ``get_angles()`` on the device. It never
powers on, releases, or commands servos. The device handle is wrapped in
``ReadOnlyArm`` so no other method is reachable from the node code.
"""

from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from mycobot_280_hardware.joint_conversion import (
    check_signs,
    degrees_to_joint_positions,
    find_limit_violations,
    NUM_JOINTS,
    validate_angles,
)

DEFAULT_JOINT_NAMES = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
]
# Defaults mirror the URDF limits (mycobot_description / Gazebo model).
DEFAULT_LOWER = [-2.9321, -2.4434, -2.6179, -2.6179, -2.7052, -3.14159]
DEFAULT_UPPER = [2.9321, 2.4434, 2.6179, 2.6179, 2.7925, 3.14159]


class ReadOnlyArm:
    """Expose only the read-only ``get_angles`` call of a pymycobot handle."""

    def __init__(self, device):
        self._device = device

    def get_angles(self):
        return self._device.get_angles()


def open_mycobot(port: str, baudrate: int) -> ReadOnlyArm:
    """Open the serial connection and return a read-only handle."""
    from pymycobot import MyCobot280  # imported lazily: not needed for tests
    return ReadOnlyArm(MyCobot280(port, baudrate))


class RobotMonitor(Node):
    """Poll joint angles and publish ``/joint_states`` (radians)."""

    def __init__(self, arm_factory: Optional[Callable[[str, int], object]] = None):
        super().__init__('robot_monitor')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        self.declare_parameter('joint_offsets', [0.0] * NUM_JOINTS)
        self.declare_parameter('joint_signs', [1.0] * NUM_JOINTS)
        self.declare_parameter('joint_lower_limits', DEFAULT_LOWER)
        self.declare_parameter('joint_upper_limits', DEFAULT_UPPER)
        self.declare_parameter('max_consecutive_failures', 5)
        self.declare_parameter('reconnect_period_s', 2.0)
        self.declare_parameter('joint_states_topic', 'joint_states')

        self._port = self.get_parameter('port').value
        self._baudrate = int(self.get_parameter('baudrate').value)
        rate = float(self.get_parameter('update_rate_hz').value)
        self._names = list(self.get_parameter('joint_names').value)
        self._offsets = [float(v) for v in self.get_parameter('joint_offsets').value]
        self._signs = [float(v) for v in self.get_parameter('joint_signs').value]
        self._lower = [float(v) for v in self.get_parameter('joint_lower_limits').value]
        self._upper = [float(v) for v in self.get_parameter('joint_upper_limits').value]
        self._max_failures = int(self.get_parameter('max_consecutive_failures').value)
        self._reconnect_period = float(self.get_parameter('reconnect_period_s').value)

        self._validate_config(rate)

        # Real wall-clock timestamps only: this node ignores simulated time.
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, False)])

        self._arm_factory = arm_factory or open_mycobot
        self._arm = None
        self._failures = 0
        self._last_connect_attempt = None

        self._pub = self.create_publisher(
            JointState, self.get_parameter('joint_states_topic').value, 10)
        self._connect()
        self._timer = self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(
            f'READ-ONLY monitor on {self._port} @ {rate:.1f} Hz. '
            'No motor commands are sent. offsets/signs are NOT calibrated '
            'until compared against the physical robot.')

    def _validate_config(self, rate: float) -> None:
        if rate <= 0.0:
            raise ValueError('update_rate_hz must be > 0')
        lists = {
            'joint_names': self._names, 'joint_offsets': self._offsets,
            'joint_signs': self._signs, 'joint_lower_limits': self._lower,
            'joint_upper_limits': self._upper,
        }
        for key, value in lists.items():
            if len(value) != NUM_JOINTS:
                raise ValueError(f'{key} must have {NUM_JOINTS} entries')
        if not check_signs(self._signs):
            raise ValueError('joint_signs entries must be +1 or -1')

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _connect(self) -> bool:
        self._last_connect_attempt = self._now_s()
        try:
            self._arm = self._arm_factory(self._port, self._baudrate)
            self._failures = 0
            return True
        except Exception as exc:  # serial/import errors
            self._arm = None
            self.get_logger().error(
                f'Cannot open {self._port}: {exc}',
                throttle_duration_sec=5.0)
            return False

    def _on_timer(self) -> None:
        if self._arm is None:
            if self._now_s() - self._last_connect_attempt >= self._reconnect_period:
                self._connect()
            return
        try:
            raw = self._arm.get_angles()
        except Exception as exc:
            self.get_logger().warn(
                f'get_angles failed: {exc}', throttle_duration_sec=5.0)
            raw = None
        self.process_reading(raw)

    def process_reading(self, raw) -> bool:
        """Validate one raw reading and publish it. Return True if published."""
        angles = validate_angles(raw)
        if angles is None:
            self._failures += 1
            self.get_logger().warn(
                f'Invalid get_angles() reading, sample dropped: {raw!r}',
                throttle_duration_sec=5.0)
            if self._failures >= self._max_failures:
                self.get_logger().error(
                    'Too many consecutive failures; reconnecting.',
                    throttle_duration_sec=5.0)
                self._arm = None
                self._last_connect_attempt = self._now_s()
            return False
        self._failures = 0

        positions = degrees_to_joint_positions(angles, self._offsets, self._signs)
        violations = find_limit_violations(positions, self._lower, self._upper)
        if violations:
            detail = ', '.join(
                f'{self._names[i]}={p:.3f}' for i, p in violations)
            self.get_logger().warn(
                f'Joint(s) outside URDF limits (published as read): {detail}',
                throttle_duration_sec=5.0)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'g_base'
        msg.name = list(self._names)
        msg.position = positions
        self._pub.publish(msg)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
