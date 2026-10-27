"""Integration-only USB emulator. Never opens a serial port."""

import math
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from mycobot_280_hardware.robot_control_node import RobotController, ControlledArm


class FakeUSB:
    def __init__(self):
        self.angles = [math.degrees(v) for v in [-0.4, 0.8, -1.2, 0.4, 0.8, 0.0]]
        self.gripper = 100
        self.refresh = 0

    def get_angles(self):
        return self.angles[:]

    def send_angles(self, values, speed, _async=False):
        assert 1 <= speed <= 10
        self.angles = list(values)
        return 1

    def is_moving(self):
        return 0

    def stop(self):
        return 1

    def close(self):
        pass

    def set_fresh_mode(self, mode):
        self.refresh = mode

    def get_fresh_mode(self):
        return self.refresh

    def get_gripper_value(self, gripper_type=1):
        return self.gripper

    def set_gripper_value(self, value, speed, gripper_type=1):
        assert 1 <= speed <= 10
        self.gripper = value
        return 1

    def gripper_stop(self):
        return 1

    def is_gripper_moving(self):
        return 0


def main():
    rclpy.init()
    device = FakeUSB()
    node = RobotController(
        arm_factory=lambda *args: ControlledArm(device, allow_motion=True),
        parameter_overrides=[
            Parameter("allow_motion", value=True),
            Parameter("require_session", value=True),
            Parameter("adaptive_gripper", value=True),
            Parameter("gripper_stop_confirmed", value=True),
            Parameter("max_joint_velocity_rad_s", value=0.1),
            Parameter("max_pymycobot_speed", value=10),
            Parameter("default_pymycobot_speed", value=5),
            Parameter("gripper_speed", value=10),
        ],
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
