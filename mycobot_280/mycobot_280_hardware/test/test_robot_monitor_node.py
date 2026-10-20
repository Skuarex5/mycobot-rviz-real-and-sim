"""Node tests with a fake (read-only) arm. Skipped if ROS 2 is not sourced."""

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('sensor_msgs')

from mycobot_280_hardware.robot_monitor_node import RobotMonitor  # noqa: E402


class FakeArm:
    def __init__(self, readings):
        self.readings = list(readings)
        self.calls = []

    def get_angles(self):
        self.calls.append('get_angles')
        return self.readings.pop(0) if self.readings else None

    def __getattr__(self, name):
        raise AssertionError(f'unexpected device call: {name}')


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_node(readings):
    arm = FakeArm(readings)
    node = RobotMonitor(arm_factory=lambda port, baud: arm)
    published = []
    node._pub.publish = published.append
    return node, arm, published


def test_publishes_radians_names_and_stamp():
    node, arm, published = make_node([])
    assert node.process_reading([180, 0, 0, 0, 0, 0])
    msg = published[0]
    assert list(msg.name) == node._names
    assert msg.position[0] == pytest.approx(3.14159265, abs=1e-6)
    assert msg.header.stamp.sec > 0 or msg.header.stamp.nanosec > 0
    node.destroy_node()


def test_invalid_readings_are_not_published():
    node, arm, published = make_node([])
    for raw in (None, [], [0] * 5, -1, [float('nan')] * 6):
        assert not node.process_reading(raw)
    assert published == []
    node.destroy_node()


def test_timer_only_calls_get_angles():
    node, arm, published = make_node([[0] * 6, None, [10] * 6])
    for _ in range(3):
        node._on_timer()
    assert set(arm.calls) == {'get_angles'}
    assert len(published) == 2
    node.destroy_node()


def test_use_sim_time_is_false():
    node, arm, published = make_node([])
    assert node.get_parameter('use_sim_time').value is False
    node.destroy_node()
