"""Physical-gripper behavior exercised with a fake serial device, never hardware."""

import math
import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.action import GoalResponse
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from mycobot_280_hardware.robot_control_node import RobotController, ControlledArm


class Device:
    def __init__(self):
        self.value = 83  # approximately zero radians
        self.commands = []
        self.stops = 0
        self.follow = True
        self.invalid = False
        self.moving = 0

    def get_angles(self):
        return [0.0] * 6

    def get_gripper_value(self, gripper_type=1):
        return None if self.invalid else self.value

    def set_gripper_value(self, value, speed, gripper_type=1):
        self.commands.append((value, speed))
        if self.follow:
            self.value = value
        return 1

    def gripper_stop(self):
        self.stops += 1
        self.moving = 0
        return 1

    def is_gripper_moving(self):
        return self.moving

    def close(self):
        pass


class Handle:
    def __init__(self, goal, cancel=False):
        self.request = goal
        self.is_cancel_requested = cancel
        self.status = None
        self.feedbacks = []

    def succeed(self):
        self.status = "succeeded"

    def abort(self):
        self.status = "aborted"

    def canceled(self):
        self.status = "canceled"

    def publish_feedback(self, fb):
        self.feedbacks.append(fb)


def goal(position=-0.2, duration=1):
    g = FollowJointTrajectory.Goal()
    g.trajectory.joint_names = ["gripper_controller"]
    p = JointTrajectoryPoint()
    p.positions = [float(position)]
    p.time_from_start.sec = duration
    g.trajectory.points = [p]
    return g


@pytest.fixture
def setup_node():
    if not rclpy.ok():
        rclpy.init()
    nodes = []

    def make(allow=False, confirmed=True):
        dev = Device()
        clock = [100.0]

        def sleep(dt):
            clock[0] += dt

        assert hasattr(ControlledArm, "get_gripper_value"), (
            "physical gripper support is missing"
        )
        node = RobotController(
            arm_factory=lambda *args: ControlledArm(
                dev, allow_motion=allow, min_io_interval_s=0
            ),
            clock_fn=lambda: clock[0],
            sleep_fn=sleep,
            parameter_overrides=[
                Parameter("adaptive_gripper", value=True),
                Parameter("allow_motion", value=allow),
                Parameter("gripper_stop_confirmed", value=confirmed),
            ],
        )
        nodes.append(node)
        return node, dev

    yield make
    for n in nodes:
        n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_startup_and_dry_run_do_not_move_gripper(setup_node):
    node, dev = setup_node()
    assert not dev.commands and dev.stops == 0
    assert node._gripper.goal_callback(goal()) == GoalResponse.REJECT
    with pytest.raises(PermissionError):
        node._arm.set_gripper_value(50, 15)


def test_gripper_execution_uses_measured_state_and_shared_reservation(setup_node):
    node, dev = setup_node(True)
    assert node._gripper.goal_callback(goal()) == GoalResponse.ACCEPT
    assert node._gripper.goal_callback(goal()) == GoalResponse.REJECT
    assert node._goal_callback(FollowJointTrajectory.Goal()) == GoalResponse.REJECT
    h = Handle(goal())
    result = node._gripper.execute(h)
    assert result.error_code == 0 and h.status == "succeeded"
    assert dev.commands and h.feedbacks and not node._active_goal
    assert abs(h.feedbacks[-1].actual.positions[0] + 0.2) < 0.03


def test_dense_gripper_path_does_not_send_stop_and_go_micro_targets(setup_node):
    node, dev = setup_node(True)
    request = goal()
    request.trajectory.points = [
        JointTrajectoryPoint(positions=[-.01 * i],
                             time_from_start=Duration(sec=i))
        for i in range(1, 21)
    ]
    handle = Handle(request)
    result = node._gripper.execute(handle)
    assert result.error_code == 0 and handle.status == 'succeeded'
    assert len(dev.commands) == 1
    assert dev.commands[-1][0] == 61


def test_settled_gripper_finishes_without_waiting_for_nominal_final_timestamp(setup_node):
    node, dev = setup_node(True)
    started = node._gripper._now_s()
    result = node._gripper.execute(Handle(goal(-.2, duration=20)))
    assert result.error_code == 0
    assert node._gripper._now_s() - started < 1.


def test_gripper_coalescing_preserves_reversals_and_checks_all_original_points(setup_node):
    node, dev = setup_node(True)
    request = goal()
    request.trajectory.points = [
        JointTrajectoryPoint(positions=[q], time_from_start=Duration(sec=i))
        for i, q in enumerate([-.1, -.2, -.1, 0.], 1)
    ]
    handle = Handle(request)
    result = node._gripper.execute(handle)
    assert result.error_code == 0 and handle.status == 'succeeded'
    assert [value for value, speed in dev.commands] == [61, 83]
    dev.commands.clear()
    request.trajectory.points[1].positions = [float('nan')]
    assert node._gripper.goal_callback(request) == GoalResponse.REJECT
    assert not dev.commands


@pytest.mark.parametrize(
    "target,duration", [(-0.74, 1), (0.2, 1), (math.nan, 1), (-0.2, 0)]
)
def test_bad_gripper_trajectories_rejected(setup_node, target, duration):
    node, dev = setup_node(True)
    assert node._gripper.goal_callback(goal(target, duration)) == GoalResponse.REJECT
    assert not dev.commands


def test_unconfirmed_stop_and_missing_measurement_reject_motion(setup_node):
    node, dev = setup_node(True, False)
    assert node._gripper.goal_callback(goal()) == GoalResponse.REJECT
    node._gripper.stop_confirmed = True
    dev.invalid = True
    assert node._gripper.goal_callback(goal()) == GoalResponse.REJECT
    assert not dev.commands


@pytest.mark.parametrize("failure", ["cancel", "timeout", "read", "stop"])
def test_gripper_failures_stop_and_never_succeed(setup_node, failure):
    node, dev = setup_node(True)
    assert node._gripper.goal_callback(goal()) == GoalResponse.ACCEPT
    h = Handle(goal(), cancel=failure == "cancel")
    dev.follow = False
    if failure == "read":
        dev.invalid = True
    if failure == "stop":
        h.is_cancel_requested = True
        dev.gripper_stop = lambda: None
        dev.is_gripper_moving = lambda: 1
    result = node._gripper.execute(h)
    assert result.error_code != 0 and h.status in ("canceled", "aborted")
    assert not node._active_goal
    if failure != "stop":
        assert dev.stops >= 1


def test_joint_states_contains_only_measured_gripper_and_arm_cache_stays_six(
    setup_node,
):
    node, dev = setup_node()
    msgs = []
    node._pub.publish = msgs.append
    node.poll_once()
    assert len(node._last_positions_rad) == 6
    assert msgs[-1].name[-1] == "gripper_controller"
    dev.invalid = True
    node.poll_once()
    assert len(msgs[-1].name) == 6


def test_read_only_monitor_reads_gripper_without_commands():
    from mycobot_280_hardware.robot_monitor_node import RobotMonitor, ReadOnlyArm

    if not rclpy.ok():
        rclpy.init()
    dev = Device()
    node = RobotMonitor(
        arm_factory=lambda *args: ReadOnlyArm(dev),
        parameter_overrides=[Parameter("adaptive_gripper", value=True)],
    )
    try:
        msgs = []
        node._pub.publish = msgs.append
        node.process_reading([0] * 6)
        assert msgs[-1].name[-1] == "gripper_controller"
        assert not dev.commands and dev.stops == 0
        dev.invalid = True
        node.process_reading([0] * 6)
        assert len(msgs[-1].name) == 6
        assert not hasattr(node._arm, "set_gripper_value")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_ros_action_can_cancel_while_execution_callback_is_running(setup_node):
    import threading
    import time
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor

    node, dev = setup_node(True)
    dev.follow = False
    node._clock_fn = time.monotonic
    node._sleep = time.sleep
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    client_node = rclpy.create_node("gripper_cancel_test")
    client = ActionClient(
        client_node,
        FollowJointTrajectory,
        "/mycobot280m5_gripper_controller/follow_joint_trajectory",
    )
    thread.start()
    try:
        assert client.wait_for_server(timeout_sec=5)
        future = client.send_goal_async(goal())
        rclpy.spin_until_future_complete(client_node, future, timeout_sec=5)
        handle = future.result()
        assert handle is not None and handle.accepted
        deadline = time.monotonic() + 2
        while not dev.commands and time.monotonic() < deadline:
            time.sleep(0.01)
        assert dev.commands
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(client_node, cancel, timeout_sec=2)
        assert cancel.done() and cancel.result().goals_canceling
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(client_node, result, timeout_sec=2)
        assert result.done() and result.result().status == 5  # STATUS_CANCELED
        assert dev.stops >= 1 and not node._active_goal
    finally:
        client.destroy()
        client_node.destroy_node()
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)


def test_unverified_gripper_stop_latches_fault_for_both_actions(setup_node):
    node, dev = setup_node(True)
    assert node._gripper.goal_callback(goal()) == GoalResponse.ACCEPT
    dev.gripper_stop = lambda: None
    dev.is_gripper_moving = lambda: 1
    result = node._gripper.execute(Handle(goal(), cancel=True))
    assert result.error_code != 0
    assert node._gripper.goal_callback(goal()) == GoalResponse.REJECT
    assert node.validate_goal_request(FollowJointTrajectory.Goal())[0] is False
    assert node._motion_fault


def test_gripper_deadlines_use_monotonic_time_not_ros_clock(setup_node, monkeypatch):
    import mycobot_280_hardware.gripper_execution as execution

    node, dev = setup_node(True)
    node._clock_fn = None
    node._now_s = lambda: -10000.0
    monkeypatch.setattr(execution.time, "monotonic", lambda: 123.0)
    assert node._gripper._now_s() == 123.0


def test_hold_stop_uses_measured_position_even_after_session_is_revoked():
    dev = Device()
    dev.value = 17
    arm = ControlledArm(dev, allow_motion=True, gripper_stop_strategy="hold_position")
    arm._session_authorizer = lambda: False
    assert arm.gripper_stop() == 1
    assert dev.commands == [(17, 10)]
    assert dev.stops == 0  # The torque stop releases this device instead of holding.


@pytest.mark.parametrize("value", [None, True, -1, 101, float("nan")])
def test_hold_stop_never_sends_a_fabricated_position(value):
    dev = Device()
    dev.value = value
    arm = ControlledArm(dev, allow_motion=True, gripper_stop_strategy="hold_position")
    with pytest.raises(ValueError):
        arm.gripper_stop()
    assert not dev.commands and not dev.stops


def test_read_only_blocks_hold_stop_writes():
    dev = Device()
    arm = ControlledArm(dev, gripper_stop_strategy="hold_position")
    with pytest.raises(PermissionError):
        arm.gripper_stop()
    assert not dev.commands and not dev.stops


def test_driver_applies_selected_stop_strategy_to_its_usb_owner():
    if not rclpy.ok():
        rclpy.init()
    dev = Device()
    dev.value = 17
    node = RobotController(
        arm_factory=lambda *args: ControlledArm(dev, allow_motion=True),
        parameter_overrides=[
            Parameter("allow_motion", value=True),
            Parameter("require_session", value=True),
            Parameter("gripper_stop_strategy", value="hold_position"),
        ],
    )
    try:
        assert node._arm.gripper_stop() == 1
        assert dev.commands == [(17, 10)] and not dev.stops
    finally:
        node.destroy_node()


@pytest.mark.parametrize('feedback', [22, None, -1, True])
def test_arm_poll_consumes_pending_gripper_feedback_before_reading_angles(feedback):
    class PendingDevice(Device):
        pending = False

        def _mesg(self, *args, **kwargs):
            self.pending = True

        def get_gripper_value(self, gripper_type=1):
            if type(feedback) is int and 0 <= feedback <= 100:
                self.pending = False
            return feedback

        def get_angles(self):
            return -1 if self.pending else [0.] * 6

    dev = PendingDevice()
    arm = ControlledArm(dev, allow_motion=True, gripper_command_mode='feedback',
                        min_io_interval_s=0)
    arm.set_gripper_value(22, 10)
    if feedback == 22:
        assert arm.get_angles() == [0.] * 6
    else:
        with pytest.raises(ValueError, match='gripper feedback'):
            arm.get_angles()
        assert dev.pending


def test_feedback_gripper_write_does_not_wait_for_an_sdk_acknowledgement():
    from mycobot_280_hardware.serial_transport import GripperDispatch

    class FeedbackDevice(Device):
        def __init__(self):
            super().__init__()
            self.frames = []

        def _mesg(self, code, *args, **kwargs):
            self.frames.append((code, args, kwargs))
            return None

        def set_gripper_value(self, *args, **kwargs):
            raise AssertionError('Synchronous acknowledgement wait must not run')

    dev = FeedbackDevice()
    arm = ControlledArm(dev, allow_motion=True, gripper_command_mode='feedback',
                        gripper_stop_strategy='hold_position')
    assert arm.set_gripper_value(22, 10) is GripperDispatch.SENT
    assert dev.frames == [(0x67, (22, 10, 1), {'_async': True})]
    dev.value = 17
    arm._session_authorizer = lambda: False
    assert arm.gripper_stop() is GripperDispatch.SENT
    assert dev.frames[-1] == (0x67, (17, 10, 1), {'_async': True})


@pytest.mark.parametrize('value,speed', [(20.5, 10), (True, 10), (20, 10.5), (20, False)])
def test_gripper_wire_commands_require_integer_fields(value, speed):
    dev = Device()
    arm = ControlledArm(dev, allow_motion=True)
    with pytest.raises(ValueError):
        arm.set_gripper_value(value, speed)
    assert not dev.commands
