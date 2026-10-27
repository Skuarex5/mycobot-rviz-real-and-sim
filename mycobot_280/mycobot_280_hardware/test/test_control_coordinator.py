"""Coordinator permissions and authoritative selection with ROS message types."""

import time
import pytest

rclpy = pytest.importorskip("rclpy")
from std_srvs.srv import SetBool
from rclpy.action import GoalResponse
from moveit_msgs.action import ExecuteTrajectory
from mycobot_280_hardware.control_coordinator import ControlCoordinator


@pytest.fixture
def coordinator():
    if not rclpy.ok():
        rclpy.init()
    n = ControlCoordinator()
    yield n
    n.destroy_node()


def test_disconnected_physical_selection_is_rejected_without_changing_target(
    coordinator,
):
    n = coordinator
    response = n.select_physical(SetBool.Request(data=True), SetBool.Response())
    assert not response.success and not n.policy.physical


def test_execute_trajectory_rejects_unregistered_cached_plan(coordinator):
    n = coordinator
    source = n.sources["simulation"]
    source.update(n.names, [0.0] * len(n.names), 1.0, time.monotonic())
    assert n.accept_execute(ExecuteTrajectory.Goal()) == GoalResponse.REJECT
    assert n.policy.jobs == 0


def test_reselecting_current_destination_clears_previous_error(coordinator):
    n = coordinator
    n.status = "Detené seguimiento y ejecución antes de cambiar destino"
    response = n.select_physical(SetBool.Request(data=False), SetBool.Response())
    assert response.success
    assert response.message == "Destino: Gazebo"


def test_verified_stop_transition_has_bounded_grace_without_active_command_gate(coordinator):
    from mycobot_280_control_interfaces.msg import HardwareStatus

    n = coordinator
    n.policy.physical = True
    n.policy.live = False
    n.hardware = HardwareStatus(connected=True, selected=True, live=True)
    n.heartbeat = time.monotonic()
    n.physical_stopping_until = time.monotonic() + 1.
    n.tick()
    assert n.policy.available
    assert not n.policy.live
    n.physical_stopping_until = time.monotonic() - .1
    n.tick()
    assert not n.policy.available


def test_trajectory_provenance_is_semantic_across_ros_serialization():
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from rclpy.serialization import serialize_message, deserialize_message
    from mycobot_280_hardware.control_coordinator import trajectory_signature

    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = list("abcdef")
    for index in range(8):
        point = JointTrajectoryPoint(
            positions=[0.1 * index] * 6, velocities=[0.02] * 6, accelerations=[0.03] * 6
        )
        point.time_from_start.nanosec = 100000000 * index
        trajectory.joint_trajectory.points.append(point)
    expected = trajectory_signature(trajectory)
    roundtrip = deserialize_message(serialize_message(trajectory), RobotTrajectory)
    assert trajectory_signature(roundtrip) == expected
    assert {trajectory_signature(trajectory) for _ in range(100)} == {expected}


def test_nonfinite_execute_request_is_rejected_without_callback_exception(coordinator):
    from trajectory_msgs.msg import JointTrajectoryPoint

    request = ExecuteTrajectory.Goal()
    request.trajectory.joint_trajectory.points = [
        JointTrajectoryPoint(positions=[float("nan")])
    ]
    assert coordinator.accept_execute(request) == GoalResponse.REJECT


def test_delayed_status_cannot_overwrite_confirmed_hardware_transition(coordinator):
    from types import SimpleNamespace

    n = coordinator
    confirmed = SimpleNamespace(driver_id="boot", status_sequence=10, selected=True)
    delayed = SimpleNamespace(driver_id="boot", status_sequence=9, selected=False)
    n.hardware = confirmed
    n.hardware_status(delayed)
    assert n.hardware is confirmed


def test_missing_physical_action_revokes_permission_without_fallback(coordinator):
    from types import SimpleNamespace
    from control_msgs.action import FollowJointTrajectory

    n = coordinator
    n.policy.physical = True
    n.token = "session"
    n.policy.begin_job()
    action = "mycobot280m5_group_controller/follow_joint_trajectory"
    n.action_clients[(action, "physical")] = SimpleNamespace(
        wait_for_server=lambda **kw: False
    )
    aborted = []
    handle = SimpleNamespace(
        request=FollowJointTrajectory.Goal(), abort=lambda: aborted.append(True)
    )
    result = n.forward(handle, FollowJointTrajectory, action)
    assert result.error_code < 0 and aborted
    assert n.policy.physical and not n.policy.available and not n.token


def test_busy_moveit_request_returns_nonnull_aborted_result_without_forwarding(coordinator):
    from types import SimpleNamespace
    from moveit_msgs.action import MoveGroup

    n = coordinator
    n.policy.live = True
    n.sources['simulation'].update(n.names, [0.] * len(n.names), 1., time.monotonic())
    aborted = []
    h = SimpleNamespace(request=MoveGroup.Goal(), abort=lambda: aborted.append(True))
    # Humble MoveGroupInterface dereferences res->error_code on transport
    # rejection. Accept transport, then return a populated failure result.
    assert n.accept_moveit(h.request) == GoalResponse.ACCEPT
    result = n.execute_moveit(h, MoveGroup, 'move_action')
    assert aborted and result.error_code.val != 1
    assert n.policy.jobs == 0 and n.policy.live
    assert 'Detené' in n.status


def test_invalid_execute_plan_returns_result_without_acquiring_job(coordinator):
    from types import SimpleNamespace

    n = coordinator
    aborted = []
    h = SimpleNamespace(request=ExecuteTrajectory.Goal(), abort=lambda: aborted.append(True))
    assert n.accept_moveit(h.request) == GoalResponse.ACCEPT
    result = n.execute_moveit(h, ExecuteTrajectory, 'execute_trajectory')
    assert aborted and result.error_code.val != 1
    assert n.policy.jobs == 0


def test_available_moveit_request_acquires_destination_before_forwarding(coordinator):
    from types import SimpleNamespace
    from moveit_msgs.action import MoveGroup

    n = coordinator
    n.sources['simulation'].update(n.names, [0.] * len(n.names), 1., time.monotonic())
    h = SimpleNamespace(request=MoveGroup.Goal())
    expected = MoveGroup.Result()
    forwarded = []

    def forward(handle, kind, action):
        assert n.policy.jobs == 1
        with pytest.raises(ValueError):
            n.policy.select(True)
        forwarded.append((handle, kind, action))
        n.policy.end_job()
        return expected

    n.forward = forward
    assert n.execute_moveit(h, MoveGroup, 'move_action') is expected
    assert forwarded == [(h, MoveGroup, 'move_action')]
    assert n.policy.jobs == 0


@pytest.mark.parametrize("connected", [False, True])
def test_explicit_gazebo_selection_survives_disconnected_physical_driver(coordinator, connected):
    from mycobot_280_control_interfaces.msg import HardwareStatus

    n = coordinator
    n.policy.select(True)
    n.token = 'expired-physical-token'
    n.hardware = HardwareStatus(connected=connected, selected=False, live=False, fault=True)
    n.policy.lost()
    n.sources['simulation'].update(n.names, [0.] * len(n.names), 1., time.monotonic())
    called = []

    def unavailable(*args):
        called.append(args)
        raise ValueError('No se pudo verificar parada: USB desconectado')

    n.physical_mode = unavailable
    response = n.select_physical(SetBool.Request(data=False), SetBool.Response())
    assert response.success and not n.policy.physical
    assert n.policy.available and not n.token
    assert not called  # No USB acknowledgement required to choose simulation.
    assert n.hardware.fault  # The physical fault remains latched.
    assert n.accept_job(None) == GoalResponse.ACCEPT
    n.policy.end_job()


def test_healthy_physical_deselection_still_requires_verified_stop(coordinator):
    from mycobot_280_control_interfaces.msg import HardwareStatus

    n = coordinator
    n.policy.select(True)
    n.token = 'active'
    n.hardware = HardwareStatus(connected=True, selected=True)
    called = []

    def fail_stop(*args):
        called.append(args)
        raise ValueError('Stop unverified')

    n.physical_mode = fail_stop
    response = n.select_physical(SetBool.Request(data=False), SetBool.Response())
    assert not response.success and n.policy.physical
    assert called


@pytest.mark.parametrize('group,velocity_factor', [('arm_group', .1), ('gripper_group', .2)])
def test_physical_planning_keeps_previous_absolute_limits_without_slowing_gazebo(group, velocity_factor):
    from moveit_msgs.action import MoveGroup
    from mycobot_280_hardware.control_coordinator import planning_goal_for_destination

    goal = MoveGroup.Goal()
    goal.request.group_name = group
    goal.request.max_velocity_scaling_factor = .5
    goal.request.max_acceleration_scaling_factor = .25
    physical = planning_goal_for_destination(goal, True)
    simulation = planning_goal_for_destination(goal, False)
    assert physical.request.max_velocity_scaling_factor == pytest.approx(.5 * velocity_factor)
    assert physical.request.max_acceleration_scaling_factor == pytest.approx(.25 * .2)
    assert simulation.request.max_velocity_scaling_factor == .5
    assert simulation.request.max_acceleration_scaling_factor == .25
    assert goal.request.max_velocity_scaling_factor == .5


def test_manual_moveit_request_is_blocked_during_automatic_following(coordinator):
    from types import SimpleNamespace
    from moveit_msgs.action import MoveGroup

    n = coordinator
    n.enabled = True
    n.sources['simulation'].update(n.names, [0.] * len(n.names), 1., time.monotonic())
    forwarded, aborted = [], []
    n.forward = lambda *args: forwarded.append(True) or MoveGroup.Result()
    h = SimpleNamespace(request=MoveGroup.Goal(), abort=lambda: aborted.append(True))
    result = n.execute_moveit(h, MoveGroup, 'move_action')
    assert not forwarded and aborted and result.error_code.val != 1
    assert n.policy.jobs == 0


def test_automatic_moveit_request_uses_selected_destination_while_following(coordinator):
    from types import SimpleNamespace
    from moveit_msgs.action import MoveGroup

    n = coordinator
    n.enabled = True
    n.sources['simulation'].update(n.names, [0.] * len(n.names), 1., time.monotonic())
    forwarded = []
    def forward(handle, kind, action):
        forwarded.append(action)
        assert n.policy.jobs == 1
        n.policy.end_job()
        return MoveGroup.Result()
    n.forward = forward
    h = SimpleNamespace(request=MoveGroup.Goal())
    n.execute_moveit(h, MoveGroup, 'move_action', automatic=True)
    assert forwarded == ['move_action'] and n.policy.jobs == 0


def test_delayed_automatic_request_after_stop_is_rejected_before_destination_job(coordinator):
    from types import SimpleNamespace
    from moveit_msgs.action import MoveGroup

    n = coordinator
    n.enabled = False
    n.sources['simulation'].update(n.names, [0.] * len(n.names), 1., time.monotonic())
    aborted, forwarded = [], []
    n.forward = lambda *args: forwarded.append(True) or MoveGroup.Result()
    h = SimpleNamespace(request=MoveGroup.Goal(), abort=lambda: aborted.append(True))
    result = n.execute_moveit(h, MoveGroup, 'move_action', automatic=True)
    assert aborted and not forwarded and result.error_code.val != 1
    assert n.policy.jobs == 0
