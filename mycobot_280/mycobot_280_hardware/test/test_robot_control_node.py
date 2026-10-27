"""Tests for RobotController (FollowJointTrajectory + single-owner serial)."""

import math

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('control_msgs')
pytest.importorskip('trajectory_msgs')

from builtin_interfaces.msg import Duration  # noqa: E402
from control_msgs.action import FollowJointTrajectory  # noqa: E402
from rclpy.action import GoalResponse  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: E402

from mycobot_280_hardware.robot_control_node import (  # noqa: E402
    ControlledArm,
    RobotController,
)
from mycobot_280_hardware.robot_monitor_node import DEFAULT_JOINT_NAMES  # noqa: E402
from mycobot_280_hardware.trajectory_validator import (  # noqa: E402
    ERROR_GOAL_TOLERANCE_VIOLATED,
    ERROR_INVALID_GOAL,
    ERROR_NONE,
)


class FakeDevice:
    """Fake serial myCobot that tracks calls and moves toward commanded angles."""

    def __init__(self, initial_deg=None, follow_commands=True, fail_reads_after=None):
        self.angles_deg = list(initial_deg or [0.0] * 6)
        self.follow_commands = follow_commands
        self.fail_reads_after = fail_reads_after
        self.read_count = 0
        self.sent_commands = []
        self.sent_async_flags = []
        self.stop_calls = 0
        self.close_calls = 0
        self.moving = 0

    def get_angles(self):
        self.read_count += 1
        if self.fail_reads_after is not None and self.read_count > self.fail_reads_after:
            return None
        return list(self.angles_deg)

    def send_angles(self, angles, speed, _async=False):
        self.sent_commands.append((list(angles), int(speed)))
        self.sent_async_flags.append(bool(_async))
        if self.follow_commands:
            self.angles_deg = [float(a) for a in angles]
        return 1

    def stop(self):
        self.stop_calls += 1
        self.moving = 0
        return 1

    def is_moving(self):
        return self.moving

    def close(self):
        self.close_calls += 1

    def __getattr__(self, name):
        raise AssertionError(f'Forbidden device method called: {name}')


class FakeGoalHandle:
    """Minimal stand-in for ServerGoalHandle to unit-test execution deterministically."""

    def __init__(self, request: FollowJointTrajectory.Goal, cancel_at_feedback: int = 0):
        self.request = request
        self.is_cancel_requested = False
        self._cancel_at_feedback = cancel_at_feedback
        self.feedbacks = []
        self.status = None

    def publish_feedback(self, fb):
        self.feedbacks.append(fb)
        if self._cancel_at_feedback and len(self.feedbacks) >= self._cancel_at_feedback:
            self.is_cancel_requested = True

    def succeed(self):
        self.status = 'succeeded'

    def abort(self):
        self.status = 'aborted'

    def canceled(self):
        self.status = 'canceled'


@pytest.fixture(scope='module', autouse=True)
def ros():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _make_goal(points_rad_and_sec, joint_names=None):
    goal = FollowJointTrajectory.Goal()
    traj = JointTrajectory()
    traj.joint_names = list(joint_names or DEFAULT_JOINT_NAMES)
    for pos, t_sec in points_rad_and_sec:
        pt = JointTrajectoryPoint()
        pt.positions = list(pos)
        sec = int(t_sec)
        nanosec = int(round((t_sec - sec) * 1e9))
        pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
        traj.points.append(pt)
    goal.trajectory = traj
    return goal


def _make_controller(device: FakeDevice, allow_motion: bool = False):
    now = [100.0]

    def fake_sleep(dt: float):
        now[0] += max(0.01, float(dt))

    node = RobotController(
        arm_factory=lambda port, baud, allow: ControlledArm(device, allow_motion=allow),
        sleep_fn=fake_sleep,
        clock_fn=lambda: now[0],
    )
    if allow_motion:
        node._allow_motion = True
        node.set_parameters([Parameter('allow_motion', Parameter.Type.BOOL, True)])
        node._connect()
    published = []
    node._pub.publish = published.append
    return node, published


def test_startup_sends_no_motor_commands_and_defaults_to_disallow_motion():
    dev = FakeDevice()
    node, _ = _make_controller(dev, allow_motion=False)
    assert node._allow_motion is False
    assert dev.sent_commands == []
    assert dev.stop_calls == 0

    goal = _make_goal([([0.1, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0)])
    assert node._goal_callback(goal) == GoalResponse.REJECT
    with pytest.raises(PermissionError):
        node._arm.send_angles([0.0] * 6, 10)
    node.destroy_node()


def test_rejects_concurrent_and_invalid_goals():
    dev = FakeDevice()
    node, _ = _make_controller(dev, allow_motion=True)

    valid_goal = _make_goal([([0.1, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0)])
    assert node._goal_callback(valid_goal) == GoalResponse.ACCEPT
    # While _active_goal is True, any second goal is rejected.
    assert node._goal_callback(valid_goal) == GoalResponse.REJECT
    node._active_goal = False

    # Out-of-bounds goal is rejected.
    bad_goal = _make_goal([([3.1, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0)])
    assert node._goal_callback(bad_goal) == GoalResponse.REJECT
    node.destroy_node()


def test_executes_trajectory_publishes_feedback_and_verifies_final_pose():
    dev = FakeDevice(initial_deg=[0.0] * 6, follow_commands=True)
    node, published_js = _make_controller(dev, allow_motion=True)

    target_rad = [math.radians(15.0), math.radians(-10.0), 0.0, 0.0, 0.0, 0.0]
    goal = _make_goal([
        ([0.0] * 6, 0.0),
        ([math.radians(5.0), 0.0, 0.0, 0.0, 0.0, 0.0], 0.5),
        (target_rad, 1.0),
    ])
    assert node._goal_callback(goal) == GoalResponse.ACCEPT

    gh = FakeGoalHandle(goal)
    res = node._execute_callback(gh)

    assert gh.status == 'succeeded'
    assert res.error_code == ERROR_NONE
    assert len(dev.sent_commands) == 2
    assert dev.sent_async_flags == [True, True]
    assert dev.sent_commands[-1][0][0] == pytest.approx(15.0)
    assert dev.sent_commands[-1][0][1] == pytest.approx(-10.0)
    assert len(gh.feedbacks) >= 2
    assert len(published_js) >= 2
    assert node._active_goal is False
    node.destroy_node()
    assert dev.close_calls >= 1


def test_cancellation_triggers_verified_stop():
    dev = FakeDevice(initial_deg=[0.0] * 6, follow_commands=True)
    node, _ = _make_controller(dev, allow_motion=True)

    goal = _make_goal([
        ([math.radians(5.0), 0.0, 0.0, 0.0, 0.0, 0.0], 0.5),
        ([math.radians(10.0), 0.0, 0.0, 0.0, 0.0, 0.0], 1.0),
    ])
    assert node._goal_callback(goal) == GoalResponse.ACCEPT

    gh = FakeGoalHandle(goal, cancel_at_feedback=1)
    res = node._execute_callback(gh)

    assert gh.status == 'canceled'
    assert res.error_code == ERROR_NONE
    assert dev.stop_calls >= 1
    assert node._active_goal is False
    node.destroy_node()


def test_aborts_and_stops_when_arm_does_not_reach_goal_tolerance():
    # Arm ignores commands and stays at 0 deg, so final error > goal_tolerance_rad.
    dev = FakeDevice(initial_deg=[0.0] * 6, follow_commands=False)
    node, _ = _make_controller(dev, allow_motion=True)
    node._goal_time_tol = 0.2

    goal = _make_goal([([math.radians(20.0), 0.0, 0.0, 0.0, 0.0, 0.0], 1.0)])
    assert node._goal_callback(goal) == GoalResponse.ACCEPT

    gh = FakeGoalHandle(goal)
    res = node._execute_callback(gh)

    assert gh.status == 'aborted'
    assert res.error_code == ERROR_GOAL_TOLERANCE_VIOLATED
    assert dev.stop_calls >= 1
    node.destroy_node()


def test_aborts_on_serial_communication_loss_and_closes_old_handle():
    # First read succeeds (for goal validation), subsequent reads fail to trigger disconnect.
    dev = FakeDevice(initial_deg=[0.0] * 6, follow_commands=True, fail_reads_after=1)
    node, _ = _make_controller(dev, allow_motion=True)
    node._max_failures = 2
    initial_closes = dev.close_calls

    goal = _make_goal([([math.radians(5.0), 0.0, 0.0, 0.0, 0.0, 0.0], 0.2)])
    assert node._goal_callback(goal) == GoalResponse.ACCEPT

    gh = FakeGoalHandle(goal)
    res = node._execute_callback(gh)

    assert gh.status == 'aborted'
    assert res.error_code == ERROR_INVALID_GOAL
    assert dev.close_calls > initial_closes
    node.destroy_node()


class SimulatedServoDevice:
    """Mock hardware that moves joints at finite angular speed over simulated time."""

    def __init__(self, clock_fn, initial_deg=None, deg_per_sec_per_speed=1.8):
        self._clock = clock_fn
        self.angles_deg = list(initial_deg or [0.0] * 6)
        self._start_deg = list(self.angles_deg)
        self._target_deg = list(self.angles_deg)
        self._cmd_time = self._clock()
        self._duration = 0.0
        self._deg_per_sec_per_speed = float(deg_per_sec_per_speed)
        self.sent_commands = []
        self.command_timestamps = []
        self.stop_calls = 0
        self.close_calls = 0

    def _advance(self):
        if self._duration <= 0.0:
            return
        elapsed = max(0.0, self._clock() - self._cmd_time)
        alpha = min(1.0, elapsed / self._duration)
        self.angles_deg = [
            s + alpha * (t - s) for s, t in zip(self._start_deg, self._target_deg)
        ]

    def get_angles(self):
        self._advance()
        return [round(a, 2) for a in self.angles_deg]

    def send_angles(self, angles, speed, _async=False):
        self._advance()
        now = self._clock()
        self._start_deg = list(self.angles_deg)
        self._target_deg = [float(a) for a in angles]
        self._cmd_time = now
        max_delta = max(abs(t - s) for s, t in zip(self._start_deg, self._target_deg))
        deg_s = max(1.0, float(speed) * self._deg_per_sec_per_speed)
        self._duration = max_delta / deg_s
        self.sent_commands.append((list(self._target_deg), int(speed), bool(_async)))
        self.command_timestamps.append(now)
        return 1

    def stop(self):
        self._advance()
        self._target_deg = list(self.angles_deg)
        self._duration = 0.0
        self.stop_calls += 1
        return 1

    def is_moving(self):
        self._advance()
        return 0 if self._clock() >= self._cmd_time + self._duration else 1

    def close(self):
        self.close_calls += 1


def test_small_goal_waits_for_actual_motion_to_finish_before_success():
    now = [200.0]
    dev = SimulatedServoDevice(clock_fn=lambda: now[0])
    node = RobotController(
        arm_factory=lambda *args: ControlledArm(dev, allow_motion=True, min_io_interval_s=0),
        parameter_overrides=[Parameter('allow_motion', value=True)],
        sleep_fn=lambda dt: now.__setitem__(0, now[0] + max(.01, dt)),
        clock_fn=lambda: now[0],
    )
    try:
        goal = _make_goal([([.035, 0., 0., 0., 0., 0.], 3.)])
        handle = FakeGoalHandle(goal)
        result = node._execute_callback(handle)
        assert result.error_code == ERROR_NONE and handle.status == 'succeeded'
        assert dev.is_moving() == 0
        assert math.radians(dev.get_angles()[0]) == pytest.approx(.035, abs=.001)
    finally:
        node.destroy_node()


def test_final_position_alone_cannot_verify_a_robot_still_reporting_motion():
    dev = FakeDevice()
    node, _ = _make_controller(dev, allow_motion=True)
    dev.moving = 1
    try:
        goal = _make_goal([([.1, 0., 0., 0., 0., 0.], 1.)])
        handle = FakeGoalHandle(goal)
        result = node._execute_callback(handle)
        assert handle.status == 'aborted'
        assert result.error_code == ERROR_GOAL_TOLERANCE_VIOLATED
        assert dev.stop_calls > 0
    finally:
        node.destroy_node()


def test_dense_moveit_trajectory_executes_smoothly_with_single_command_and_instrumentation():
    now = [200.0]

    def fake_sleep(dt: float):
        now[0] += max(0.01, float(dt))

    sim_dev = SimulatedServoDevice(clock_fn=lambda: now[0], initial_deg=[0.0] * 6)
    node = RobotController(
        arm_factory=lambda port, baud, allow: ControlledArm(
            sim_dev, allow_motion=allow, min_io_interval_s=0.0
        ),
        sleep_fn=fake_sleep,
        clock_fn=lambda: now[0],
    )
    node._allow_motion = True
    node.set_parameters([Parameter('allow_motion', Parameter.Type.BOOL, True)])
    node._connect()

    # Build a 25-point MoveIt 2 TOTG trajectory along a 6D joint-space line.
    start_rad = [0.0] * 6
    goal_rad = [
        math.radians(25.0),
        math.radians(-18.0),
        math.radians(12.0),
        math.radians(-10.0),
        math.radians(15.0),
        math.radians(-20.0),
    ]
    total_time = 2.4
    points = []
    for i in range(25):
        u = i / 24.0
        s = 3 * (u ** 2) - 2 * (u ** 3)
        pos = [a + s * (b - a) for a, b in zip(start_rad, goal_rad)]
        points.append((pos, round(u * total_time, 4)))

    goal = _make_goal(points)
    assert node._goal_callback(goal) == GoalResponse.ACCEPT

    gh = FakeGoalHandle(goal)
    res = node._execute_callback(gh)

    assert gh.status == 'succeeded'
    assert res.error_code == ERROR_NONE
    # 24 post-t0 MoveIt points coalesced into 1 continuous point-to-point command!
    assert len(sim_dev.sent_commands) == 1
    assert sim_dev.sent_commands[0][2] is True  # _async=True
    assert len(gh.feedbacks) >= 5

    trace = node.last_execution_trace
    assert len(trace) == 1
    rec = trace[0]
    assert rec.cmd_index == 0
    assert rec.total_commands == 1
    assert rec.planned_segment_duration_s == pytest.approx(total_time)
    assert rec.actual_segment_duration_s > 0.3
    assert rec.poll_count >= 5
    assert rec.max_error_rad <= node._goal_tol
    assert rec.measured_start_rad == pytest.approx([0.0] * 6)
    for m, g in zip(rec.measured_end_rad, goal_rad):
        assert abs(m - g) <= node._goal_tol

    node.destroy_node()
