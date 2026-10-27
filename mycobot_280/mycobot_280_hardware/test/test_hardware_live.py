"""Fake USB checks; these do not validate physical firmware."""

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.parameter import Parameter
from mycobot_280_control_interfaces.srv import SetHardwareMode
from mycobot_280_hardware.robot_control_node import RobotController, ControlledArm
from test_robot_control_node import FakeDevice, _make_goal


class RefreshDevice(FakeDevice):
    def __init__(self):
        super().__init__()
        self.refresh = 0

    def set_fresh_mode(self, mode):
        self.refresh = mode

    def get_fresh_mode(self):
        return self.refresh


@pytest.fixture
def node():
    if not rclpy.ok():
        rclpy.init()
    dev = RefreshDevice()
    n = RobotController(
        arm_factory=lambda *args: ControlledArm(dev, allow_motion=True),
        parameter_overrides=[
            Parameter("allow_motion", value=True),
            Parameter("require_session", value=True),
        ],
    )
    n.poll_once()
    yield n, dev
    n.destroy_node()


def mode(live, token="", selected=True):
    return SetHardwareMode.Request(selected=selected, live=live, session_id=token)


def test_driver_has_no_startup_motion_and_blocks_direct_actions(node):
    n, d = node
    assert n._hardware_live is not None and not d.sent_commands
    assert not n.validate_goal_request(_make_goal([([0.1] * 6, 1.0)]))[0]
    reply = n._hardware_live.set_mode(mode(False))
    assert reply.success and reply.session_id and not d.sent_commands
    assert n.validate_goal_request(_make_goal([([0.1] * 6, 1.0)]))[0]
    reply = n._hardware_live.set_mode(mode(True, reply.session_id))
    assert reply.success and d.refresh == 1
    assert not n.validate_goal_request(_make_goal([([0.1] * 6, 1.0)]))[0]
    assert not d.sent_commands


def test_unknown_refresh_mode_prevents_live(node):
    n, d = node
    reply = n._hardware_live.set_mode(mode(False))
    d.get_fresh_mode = lambda: None
    assert not n._hardware_live.set_mode(mode(True, reply.session_id)).success
    assert not d.sent_commands


def test_verified_live_stop_publishes_fresh_state_before_restoring_trajectory(node):
    import time

    n, _ = node
    live = n._hardware_live
    token = live.set_mode(mode(False)).session_id
    assert live.set_mode(mode(True, token)).success

    def delayed_stop():
        live.measured_time -= 2.
        live.session.heartbeat_time -= 2.
        return True

    live._stop = delayed_stop
    response = live.set_mode(mode(False, token))
    assert response.success
    assert time.monotonic() - live.measured_time < .1
    assert live.authorize_trajectory()


def test_stop_requires_nonmoving_measurement_and_latches_failure(node):
    n, d = node
    reply = n._hardware_live.set_mode(mode(False))
    d.is_moving = lambda: None
    n._sleep = lambda dt: None
    stopped = n._hardware_live.set_mode(mode(False, reply.session_id, selected=False))
    assert not stopped.success and n._motion_fault
    assert not n._hardware_live.session.selected
    assert not n._hardware_live.set_mode(mode(False)).success


def test_serial_replacement_revokes_session(node):
    n, d = node
    assert n._hardware_live.set_mode(mode(False)).success
    n._close_arm_locked()
    assert not n._hardware_live.session.selected
    n._connect()
    assert not n._hardware_live.session.selected


def test_usb_motion_gate_is_checked_at_dispatch_after_session_revocation(node):
    n, d = node
    reply = n._hardware_live.set_mode(mode(False))
    assert reply.success
    n._hardware_live.session.revoke()
    with pytest.raises(PermissionError):
        n._arm.send_angles([1.0] * 6, 5)
    assert not d.sent_commands


def test_trajectory_stop_uses_the_same_verified_stop_gate(node):
    n, d = node
    assert n._hardware_live.set_mode(mode(False)).success
    d.is_moving = lambda: None
    n._sleep = lambda dt: None
    assert not n._verified_stop()
    assert n._motion_fault and not n._hardware_live.session.selected


def test_physical_gripper_session_requires_operator_confirmed_stop():
    from types import SimpleNamespace

    if not rclpy.ok():
        rclpy.init()
    d = RefreshDevice()
    n = RobotController(
        arm_factory=lambda *args: ControlledArm(d, allow_motion=True),
        parameter_overrides=[
            Parameter("allow_motion", value=True),
            Parameter("require_session", value=True),
        ],
    )
    n.poll_once()
    n._adaptive_gripper = True
    n._gripper = SimpleNamespace(stop_confirmed=False)
    try:
        reply = n._hardware_live.set_mode(mode(False))
        assert not reply.success and "gripper" in reply.message.lower()
        assert not n._hardware_live.session.selected and not d.sent_commands
    finally:
        n._adaptive_gripper = False
        n._gripper = None
        n.destroy_node()


def test_old_deselection_cannot_stop_a_new_session(node):
    n, d = node
    old = n._hardware_live.set_mode(mode(False)).session_id
    assert n._hardware_live.set_mode(mode(False, old, selected=False)).success
    new = n._hardware_live.set_mode(mode(False)).session_id
    assert new != old
    stops = d.stop_calls
    response = n._hardware_live.set_mode(mode(False, old, selected=False))
    assert not response.success
    assert n._hardware_live.session.selected and d.stop_calls == stops


def test_physical_watchdog_runs_when_ros_clock_is_paused(node):
    import time
    from rclpy.executors import SingleThreadedExecutor

    n, _ = node
    assert n._hardware_live.set_mode(mode(False)).success
    n.set_parameters([Parameter("use_sim_time", value=True)])
    executor = SingleThreadedExecutor()
    executor.add_node(n)
    try:
        deadline = time.monotonic() + 1.1
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.03)
        assert not n._hardware_live.session.selected
    finally:
        executor.remove_node(n)
        executor.shutdown()
        n.set_parameters([Parameter("use_sim_time", value=False)])


def test_connected_read_only_driver_reports_permission_instead_of_missing_usb():
    if not rclpy.ok():
        rclpy.init()
    d = RefreshDevice()
    n = RobotController(
        arm_factory=lambda *args: ControlledArm(d, allow_motion=False),
        parameter_overrides=[Parameter("require_session", value=True)],
    )
    try:
        n.poll_once()
        response = n._hardware_live.set_mode(mode(False))
        assert not response.success and "solo lectura" in response.message.lower()
        assert "USB no disponible" not in response.message
        assert not d.sent_commands
    finally:
        n.destroy_node()


def test_missing_sample_does_not_erase_or_renew_last_valid_measurement(node):
    n, _ = node
    live = n._hardware_live
    assert live.set_mode(mode(False)).success
    previous, timestamp = live.measured, live.measured_time
    live.measured_state([])
    assert live.measured == previous and live.measured_time == timestamp
    assert live.session.watchdog(timestamp + .2, live.measured_time) == ""
    assert live.session.watchdog(timestamp + .51, live.measured_time) == "measurements"


def test_complete_out_of_bounds_measurement_still_blocks_immediately(node):
    n, _ = node
    live = n._hardware_live
    assert live.set_mode(mode(False)).success
    live.measured_state([100.0] * 6)
    assert live.measured_time is None
    assert live.session.watchdog(live.session.heartbeat_time, None) == "measurements"


def test_stop_reason_remains_visible_after_verified_stop(node, monkeypatch):
    n, _ = node
    live = n._hardware_live
    assert live.set_mode(mode(False)).success
    messages = []
    monkeypatch.setattr(live.pub, "publish", messages.append)
    live._halt("Se perdió measurements")
    live.publish_status()
    assert not messages[-1].selected
    assert "Se perdió measurements" in messages[-1].message
    assert "parada verificada" in messages[-1].message


def test_live_hold_does_not_repeatedly_send_measured_angles_to_the_servos(node):
    import time

    n, dev = node
    live = n._hardware_live
    token = live.set_mode(mode(False)).session_id
    assert live.set_mode(mode(True, token)).success
    now = time.monotonic()
    live.session.command(token, 1, live.measured, time.time(), time.time(), now)
    live.session.send_time -= .11
    live.tick()
    assert not dev.sent_commands


def test_inactive_live_packet_cannot_move_arm_even_with_a_different_target(node):
    from mycobot_280_control_interfaces.msg import HardwareCommand

    n, dev = node
    live = n._hardware_live
    token = live.set_mode(mode(False)).session_id
    assert live.set_mode(mode(True, token)).success
    message = HardwareCommand(session_id=token, sequence=1, positions=[.05] * 6,
                              arm_enabled=False, gripper_enabled=False)
    message.stamp = n.get_clock().now().to_msg()
    live._command(message)
    live.session.send_time -= .11
    live.tick()
    assert not dev.sent_commands
    assert live.session.live


def test_usb_dispatch_does_not_block_command_and_measurement_callbacks(node):
    import threading
    import time

    n, dev = node
    live = n._hardware_live
    token = live.set_mode(mode(False)).session_id
    assert live.set_mode(mode(True, token)).success
    observed = []
    original = dev.send_angles

    def send(*args, **kwargs):
        def callback():
            acquired = n._state_lock.acquire(timeout=.05)
            observed.append(acquired)
            if acquired:
                n._state_lock.release()
        thread = threading.Thread(target=callback)
        thread.start()
        thread.join(timeout=.2)
        return original(*args, **kwargs)

    dev.send_angles = send
    now = time.monotonic()
    live.session.command(token, 1, [.05] * 6, time.time(), time.time(), now)
    live.session.send_time -= .11
    live.tick()
    assert observed == [True]
