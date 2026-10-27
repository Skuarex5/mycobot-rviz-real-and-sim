"""Physical sessions never inherit targets or permissions from another session."""

import math
import pytest
from mycobot_280_hardware.hardware_session import HardwareSession


def session():
    return HardwareSession([-1.0] * 6, [1.0] * 6)


def test_selection_requires_real_measurements_and_has_no_initial_command():
    s = session()
    with pytest.raises(ValueError):
        s.select([math.nan] * 6, 10.0)
    token = s.select([0.0] * 6, 10.0)
    assert token and not s.live and s.next_command([0.0] * 6, 10.0) is None


def test_command_requires_live_current_session_sequence_and_wall_age():
    s = session()
    token = s.select([0.0] * 6, 10.0)
    with pytest.raises(ValueError):
        s.command(token, 1, [0.0] * 6, 10.0, 10.0, 10.0)
    s.start_live(token, [0.0] * 6, 10.0)
    for bad_token, seq, q, stamp in [
        ("old", 1, [0.0] * 6, 10.0),
        (token, 1, [2.0] * 6, 10.0),
        (token, 1, [0.0] * 6, 9.0),
        (token, 1, [math.nan] * 6, 10.0),
    ]:
        with pytest.raises(ValueError):
            s.command(bad_token, seq, q, stamp, 10.0, 10.0)
    s.command(token, 1, [0.03] * 6, 10.0, 10.0, 10.0)
    with pytest.raises(ValueError):
        s.command(token, 1, [0.02] * 6, 10.0, 10.0, 10.0)
    assert s.next_command([0.0] * 6, 10.05) is None
    assert s.next_command([0.0] * 6, 10.1) == pytest.approx([0.01] * 6)


def test_watchdog_revokes_and_new_session_rejects_old_commands():
    s = session()
    old = s.select([0.0] * 6, 10.0)
    s.start_live(old, [0.0] * 6, 10.0)
    assert s.watchdog(10.36, 10.0) == "commands"
    s.revoke()
    assert not s.selected and not s.live
    new = s.select([0.0] * 6, 11.0)
    assert new != old
    s.start_live(new, [0.0] * 6, 11.0)
    with pytest.raises(ValueError):
        s.command(old, 99, [0.0] * 6, 11.0, 11.0, 11.0)
    s.heartbeat(new, 11.2)
    assert s.watchdog(11.8, 11.0) == "measurements"
    assert s.watchdog(12.0, 12.0) == "heartbeat"


def test_fault_cannot_be_rearmed_and_measurement_lead_is_bounded():
    s = session()
    token = s.select([0.0] * 6, 1.0)
    s.start_live(token, [0.0] * 6, 1.0)
    s.command(token, 1, [0.5] * 6, 1.0, 1.0, 1.0)
    assert max(s.next_command([0.0] * 6, 1.11)) <= 0.01100001
    assert max(s.next_command([0.0] * 6, 2.0)) <= 0.02600001
    s.revoke(fault=True)
    with pytest.raises(ValueError):
        s.select([0.0] * 6, 3.0)


def test_gripper_bounds_and_nonfinite_stamp_rejected():
    s = HardwareSession([-1.0] * 7, [1.0] * 6 + [0.15])
    token = s.select([0.0] * 7, 1.0)
    s.start_live(token, [0.0] * 7, 1.0)
    for q, stamp in [([0.0] * 6 + [0.16], 1.0), ([0.0] * 7, math.nan)]:
        with pytest.raises(ValueError):
            s.command(token, 1, q, stamp, 1.0, 1.0)


def test_gripper_intent_cannot_change_arm_target_or_retain_an_old_arm_goal():
    s = HardwareSession([-1.] * 7, [1.] * 7)
    token = s.select([0.] * 7, 1.)
    s.start_live(token, [0.] * 7, 1.)
    s.command(token, 1, [.5] * 7, 1., 1., 1.,
              arm_enabled=False, gripper_enabled=True)
    first = s.next_command([.02] * 6 + [0.], 1.1)
    assert first[:6] == pytest.approx([.02] * 6)
    assert first[6] > 0.
    s.command(token, 2, [.5] * 7, 1.2, 1.2, 1.2,
              arm_enabled=False, gripper_enabled=False)
    idle = s.next_command([.025] * 6 + [.01], 1.2)
    assert idle == pytest.approx([.025] * 6 + [.01])


def test_unchanged_settled_joint_does_not_ratchet_toward_encoder_residual():
    s = HardwareSession([-1.] * 6, [1.] * 6, command_period=.2,
                        goal_tolerance=.04, shared_step_budget=True)
    start = [.373, 0., 0., 0., .208, 0.]
    measured = [.349, 0., 0., 0., .247, 0.]
    goal = [.409, 0., 0., 0., .208, 0.]
    token = s.select(start, 1.)
    s.start_live(token, start, 1.)
    s.command(token, 1, goal, 1., 1., 1.)
    result = s.next_command(measured, 1.2)
    assert result[4] == start[4]  # unchanged command, within verified goal tolerance
    assert start[0] < result[0] <= measured[0] + .03
    assert sum(abs(a-b) for a,b in zip(result, start)) <= .012001
    with pytest.raises(ValueError):
        s.next_command([.349, 0., 0., 0., .28, 0.], 1.4)


def test_repeated_commands_cannot_hide_a_stationary_physical_robot():
    s = session()
    token = s.select([0.0] * 6, 1.0)
    s.start_live(token, [0.0] * 6, 1.0)
    for index in range(1, 27):
        now = 1.0 + 0.11 * index
        s.command(token, index, [0.5] * 6, now, now, now)
        s.next_command([0.0] * 6, now)
    with pytest.raises(ValueError, match="progress"):
        s.next_command([0.0] * 6, 4.5)


def test_physical_transport_can_limit_usb_writes_to_five_hz():
    s = HardwareSession([-1.] * 6, [1.] * 6, command_period=.2)
    token = s.select([0.] * 6, 1.)
    s.start_live(token, [0.] * 6, 1.)
    s.command(token, 1, [.5] * 6, 1., 1., 1.)
    assert s.next_command([0.] * 6, 1.1) is None
    assert s.next_command([0.] * 6, 1.2) == pytest.approx([.015] * 6)


@pytest.mark.parametrize("value", [0., -.01, .2, math.nan])
def test_invalid_live_goal_tolerance_cannot_disable_progress_checks(value):
    with pytest.raises(ValueError):
        HardwareSession([-1.] * 6, [1.] * 6, goal_tolerance=value)


def test_shared_step_budget_limits_total_arm_motion():
    s = HardwareSession([-1.] * 6, [1.] * 6, command_period=.2,
                        shared_step_budget=True)
    token = s.select([0.] * 6, 1.)
    s.start_live(token, [0.] * 6, 1.)
    s.command(token, 1, [.5] * 6, 1., 1., 1.)
    target = s.next_command([0.] * 6, 1.2)
    assert target == pytest.approx([.002] * 6)
    assert sum(abs(q) for q in target) <= .01200001


def test_measured_physical_usb_retry_window_is_bounded_without_renewing_samples():
    s = HardwareSession([-1.] * 6, [1.] * 6, measurement_timeout=.8)
    token = s.select([0.] * 6, 1.)
    s.heartbeat(token, 1.7)
    assert s.watchdog(1.7, 1.) == ''
    assert s.watchdog(1.81, 1.) == 'measurements'


def test_cartesian_ik_joint_ratios_survive_physical_rate_limiting():
    # Measured posture: q2=-.643, q3=-.647, q4=-.436. A pure +3 mm Z
    # target with unchanged tool orientation requires these three unequal
    # increments. Independent clipping makes the first TCP step go DOWN.
    s = HardwareSession([-1.] * 6, [1.] * 6, command_period=.2,
                        shared_step_budget=True)
    start = [.48153634, -.64263023, -.64716809, -.43563418, .2560398, -.1993166]
    delta = [0., -.01395520286, .05686320696, -.04290799174, 0., 0.]
    goal = [q+d for q, d in zip(start, delta)]
    token = s.select(start, 1.)
    s.start_live(token, start, 1.)
    s.command(token, 1, goal, 1., 1., 1.)
    result = s.next_command(start, 1.2)
    fractions = [(v-q)/d for v, q, d in zip(result, start, delta) if d]
    assert fractions == pytest.approx([fractions[0]] * 3)
    assert 0. < fractions[0] < 1.
    assert sum(abs(v-q) for v, q in zip(result, start)) <= .0120001


def test_measured_lead_limit_slows_whole_arm_without_reversing_joint_direction():
    s = HardwareSession([-1.] * 6, [1.] * 6, command_period=.2,
                        shared_step_budget=True)
    start = [.025, 0., 0., 0., 0., 0.]
    token = s.select(start, 1.)
    s.start_live(token, start, 1.)
    s.command(token, 1, [.125, .05, 0., 0., 0., 0.], 1., 1., 1.)
    result = s.next_command([0.] * 6, 1.2)
    assert result[0] == pytest.approx(.03)
    assert result[1] == pytest.approx(.0025)
