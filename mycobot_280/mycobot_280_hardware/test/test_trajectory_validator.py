"""Unit tests for pure trajectory validation and speed scaling."""

import pytest


def test_dense_straight_path_is_checked_without_quadratic_prefix_rescans(monkeypatch):
    import mycobot_280_hardware.trajectory_validator as validator

    points = [validator.WaypointSpec([i / 1400] * 6, [], i * .1, .1, 5)
              for i in range(1, 1401)]
    inspected = 0
    original = validator._can_coalesce_subpath

    def check(anchor, subpath, tolerance):
        nonlocal inspected
        inspected += len(subpath)
        return original(anchor, subpath, tolerance)

    monkeypatch.setattr(validator, '_can_coalesce_subpath', check)
    result = validator.coalesce_waypoints([0.] * 6, points)
    assert len(result) == 1 and result[0].positions_rad == [1.] * 6
    assert inspected <= 2800

from mycobot_280_hardware.joint_conversion import (
    DEFAULT_JOINT_NAMES,
    DEFAULT_LOWER,
    DEFAULT_UPPER,
)
from mycobot_280_hardware.trajectory_validator import (
    coalesce_waypoints,
    compute_pymycobot_speed,
    ERROR_INVALID_GOAL,
    ERROR_INVALID_JOINTS,
    ERROR_NONE,
    interpolate_desired_waypoint,
    validate_trajectory,
)


def _validate(
    points,
    joint_names=None,
    current=None,
    max_vel=1.0,
    start_tol=0.15,
):
    return validate_trajectory(
        traj_joint_names=joint_names if joint_names is not None else DEFAULT_JOINT_NAMES,
        traj_points=points,
        canonical_joint_names=DEFAULT_JOINT_NAMES,
        current_positions_rad=[0.0] * 6 if current is None else current,
        lower_limits_rad=DEFAULT_LOWER,
        upper_limits_rad=DEFAULT_UPPER,
        max_joint_velocity_rad_s=max_vel,
        start_state_tolerance_rad=start_tol,
        default_pymycobot_speed=15,
        min_pymycobot_speed=5,
        max_pymycobot_speed=30,
    )


def test_valid_multi_point_trajectory_drops_t0_and_reorders_joints():
    reversed_names = list(reversed(DEFAULT_JOINT_NAMES))
    # In reversed_names, index 5 corresponds to joint2_to_joint1 (first canonical joint).
    p0 = [0.0] * 6
    p1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.2]
    p2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.4]
    res = _validate(
        points=[
            (p0, [0.0] * 6, 0.0),
            (p1, [0.0, 0.0, 0.0, 0.0, 0.0, 0.2], 1.0),
            (p2, [0.0] * 6, 2.0),
        ],
        joint_names=reversed_names,
    )
    assert res.ok
    assert res.error_code == ERROR_NONE
    assert len(res.waypoints) == 2
    assert res.waypoints[0].positions_rad[0] == pytest.approx(0.2)
    assert res.waypoints[1].positions_rad[0] == pytest.approx(0.4)
    assert 5 <= res.waypoints[0].pymycobot_speed <= 30


def test_rejects_mismatched_joint_names():
    res = _validate(
        points=[([0.0] * 6, [], 1.0)],
        joint_names=['bad_joint'] + DEFAULT_JOINT_NAMES[1:],
    )
    assert not res.ok
    assert res.error_code == ERROR_INVALID_JOINTS


def test_rejects_empty_points():
    res = _validate(points=[])
    assert not res.ok
    assert res.error_code == ERROR_INVALID_GOAL


def test_rejects_missing_current_state():
    res = validate_trajectory(
        traj_joint_names=DEFAULT_JOINT_NAMES,
        traj_points=[([0.0] * 6, [], 1.0)],
        canonical_joint_names=DEFAULT_JOINT_NAMES,
        current_positions_rad=None,
        lower_limits_rad=DEFAULT_LOWER,
        upper_limits_rad=DEFAULT_UPPER,
        max_joint_velocity_rad_s=1.0,
        start_state_tolerance_rad=0.15,
    )
    assert not res.ok
    assert res.error_code == ERROR_INVALID_GOAL


def test_rejects_position_outside_urdf_limits():
    over_limit = [3.1, 0.0, 0.0, 0.0, 0.0, 0.0]  # joint1 limit is 2.9321
    res = _validate(points=[(over_limit, [], 2.0)])
    assert not res.ok
    assert 'exceeds URDF joint limits' in res.reason


def test_rejects_velocity_above_limit():
    res = _validate(
        points=[([0.2, 0.0, 0.0, 0.0, 0.0, 0.0], [1.5, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0)],
        max_vel=1.0,
    )
    assert not res.ok
    assert 'velocity' in res.reason


def test_rejects_non_increasing_timestamps():
    res = _validate(
        points=[
            ([0.1, 0.0, 0.0, 0.0, 0.0, 0.0], [], 1.0),
            ([0.2, 0.0, 0.0, 0.0, 0.0, 0.0], [], 1.0),
        ]
    )
    assert not res.ok
    assert 'strictly increasing' in res.reason


def test_rejects_start_state_deviation_at_t0():
    res = _validate(
        points=[
            ([0.5, 0.0, 0.0, 0.0, 0.0, 0.0], [], 0.0),
            ([0.6, 0.0, 0.0, 0.0, 0.0, 0.0], [], 1.0),
        ],
        current=[0.0] * 6,
        start_tol=0.15,
    )
    assert not res.ok
    assert 'Start state deviates' in res.reason


def test_compute_pymycobot_speed_clamps_to_bounds():
    assert compute_pymycobot_speed(
        [0.0] * 6, [0.5] + [0.0] * 5, 0.5, 1.0, 15, 5, 30
    ) == 30
    assert compute_pymycobot_speed(
        [0.0] * 6, [0.0] * 6, 1.0, 1.0, 15, 5, 30
    ) == 5
    assert compute_pymycobot_speed(
        [0.0] * 6, [0.1] + [0.0] * 5, 0.0, 1.0, 15, 5, 30
    ) == 15


def test_coalesce_waypoints_reduces_dense_collinear_moveit_trajectory_to_single_command():
    start = [0.0, -0.1, 0.05, 0.0, -0.08, -1.2]
    goal = [0.4, -0.35, 0.25, -0.15, 0.12, -0.8]
    total_time = 2.4
    num_samples = 25
    points = []
    for i in range(num_samples):
        u = i / (num_samples - 1)
        # Smooth cubic s(u) = 3u^2 - 2u^3 mimicking TOTG acceleration/deceleration
        s = 3 * (u ** 2) - 2 * (u ** 3)
        pos = [a + s * (b - a) for a, b in zip(start, goal)]
        points.append((pos, [], round(u * total_time, 4)))

    res = _validate(points=points, current=start)
    assert res.ok
    assert len(res.waypoints) == num_samples - 1

    coalesced = coalesce_waypoints(start, res.waypoints)
    assert len(coalesced) == 1
    assert coalesced[0].positions_rad == pytest.approx(goal)
    assert coalesced[0].time_from_start_s == pytest.approx(total_time)
    assert coalesced[0].segment_duration_s == pytest.approx(total_time)


def test_coalesce_waypoints_preserves_corner_and_reversal_waypoints():
    start = [0.0] * 6
    # Corner trajectory: move joint 1 first, then move joint 1 + joint 2.
    res_corner = _validate(
        points=[
            (start, [], 0.0),
            ([0.15, 0.0, 0.0, 0.0, 0.0, 0.0], [], 0.6),
            ([0.30, -0.25, 0.0, 0.0, 0.0, 0.0], [], 1.2),
        ],
        current=start,
    )
    assert res_corner.ok
    coalesced_corner = coalesce_waypoints(start, res_corner.waypoints)
    assert len(coalesced_corner) == 2

    # Reversal trajectory on joint 1: 0.0 -> +0.3 -> 0.0
    res_rev = _validate(
        points=[
            (start, [], 0.0),
            ([0.3, 0.0, 0.0, 0.0, 0.0, 0.0], [], 1.0),
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], 2.0),
        ],
        current=start,
    )
    assert res_rev.ok
    coalesced_rev = coalesce_waypoints(start, res_rev.waypoints)
    assert len(coalesced_rev) == 2
    assert coalesced_rev[0].positions_rad[0] == pytest.approx(0.3)
    assert coalesced_rev[1].positions_rad[0] == pytest.approx(0.0)


def test_interpolate_desired_waypoint_interpolates_positions_at_elapsed_time():
    start = [0.0] * 6
    res = _validate(
        points=[
            (start, [], 0.0),
            ([0.4, -0.2, 0.0, 0.0, 0.0, 0.0], [], 2.0),
        ],
        current=start,
    )
    assert res.ok
    mid = interpolate_desired_waypoint(res.waypoints, start, elapsed_s=1.0)
    assert mid.positions_rad[0] == pytest.approx(0.2)
    assert mid.positions_rad[1] == pytest.approx(-0.1)
    assert mid.time_from_start_s == pytest.approx(1.0)
