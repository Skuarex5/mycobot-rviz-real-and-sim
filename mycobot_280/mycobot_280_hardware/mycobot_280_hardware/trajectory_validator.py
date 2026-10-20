"""
Pure validation and waypoint planning helpers for FollowJointTrajectory goals.

No ROS runtime or serial I/O is used here so every rule is unit-testable.
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

from mycobot_280_hardware.joint_conversion import find_limit_violations, NUM_JOINTS


@dataclass(frozen=True)
class WaypointSpec:
    """Single validated waypoint in canonical URDF joint order."""

    positions_rad: List[float]
    velocities_rad_s: List[float]
    time_from_start_s: float
    segment_duration_s: float
    pymycobot_speed: int


@dataclass(frozen=True)
class ValidationResult:
    """Outcome of trajectory validation."""

    ok: bool
    reason: str
    error_code: int
    waypoints: List[WaypointSpec]


# Mirror control_msgs/action/FollowJointTrajectory result error codes
ERROR_NONE = 0
ERROR_INVALID_GOAL = -1
ERROR_INVALID_JOINTS = -2
ERROR_OLD_HEADER_TIMESTAMP = -3
ERROR_PATH_TOLERANCE_VIOLATED = -4
ERROR_GOAL_TOLERANCE_VIOLATED = -5


def duration_to_seconds(sec: int, nanosec: int) -> float:
    """Convert a ROS builtin_interfaces/Duration (sec, nanosec) to float seconds."""
    return float(sec) + float(nanosec) * 1e-9


def compute_pymycobot_speed(
    prev_positions_rad: Sequence[float],
    next_positions_rad: Sequence[float],
    segment_duration_s: float,
    max_joint_velocity_rad_s: float,
    default_speed: int,
    min_speed: int,
    max_speed: int,
) -> int:
    """
    Map the segment's peak joint speed to a conservative pymycobot speed (1..100).

    If ``segment_duration_s`` is zero or negligible, returns ``default_speed``
    clamped to ``[min_speed, max_speed]``.
    """
    lo = max(1, int(min_speed))
    hi = min(100, max(lo, int(max_speed)))
    fallback = max(lo, min(hi, int(default_speed)))
    if segment_duration_s <= 1e-3 or max_joint_velocity_rad_s <= 1e-6:
        return fallback

    peak_vel = max(
        abs(b - a) / segment_duration_s
        for a, b in zip(prev_positions_rad, next_positions_rad)
    )
    if peak_vel <= 1e-6:
        return lo
    ratio = min(1.0, peak_vel / max_joint_velocity_rad_s)
    scaled = int(round(lo + ratio * (hi - lo)))
    return max(lo, min(hi, scaled))


def validate_trajectory(
    traj_joint_names: Sequence[str],
    traj_points: Sequence[Tuple[Sequence[float], Sequence[float], float]],
    canonical_joint_names: Sequence[str],
    current_positions_rad: Optional[Sequence[float]],
    lower_limits_rad: Sequence[float],
    upper_limits_rad: Sequence[float],
    max_joint_velocity_rad_s: float,
    start_state_tolerance_rad: float,
    default_pymycobot_speed: int = 15,
    min_pymycobot_speed: int = 5,
    max_pymycobot_speed: int = 30,
) -> ValidationResult:
    """
    Validate a FollowJointTrajectory request and build canonical waypoints.

    ``traj_points`` is a sequence of ``(positions, velocities, time_from_start_s)``
    tuples extracted from ``trajectory_msgs/JointTrajectoryPoint``.
    """
    if len(canonical_joint_names) != NUM_JOINTS:
        return ValidationResult(
            False, 'Canonical joint list must have 6 joints', ERROR_INVALID_JOINTS, []
        )

    if (
        len(traj_joint_names) != NUM_JOINTS
        or set(traj_joint_names) != set(canonical_joint_names)
    ):
        return ValidationResult(
            False,
            f'Trajectory joints {list(traj_joint_names)} do not match '
            f'expected {list(canonical_joint_names)}',
            ERROR_INVALID_JOINTS,
            [],
        )

    if not traj_points:
        return ValidationResult(
            False, 'Trajectory contains no waypoints', ERROR_INVALID_GOAL, []
        )

    if current_positions_rad is None or len(current_positions_rad) != NUM_JOINTS:
        return ValidationResult(
            False,
            'No valid measured robot joint state available before execution',
            ERROR_INVALID_GOAL,
            [],
        )

    index_map = [traj_joint_names.index(name) for name in canonical_joint_names]

    waypoints: List[WaypointSpec] = []
    prev_time = -1.0
    prev_positions = list(current_positions_rad)

    for idx, (raw_pos, raw_vel, t_sec) in enumerate(traj_points):
        if len(raw_pos) != NUM_JOINTS:
            return ValidationResult(
                False,
                f'Point {idx} has {len(raw_pos)} positions; expected {NUM_JOINTS}',
                ERROR_INVALID_GOAL,
                [],
            )
        if not math.isfinite(t_sec) or t_sec < 0.0:
            return ValidationResult(
                False,
                f'Point {idx} has invalid time_from_start={t_sec}',
                ERROR_INVALID_GOAL,
                [],
            )
        if idx == 0 and t_sec < 0.0:
            return ValidationResult(
                False, 'First point has negative time_from_start', ERROR_INVALID_GOAL, []
            )
        if idx > 0 and t_sec <= prev_time:
            return ValidationResult(
                False,
                f'Point {idx} time_from_start ({t_sec:.4f}s) is not strictly '
                f'increasing after {prev_time:.4f}s',
                ERROR_INVALID_GOAL,
                [],
            )

        ordered_pos: List[float] = []
        for j in index_map:
            val = raw_pos[j]
            if isinstance(val, bool) or not isinstance(val, (int, float)):
                return ValidationResult(
                    False, f'Point {idx} has non-numeric position', ERROR_INVALID_GOAL, []
                )
            if not math.isfinite(val):
                return ValidationResult(
                    False, f'Point {idx} has non-finite position', ERROR_INVALID_GOAL, []
                )
            ordered_pos.append(float(val))

        violations = find_limit_violations(
            ordered_pos, lower_limits_rad, upper_limits_rad, tolerance=1e-3
        )
        if violations:
            detail = ', '.join(
                f'{canonical_joint_names[i]}={p:.3f}' for i, p in violations
            )
            return ValidationResult(
                False,
                f'Point {idx} exceeds URDF joint limits: {detail}',
                ERROR_INVALID_GOAL,
                [],
            )

        ordered_vel: List[float] = [0.0] * NUM_JOINTS
        if raw_vel:
            if len(raw_vel) != NUM_JOINTS:
                return ValidationResult(
                    False,
                    f'Point {idx} has {len(raw_vel)} velocities; expected {NUM_JOINTS}',
                    ERROR_INVALID_GOAL,
                    [],
                )
            for canonical_idx, j in enumerate(index_map):
                v = raw_vel[j]
                if (
                    isinstance(v, bool)
                    or not isinstance(v, (int, float))
                    or not math.isfinite(v)
                ):
                    return ValidationResult(
                        False,
                        f'Point {idx} has invalid velocity',
                        ERROR_INVALID_GOAL,
                        [],
                    )
                if abs(float(v)) > max_joint_velocity_rad_s + 1e-3:
                    return ValidationResult(
                        False,
                        f'Point {idx} joint {canonical_joint_names[canonical_idx]} '
                        f'velocity {float(v):.3f} rad/s exceeds limit '
                        f'{max_joint_velocity_rad_s:.3f} rad/s',
                        ERROR_INVALID_GOAL,
                        [],
                    )
                ordered_vel[canonical_idx] = float(v)

        if idx == 0 and t_sec <= 1e-3:
            max_start_err = max(
                abs(a - b) for a, b in zip(ordered_pos, current_positions_rad)
            )
            if max_start_err > start_state_tolerance_rad:
                return ValidationResult(
                    False,
                    f'Start state deviates from measured pose by '
                    f'{max_start_err:.3f} rad (limit {start_state_tolerance_rad:.3f} rad)',
                    ERROR_INVALID_GOAL,
                    [],
                )

        seg_dt = t_sec if idx == 0 else (t_sec - prev_time)
        if seg_dt > 1e-3:
            implied_vel = max(
                abs(b - a) / seg_dt for a, b in zip(prev_positions, ordered_pos)
            )
            if implied_vel > max_joint_velocity_rad_s * 1.5 + 1e-3:
                return ValidationResult(
                    False,
                    f'Segment {idx} implies joint velocity {implied_vel:.3f} rad/s '
                    f'above allowed maximum {max_joint_velocity_rad_s:.3f} rad/s',
                    ERROR_INVALID_GOAL,
                    [],
                )

        speed = compute_pymycobot_speed(
            prev_positions,
            ordered_pos,
            seg_dt,
            max_joint_velocity_rad_s,
            default_pymycobot_speed,
            min_pymycobot_speed,
            max_pymycobot_speed,
        )

        waypoints.append(
            WaypointSpec(
                positions_rad=ordered_pos,
                velocities_rad_s=ordered_vel,
                time_from_start_s=t_sec,
                segment_duration_s=max(0.0, seg_dt),
                pymycobot_speed=speed,
            )
        )
        prev_time = t_sec
        prev_positions = ordered_pos

    # Drop leading waypoint if it is at t == 0 and already matches current pose,
    # unless it is the only waypoint in the trajectory.
    if len(waypoints) > 1 and waypoints[0].time_from_start_s <= 1e-3:
        waypoints = waypoints[1:]

    return ValidationResult(True, 'OK', ERROR_NONE, waypoints)


def _can_coalesce_subpath(
    anchor_pos: Sequence[float],
    sub_waypoints: Sequence[WaypointSpec],
    collinear_tolerance_rad: float,
) -> bool:
    """
    Check whether intermediate waypoints can be merged into a single linear segment.

    Returns True if all intermediate points in ``sub_waypoints[:-1]`` lie within
    ``collinear_tolerance_rad`` of the straight 6D joint-space line from
    ``anchor_pos`` to ``sub_waypoints[-1].positions_rad`` and move monotonically.
    """
    if len(sub_waypoints) <= 1:
        return True

    end_pos = sub_waypoints[-1].positions_rad
    delta = [b - a for a, b in zip(anchor_pos, end_pos)]
    norm_sq = sum(d * d for d in delta)

    prev_pos = list(anchor_pos)
    prev_s = 0.0

    for wp in sub_waypoints[:-1]:
        pos = wp.positions_rad
        if norm_sq <= 1e-12:
            # Segment returns to anchor_pos; any intermediate excursion > tol is a reversal.
            max_dev = max(abs(p - a) for p, a in zip(pos, anchor_pos))
            if max_dev > collinear_tolerance_rad:
                return False
            s = 0.0
        else:
            dot = sum((p - a) * d for p, a, d in zip(pos, anchor_pos, delta))
            s = dot / norm_sq
            if s < -1e-4 or s > 1.0 + 1e-4:
                return False
            if s < prev_s - 1e-4:
                return False
            s_clamped = max(0.0, min(1.0, s))
            max_dev = max(
                abs(p - (a + s_clamped * d))
                for p, a, d in zip(pos, anchor_pos, delta)
            )
            if max_dev > collinear_tolerance_rad:
                return False

        # Check per-joint monotonicity against segment direction.
        for p_prev, p_curr, d in zip(prev_pos, pos, delta):
            step = p_curr - p_prev
            if abs(d) <= collinear_tolerance_rad:
                if abs(step) > collinear_tolerance_rad:
                    return False
            elif step * d < 0.0 and abs(step) > collinear_tolerance_rad:
                return False

        prev_pos = list(pos)
        prev_s = max(prev_s, s)

    # Also check final step from last intermediate point to end_pos.
    for p_prev, p_end, d in zip(prev_pos, end_pos, delta):
        step = p_end - p_prev
        if abs(d) <= collinear_tolerance_rad:
            if abs(step) > collinear_tolerance_rad:
                return False
        elif step * d < 0.0 and abs(step) > collinear_tolerance_rad:
            return False

    return True


def coalesce_waypoints(
    start_positions_rad: Sequence[float],
    waypoints: Sequence[WaypointSpec],
    max_joint_velocity_rad_s: float = 1.0,
    default_pymycobot_speed: int = 15,
    min_pymycobot_speed: int = 5,
    max_pymycobot_speed: int = 30,
    collinear_tolerance_rad: float = 0.02,
    min_waypoint_separation_rad: float = 0.035,
    min_segment_duration_s: float = 0.20,
) -> List[WaypointSpec]:
    """
    Coalesce dense time-parameterized waypoints into sparse point-to-point commands.

    MoveIt 2 time parameterization (TOTG) subdivides smooth joint-space paths into
    many closely spaced samples (~10 Hz). Because ``MyCobot280.send_angles()`` is a
    point-to-point command with its own internal servo velocity profile that
    decelerates to zero at each target, sending every dense sample causes repeated
    stop-and-go motion or saturates the firmware command queue.

    This function merges intermediate waypoints that are collinear in 6D joint space
    (within ``collinear_tolerance_rad``) or closer than ``min_waypoint_separation_rad``
    / ``min_segment_duration_s``, while preserving true direction changes / corners
    and always keeping the exact final goal waypoint.
    """
    if len(waypoints) <= 1:
        return list(waypoints)

    tol = max(0.0, float(collinear_tolerance_rad))
    min_sep = max(0.0, float(min_waypoint_separation_rad))
    min_dt = max(0.0, float(min_segment_duration_s))

    coalesced: List[WaypointSpec] = []
    anchor_pos = list(start_positions_rad)
    anchor_time = 0.0
    seg_start_idx = 0
    n = len(waypoints)

    while seg_start_idx < n:
        best_end_idx = seg_start_idx
        for cand_idx in range(seg_start_idx + 1, n):
            sub = waypoints[seg_start_idx:cand_idx + 1]
            if _can_coalesce_subpath(anchor_pos, sub, tol):
                best_end_idx = cand_idx
            else:
                cand_wp = waypoints[best_end_idx]
                max_disp = max(
                    abs(b - a) for a, b in zip(anchor_pos, cand_wp.positions_rad)
                )
                dt = cand_wp.time_from_start_s - anchor_time
                next_pos = waypoints[cand_idx].positions_rad
                reverses = any(
                    (c - a) * (nx - c) < 0.0 and abs(c - a) > tol
                    for a, c, nx in zip(anchor_pos, cand_wp.positions_rad, next_pos)
                )
                if (max_disp < min_sep or dt < min_dt) and not reverses:
                    best_end_idx = cand_idx
                    continue
                break

        chosen = waypoints[best_end_idx]
        seg_dt = max(0.0, chosen.time_from_start_s - anchor_time)
        speed = compute_pymycobot_speed(
            anchor_pos,
            chosen.positions_rad,
            seg_dt,
            max_joint_velocity_rad_s,
            default_pymycobot_speed,
            min_pymycobot_speed,
            max_pymycobot_speed,
        )
        coalesced.append(
            WaypointSpec(
                positions_rad=list(chosen.positions_rad),
                velocities_rad_s=list(chosen.velocities_rad_s),
                time_from_start_s=chosen.time_from_start_s,
                segment_duration_s=seg_dt,
                pymycobot_speed=speed,
            )
        )
        anchor_pos = list(chosen.positions_rad)
        anchor_time = chosen.time_from_start_s
        seg_start_idx = best_end_idx + 1

    return coalesced


def interpolate_desired_waypoint(
    waypoints: Sequence[WaypointSpec],
    start_positions_rad: Sequence[float],
    elapsed_s: float,
) -> WaypointSpec:
    """Return the time-interpolated desired WaypointSpec at ``elapsed_s``."""
    if not waypoints:
        raise ValueError('waypoints must not be empty')
    if elapsed_s >= waypoints[-1].time_from_start_s:
        return waypoints[-1]
    if elapsed_s <= 0.0:
        first = waypoints[0]
        return WaypointSpec(
            positions_rad=list(start_positions_rad),
            velocities_rad_s=[0.0] * NUM_JOINTS,
            time_from_start_s=0.0,
            segment_duration_s=first.segment_duration_s,
            pymycobot_speed=first.pymycobot_speed,
        )

    prev_t = 0.0
    prev_pos = list(start_positions_rad)
    prev_vel = [0.0] * NUM_JOINTS

    for wp in waypoints:
        if elapsed_s <= wp.time_from_start_s:
            dt = wp.time_from_start_s - prev_t
            alpha = (elapsed_s - prev_t) / dt if dt > 1e-6 else 1.0
            alpha = max(0.0, min(1.0, alpha))
            interp_pos = [
                a + alpha * (b - a) for a, b in zip(prev_pos, wp.positions_rad)
            ]
            interp_vel = [
                a + alpha * (b - a) for a, b in zip(prev_vel, wp.velocities_rad_s)
            ]
            return WaypointSpec(
                positions_rad=interp_pos,
                velocities_rad_s=interp_vel,
                time_from_start_s=elapsed_s,
                segment_duration_s=wp.segment_duration_s,
                pymycobot_speed=wp.pymycobot_speed,
            )
        prev_t = wp.time_from_start_s
        prev_pos = list(wp.positions_rad)
        prev_vel = list(wp.velocities_rad_s)

    return waypoints[-1]
