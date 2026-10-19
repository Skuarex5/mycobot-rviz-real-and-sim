"""Domain models and pure configuration/safety validators for pick-and-place motion."""

from dataclasses import dataclass
import math
from typing import Any, Dict, List, Mapping, Sequence, Tuple
import yaml


CANONICAL_JOINT_NAMES: Tuple[str, ...] = (
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
)

URDF_JOINT_LIMITS: Dict[str, Tuple[float, float]] = {
    "joint2_to_joint1": (-2.9321, 2.9321),
    "joint3_to_joint2": (-2.4434, 2.4434),
    "joint4_to_joint3": (-2.6179, 2.6179),
    "joint5_to_joint4": (-2.6179, 2.6179),
    "joint6_to_joint5": (-2.7052, 2.7925),
    "joint6output_to_joint6": (-3.14159, 3.14159),
}

URDF_MAX_JOINT_VELOCITY_RAD_S: float = 2.0

REQUIRED_SEQUENCE: Tuple[str, ...] = (
    "HOME",
    "PRE_PICK",
    "PICK",
    "LIFT",
    "TRANSFER",
    "PRE_PLACE",
    "PLACE",
    "RETREAT",
    "HOME",
)


class ValidationError(ValueError):
    """Raised when configuration, waypoint, or runtime state validation fails."""


@dataclass(frozen=True)
class JointLimit:
    """Lower and upper position bounds (in radians) for a single revolute joint."""

    lower: float
    upper: float


@dataclass(frozen=True)
class WaypointSpec:
    """Validated motion waypoint specification (joint-space now, extensible to Cartesian)."""

    name: str
    positions: Tuple[float, ...]
    duration_sec: float


@dataclass(frozen=True)
class MarkerLocation:
    """Nominal 3D visualization marker coordinates in the reference frame."""

    label: str
    xyz: Tuple[float, float, float]
    rgba: Tuple[float, float, float, float]


@dataclass(frozen=True)
class PickPlaceConfig:
    """Complete, validated configuration for a pick-and-place demonstration run."""

    simulation_only: bool
    controller_action_name: str
    joint_states_topic: str
    publish_rviz_markers: bool
    marker_topic: str
    marker_frame_id: str
    nominal_pick_xyz: Tuple[float, float, float]
    nominal_place_xyz: Tuple[float, float, float]
    controller_wait_timeout_sec: float
    joint_state_wait_timeout_sec: float
    goal_response_timeout_sec: float
    execution_timeout_margin_sec: float
    pause_between_steps_sec: float
    min_duration_sec: float
    max_duration_sec: float
    max_joint_velocity_rad_s: float
    position_tolerance_rad: float
    joint_names: Tuple[str, ...]
    joint_limits: Dict[str, JointLimit]
    sequence_order: Tuple[str, ...]
    waypoints: Dict[str, WaypointSpec]


@dataclass(frozen=True)
class StepExecutionResult:
    """Outcome of executing a single step in the motion sequence."""

    success: bool
    stage_name: str
    error_code: int
    message: str


def _require_finite_float(value: Any, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValidationError(f"Field '{field_name}' must be a finite number, got {value!r}.")
    fval = float(value)
    if not math.isfinite(fval):
        raise ValidationError(f"Field '{field_name}' must be finite, got {value!r}.")
    return fval


def _require_positive_float(value: Any, field_name: str, allow_zero: bool = False) -> float:
    fval = _require_finite_float(value, field_name)
    if allow_zero and fval < 0.0:
        raise ValidationError(f"Field '{field_name}' must be >= 0.0, got {fval}.")
    if not allow_zero and fval <= 0.0:
        raise ValidationError(f"Field '{field_name}' must be > 0.0, got {fval}.")
    return fval


def _require_xyz_tuple(value: Any, field_name: str) -> Tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValidationError(f"Field '{field_name}' must be a 3-element [x, y, z] list.")
    return (
        _require_finite_float(value[0], f"{field_name}[0]"),
        _require_finite_float(value[1], f"{field_name}[1]"),
        _require_finite_float(value[2], f"{field_name}[2]"),
    )


def validate_config_dict(raw_data: Mapping[str, Any]) -> PickPlaceConfig:
    """Validate raw dictionary data and return an immutable PickPlaceConfig."""
    if not isinstance(raw_data, Mapping):
        raise ValidationError("Configuration root must be a mapping.")

    # Support both ROS 2 parameter YAML layout and flat layout
    if "pick_place_demo_node" in raw_data:
        node_block = raw_data["pick_place_demo_node"]
        if not isinstance(node_block, Mapping) or "ros__parameters" not in node_block:
            raise ValidationError(
                "'pick_place_demo_node' must contain a 'ros__parameters' mapping."
            )
        params = node_block["ros__parameters"]
    elif "ros__parameters" in raw_data:
        params = raw_data["ros__parameters"]
    else:
        params = raw_data

    if not isinstance(params, Mapping):
        raise ValidationError("Parameters block must be a mapping.")

    simulation_only = params.get("simulation_only")
    if simulation_only is not True:
        raise ValidationError(
            "Safety safeguard triggered: 'simulation_only' must be explicitly set to true "
            "for Milestone A."
        )

    controller_action_name = params.get(
        "controller_action_name",
        "/mycobot280m5_group_controller/follow_joint_trajectory",
    )
    if not isinstance(controller_action_name, str) or not controller_action_name.strip():
        raise ValidationError("'controller_action_name' must be a non-empty string.")

    joint_states_topic = params.get("joint_states_topic", "/joint_states")
    if not isinstance(joint_states_topic, str) or not joint_states_topic.strip():
        raise ValidationError("'joint_states_topic' must be a non-empty string.")

    publish_rviz_markers = params.get("publish_rviz_markers", True)
    if not isinstance(publish_rviz_markers, bool):
        raise ValidationError("'publish_rviz_markers' must be a boolean.")

    marker_topic = params.get("marker_topic", "/pick_place_markers")
    if not isinstance(marker_topic, str) or not marker_topic.strip():
        raise ValidationError("'marker_topic' must be a non-empty string.")

    marker_frame_id = params.get("marker_frame_id", "g_base")
    if not isinstance(marker_frame_id, str) or not marker_frame_id.strip():
        raise ValidationError("'marker_frame_id' must be a non-empty string.")

    nominal_pick_xyz = _require_xyz_tuple(
        params.get("nominal_pick_xyz", [0.13, 0.10, 0.05]),
        "nominal_pick_xyz",
    )
    nominal_place_xyz = _require_xyz_tuple(
        params.get("nominal_place_xyz", [0.13, -0.10, 0.05]),
        "nominal_place_xyz",
    )

    controller_wait_timeout_sec = _require_positive_float(
        params.get("controller_wait_timeout_sec", 10.0),
        "controller_wait_timeout_sec",
    )
    joint_state_wait_timeout_sec = _require_positive_float(
        params.get("joint_state_wait_timeout_sec", 5.0),
        "joint_state_wait_timeout_sec",
    )
    goal_response_timeout_sec = _require_positive_float(
        params.get("goal_response_timeout_sec", 5.0),
        "goal_response_timeout_sec",
    )
    execution_timeout_margin_sec = _require_positive_float(
        params.get("execution_timeout_margin_sec", 3.0),
        "execution_timeout_margin_sec",
    )
    pause_between_steps_sec = _require_positive_float(
        params.get("pause_between_steps_sec", 0.3),
        "pause_between_steps_sec",
        allow_zero=True,
    )
    min_duration_sec = _require_positive_float(
        params.get("min_duration_sec", 1.0),
        "min_duration_sec",
    )
    max_duration_sec = _require_positive_float(
        params.get("max_duration_sec", 15.0),
        "max_duration_sec",
    )
    if min_duration_sec > max_duration_sec:
        raise ValidationError(
            f"'min_duration_sec' ({min_duration_sec}) cannot exceed "
            f"'max_duration_sec' ({max_duration_sec})."
        )

    max_joint_velocity_rad_s = _require_positive_float(
        params.get("max_joint_velocity_rad_s", 1.5),
        "max_joint_velocity_rad_s",
    )
    if max_joint_velocity_rad_s > URDF_MAX_JOINT_VELOCITY_RAD_S:
        raise ValidationError(
            f"'max_joint_velocity_rad_s' ({max_joint_velocity_rad_s}) exceeds URDF "
            f"velocity limit ({URDF_MAX_JOINT_VELOCITY_RAD_S} rad/s)."
        )

    position_tolerance_rad = _require_positive_float(
        params.get("position_tolerance_rad", 0.15),
        "position_tolerance_rad",
    )

    # Validate joint_names
    raw_joint_names = params.get("joint_names")
    if not isinstance(raw_joint_names, (list, tuple)):
        raise ValidationError("'joint_names' must be a list of 6 joint names.")
    joint_names = tuple(str(j) for j in raw_joint_names)
    if joint_names != CANONICAL_JOINT_NAMES:
        raise ValidationError(
            f"'joint_names' must match canonical myCobot 280 M5 order "
            f"{list(CANONICAL_JOINT_NAMES)}, got {list(joint_names)}."
        )

    # Validate joint_limits against URDF bounds
    raw_limits = params.get("joint_limits")
    if not isinstance(raw_limits, Mapping):
        raise ValidationError("'joint_limits' must be a mapping of joint bounds.")
    joint_limits: Dict[str, JointLimit] = {}
    for jname in CANONICAL_JOINT_NAMES:
        if jname not in raw_limits:
            raise ValidationError(f"Missing joint limit definition for '{jname}'.")
        bounds = raw_limits[jname]
        if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
            raise ValidationError(
                f"Joint limit for '{jname}' must be a 2-element [lower, upper] list."
            )
        lower = _require_finite_float(bounds[0], f"joint_limits.{jname}.lower")
        upper = _require_finite_float(bounds[1], f"joint_limits.{jname}.upper")
        if lower >= upper:
            raise ValidationError(
                f"Joint limit for '{jname}' has lower ({lower}) >= upper ({upper})."
            )
        urdf_lower, urdf_upper = URDF_JOINT_LIMITS[jname]
        if lower < urdf_lower - 1e-6 or upper > urdf_upper + 1e-6:
            raise ValidationError(
                f"Joint limit for '{jname}' [{lower}, {upper}] exceeds URDF limits "
                f"[{urdf_lower}, {urdf_upper}]."
            )
        joint_limits[jname] = JointLimit(lower=lower, upper=upper)

    # Validate sequence_order
    raw_sequence = params.get("sequence_order")
    if not isinstance(raw_sequence, (list, tuple)):
        raise ValidationError("'sequence_order' must be a list of stage names.")
    sequence_order = tuple(str(s) for s in raw_sequence)
    if sequence_order != REQUIRED_SEQUENCE:
        raise ValidationError(
            f"'sequence_order' must match required sequence {list(REQUIRED_SEQUENCE)}, "
            f"got {list(sequence_order)}."
        )

    # Validate waypoints
    raw_waypoints = params.get("waypoints")
    if not isinstance(raw_waypoints, Mapping):
        raise ValidationError("'waypoints' must be a mapping of named waypoints.")

    waypoints: Dict[str, WaypointSpec] = {}
    for stage_name in set(sequence_order):
        if stage_name not in raw_waypoints:
            raise ValidationError(
                f"Stage '{stage_name}' in sequence_order is missing from 'waypoints'."
            )
        wp_data = raw_waypoints[stage_name]
        if not isinstance(wp_data, Mapping):
            raise ValidationError(f"Waypoint '{stage_name}' must be a mapping.")

        raw_positions = wp_data.get("positions")
        if not isinstance(raw_positions, (list, tuple)) or len(raw_positions) != len(
            CANONICAL_JOINT_NAMES
        ):
            raise ValidationError(
                f"Waypoint '{stage_name}' must have exactly {len(CANONICAL_JOINT_NAMES)} "
                f"joint positions, got {raw_positions!r}."
            )

        positions_list: List[float] = []
        for idx, jname in enumerate(CANONICAL_JOINT_NAMES):
            pos = _require_finite_float(
                raw_positions[idx], f"waypoints.{stage_name}.positions[{idx}]"
            )
            jlim = joint_limits[jname]
            if pos < jlim.lower or pos > jlim.upper:
                raise ValidationError(
                    f"Waypoint '{stage_name}' joint '{jname}' position {pos:.4f} rad is "
                    f"outside limits [{jlim.lower:.4f}, {jlim.upper:.4f}] rad."
                )
            positions_list.append(pos)

        duration_sec = _require_positive_float(
            wp_data.get("duration_sec"), f"waypoints.{stage_name}.duration_sec"
        )
        if duration_sec < min_duration_sec or duration_sec > max_duration_sec:
            raise ValidationError(
                f"Waypoint '{stage_name}' duration_sec ({duration_sec}s) must be within "
                f"[{min_duration_sec}, {max_duration_sec}]s."
            )

        waypoints[stage_name] = WaypointSpec(
            name=stage_name,
            positions=tuple(positions_list),
            duration_sec=duration_sec,
        )

    # Validate consecutive step velocities across the sequence
    for step_idx in range(1, len(sequence_order)):
        prev_wp = waypoints[sequence_order[step_idx - 1]]
        curr_wp = waypoints[sequence_order[step_idx]]
        validate_transition_velocity(
            start_positions=prev_wp.positions,
            target_waypoint=curr_wp,
            max_velocity_rad_s=max_joint_velocity_rad_s,
            context_label=f"{prev_wp.name} -> {curr_wp.name}",
        )

    return PickPlaceConfig(
        simulation_only=True,
        controller_action_name=controller_action_name,
        joint_states_topic=joint_states_topic,
        publish_rviz_markers=publish_rviz_markers,
        marker_topic=marker_topic,
        marker_frame_id=marker_frame_id,
        nominal_pick_xyz=nominal_pick_xyz,
        nominal_place_xyz=nominal_place_xyz,
        controller_wait_timeout_sec=controller_wait_timeout_sec,
        joint_state_wait_timeout_sec=joint_state_wait_timeout_sec,
        goal_response_timeout_sec=goal_response_timeout_sec,
        execution_timeout_margin_sec=execution_timeout_margin_sec,
        pause_between_steps_sec=pause_between_steps_sec,
        min_duration_sec=min_duration_sec,
        max_duration_sec=max_duration_sec,
        max_joint_velocity_rad_s=max_joint_velocity_rad_s,
        position_tolerance_rad=position_tolerance_rad,
        joint_names=joint_names,
        joint_limits=joint_limits,
        sequence_order=sequence_order,
        waypoints=waypoints,
    )


def load_and_validate_config(yaml_path: str) -> PickPlaceConfig:
    """Load a YAML file from disk and validate it into a PickPlaceConfig."""
    try:
        with open(yaml_path, "r", encoding="utf-8") as f:
            raw_data = yaml.safe_load(f)
    except OSError as exc:
        raise ValidationError(f"Failed to read configuration file '{yaml_path}': {exc}") from exc
    except yaml.YAMLError as exc:
        raise ValidationError(f"Invalid YAML syntax in '{yaml_path}': {exc}") from exc

    return validate_config_dict(raw_data)


def validate_transition_velocity(
    start_positions: Sequence[float],
    target_waypoint: WaypointSpec,
    max_velocity_rad_s: float,
    context_label: str = "transition",
) -> None:
    """Verify that estimated joint velocities for a transition do not exceed limits."""
    if len(start_positions) != len(target_waypoint.positions):
        raise ValidationError(
            f"Start positions length ({len(start_positions)}) does not match "
            f"target waypoint '{target_waypoint.name}' length ({len(target_waypoint.positions)})."
        )
    for idx, jname in enumerate(CANONICAL_JOINT_NAMES):
        delta = abs(target_waypoint.positions[idx] - float(start_positions[idx]))
        est_vel = delta / target_waypoint.duration_sec
        if est_vel > max_velocity_rad_s:
            raise ValidationError(
                f"Transition '{context_label}' requires estimated velocity "
                f"{est_vel:.3f} rad/s on joint '{jname}', exceeding max_joint_velocity_rad_s "
                f"({max_velocity_rad_s:.3f} rad/s)."
            )


def extract_ordered_joint_positions(
    msg_joint_names: Sequence[str],
    msg_positions: Sequence[float],
    expected_joint_names: Sequence[str],
    joint_limits: Mapping[str, JointLimit],
) -> Tuple[float, ...]:
    """Validate a JointState message and return positions ordered by expected_joint_names."""
    if len(msg_joint_names) != len(msg_positions):
        raise ValidationError(
            f"JointState name count ({len(msg_joint_names)}) does not match "
            f"position count ({len(msg_positions)})."
        )

    state_map: Dict[str, float] = {}
    for name, pos in zip(msg_joint_names, msg_positions):
        fpos = _require_finite_float(pos, f"joint_states.{name}")
        state_map[str(name)] = fpos

    ordered: List[float] = []
    for jname in expected_joint_names:
        if jname not in state_map:
            raise ValidationError(f"Required joint '{jname}' missing from /joint_states message.")
        pos = state_map[jname]
        jlim = joint_limits[jname]
        # Allow a small numerical margin (0.05 rad) for Gazebo controller overshoot at bounds
        if pos < jlim.lower - 0.05 or pos > jlim.upper + 0.05:
            raise ValidationError(
                f"Current joint '{jname}' position {pos:.4f} rad in /joint_states is "
                f"outside safe limits [{jlim.lower:.4f}, {jlim.upper:.4f}] rad."
            )
        ordered.append(pos)

    return tuple(ordered)


def compute_max_position_error(
    current_positions: Sequence[float],
    target_positions: Sequence[float],
) -> Tuple[float, str]:
    """Return (max_abs_error_rad, joint_name_with_max_error)."""
    if len(current_positions) != len(target_positions):
        raise ValidationError("Position vectors must have equal length to compute error.")
    max_err = -1.0
    worst_joint = CANONICAL_JOINT_NAMES[0]
    for idx, (curr, targ) in enumerate(zip(current_positions, target_positions)):
        err = abs(float(curr) - float(targ))
        if err > max_err:
            max_err = err
            worst_joint = (
                CANONICAL_JOINT_NAMES[idx]
                if idx < len(CANONICAL_JOINT_NAMES)
                else f"joint_{idx}"
            )
    return max_err, worst_joint
