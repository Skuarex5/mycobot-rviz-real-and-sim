# Milestone A: Simulated Pick-and-Place Implementation Plan

- **Date:** 2026-10-18
- **Spec:** [`docs/superpowers/specs/2026-10-18-pick-place-motion-design.md`](../specs/2026-10-18-pick-place-motion-design.md)
- **Target Package:** `mycobot_280/mycobot_280_pick_place_demo`

## Step 1: Create Package Manifest and Setup Files
- Create `mycobot_280/mycobot_280_pick_place_demo/package.xml` (`ament_python`, dependencies: `rclpy`, `control_msgs`, `trajectory_msgs`, `sensor_msgs`, `visualization_msgs`, `geometry_msgs`, `builtin_interfaces`, `action_msgs`, `launch`, `launch_ros`, `python3-yaml`).
- Create `mycobot_280/mycobot_280_pick_place_demo/setup.cfg` and `setup.py` without any dynamic `setup.cfg` mutation.
- Create resource marker `mycobot_280/mycobot_280_pick_place_demo/resource/mycobot_280_pick_place_demo`.

## Step 2: Implement Pure Domain Models and Validators
- Create `mycobot_280_pick_place_demo/models_and_validators.py`:
  - Canonical joint names and URDF limits dictionary (`URDF_JOINT_LIMITS`).
  - Required 9-step sequence constant (`REQUIRED_SEQUENCE`).
  - Dataclasses: `JointLimit`, `WaypointSpec`, `MarkerSpec`, `PickPlaceConfig`, `StepExecutionResult`.
  - Functions:
    - `load_and_validate_config(yaml_path: str) -> PickPlaceConfig`
    - `validate_joint_state_message(joint_names, positions, limits) -> dict[str, float]`
    - `validate_transition_velocity(start_positions, target_waypoint, max_velocity_rad_s) -> None`
    - `compute_max_position_error(current_positions, target_positions) -> float`

## Step 3: Implement Motion Executor Abstraction
- Create `mycobot_280_pick_place_demo/motion_executor.py`:
  - Abstract base class `MotionExecutor` with `wait_for_ready()` and `execute_waypoint()`.
  - Concrete `FollowJointTrajectoryExecutor` using `rclpy.action.ActionClient` for `control_msgs/action/FollowJointTrajectory`.
  - Enforces timeouts on server availability, goal acceptance, and result completion, canceling active goals on timeout.

## Step 4: Implement Orchestrator Node and RViz Marker Publisher
- Create `mycobot_280_pick_place_demo/pick_place_node.py`:
  - Subscribes to `/joint_states`.
  - Publishes optional `visualization_msgs/msg/MarkerArray` (`TRANSIENT_LOCAL` QoS) on `/pick_place_markers` for nominal `PICK` and `PLACE` locations.
  - Runs pre-flight checks (simulation-only guard, YAML validation, `/joint_states` readiness, controller server readiness).
  - Executes `HOME -> PRE_PICK -> PICK -> LIFT -> TRANSFER -> PRE_PLACE -> PLACE -> RETREAT -> HOME` sequentially once, checking trajectory results and joint position tolerance at each step, then exits cleanly.

## Step 5: Create Default YAML Config and Standalone Launch File
- Create `config/pick_place_waypoints.yaml` with conservative joint-space waypoints, timeouts, and nominal RViz marker coordinates.
- Create `launch/pick_place_demo.launch.py` starting **only** `pick_place_demo_node` (no Gazebo, no spawners, no `start_gazebo` argument).

## Step 6: Implement Automated Unit Tests
- Create `test/test_pick_place_validation.py` testing:
  - Package structure, launch file constraints (no Gazebo/spawner/`start_gazebo`/hardware imports), and `setup.py` hygiene.
  - YAML config loading and validation of the 9-stage sequence.
  - Rejection of invalid joint names, wrong waypoint dimensions, out-of-range joint angles, invalid durations, excessive joint speeds, and `simulation_only=False`.
  - `/joint_states` validation and position error calculation.
- Run `pytest` on `mycobot_280/mycobot_280_pick_place_demo/test/test_pick_place_validation.py` and report exact results.
