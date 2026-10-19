# Milestone A: Simulated Pick-and-Place Motion Specification

- **Date:** 2026-10-18
- **Status:** Approved
- **Repository:** `UruBots/mycobot_ros2_packages` (`dev/limo-cobot-humble`)
- **Target Package:** `mycobot_280/mycobot_280_pick_place_demo`
- **Governing Policy:** [`AGENTS.md`](../../../AGENTS.md)

## 1. Objective and Scope

Design and implement a standalone, simulation-only ROS 2 Humble application (`mycobot_280_pick_place_demo`) that autonomously executes a deterministic pick-and-place motion sequence on the simulated Elephant Robotics myCobot 280 M5 in Gazebo Classic:

`HOME -> PRE_PICK -> PICK -> LIFT -> TRANSFER -> PRE_PLACE -> PLACE -> RETREAT -> HOME`

### In Scope (Milestone A)
- Sequential execution of configurable joint-space waypoints over `/mycobot280m5_group_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`).
- Strict pre-flight validation of YAML configuration, waypoint dimensions, canonical joint names, URDF joint limits, trajectory durations, and estimated joint velocities.
- Runtime monitoring of `/joint_states` (`sensor_msgs/msg/JointState`) and controller availability prior to sending any trajectory goal.
- Single-cycle execution that exits cleanly (`exit code 0` on success, non-zero on error) and can be re-run repeatedly without restarting Gazebo.
- Optional RViz markers (`visualization_msgs/msg/MarkerArray`) to visualize nominal `PICK` and `PLACE` reference zones.
- Clean abstraction layer (`MotionExecutor`) so future milestones can integrate physical hardware or MoveIt 2 Cartesian pose planning without rewriting the sequence state machine.

### Out of Scope (Milestone A)
- Gripper control, object grasping, or Gazebo link attachment.
- Camera perception or TF-based object detection.
- Physical hardware control (`pymycobot`, serial ports `/dev/tty*`).
- Starting Gazebo, spawning models, or spawning controllers inside `pick_place_demo.launch.py`.

---

## 2. Architectural Decisions

### D1. Strict Separation of Launch Files
- Gazebo and `ros2_control` controllers are started exclusively by `ros2 launch mycobot_280_gazebo gazebo.launch.py` in a dedicated terminal.
- `ros2 launch mycobot_280_pick_place_demo pick_place_demo.launch.py` starts **only** the `pick_place_demo_node`.
- `pick_place_demo.launch.py` **must not** declare or implement a `start_gazebo` argument, spawn entities, or interact with `controller_manager` spawners.

### D2. Single-Cycle, Re-runnable Execution
- The node connects to the existing `/mycobot280m5_group_controller/follow_joint_trajectory` action server and `/joint_states` topic.
- If Gazebo, `/joint_states`, or the trajectory action server is unavailable within the configured timeout, the node logs a clear `ERROR` message and exits with a non-zero exit code without sending any motion command.
- The node executes the 9-stage sequence (`HOME` through `HOME`) exactly once and terminates cleanly.
- It can be launched again as many times as needed against the same running Gazebo session.

### D3. Backend-Agnostic Sequence & Future MoveIt 2 Seam
- The state machine orchestrates named steps (`StepSpec`), delegating motion execution to a `MotionExecutor` interface.
- Milestone A provides `FollowJointTrajectoryExecutor` (joint-space goals).
- Future milestones can inject a `MoveItPoseExecutor` (Cartesian `geometry_msgs/msg/PoseStamped` goals) while preserving the sequence validator and state machine.

### D4. Simulation-Only Safeguards
- Parameter `simulation_only` defaults to `True`. If `False` is passed in Milestone A, the node refuses to run and exits with an error.
- Parameter `use_sim_time` defaults to `True` in the launch file.
- No imports of `pymycobot` or references to `/dev/tty*` exist anywhere in `mycobot_280_pick_place_demo`.

---

## 3. Package Layout

```text
mycobot_280/mycobot_280_pick_place_demo/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── mycobot_280_pick_place_demo
├── config/
│   └── pick_place_waypoints.yaml
├── launch/
│   └── pick_place_demo.launch.py
├── mycobot_280_pick_place_demo/
│   ├── __init__.py
│   ├── models_and_validators.py
│   ├── motion_executor.py
│   └── pick_place_node.py
└── test/
    └── test_pick_place_validation.py
```

---

## 4. Validation and Failure Handling Contract

### 4.1 Static Configuration Validation (Pre-Motion)
Before waiting for ROS interfaces or sending any command, `models_and_validators.py` validates:
1. `simulation_only` is `True`.
2. `joint_names` matches the exact 6 revolute joints of the myCobot 280 M5:
   - `joint2_to_joint1` `[-2.9321, 2.9321]`
   - `joint3_to_joint2` `[-2.4434, 2.4434]`
   - `joint4_to_joint3` `[-2.6179, 2.6179]`
   - `joint5_to_joint4` `[-2.6179, 2.6179]`
   - `joint6_to_joint5` `[-2.7052, 2.7925]`
   - `joint6output_to_joint6` `[-3.14159, 3.14159]`
3. `sequence_order` matches `["HOME", "PRE_PICK", "PICK", "LIFT", "TRANSFER", "PRE_PLACE", "PLACE", "RETREAT", "HOME"]`.
4. Every stage in `sequence_order` is defined in `waypoints` with:
   - `positions`: list of exactly 6 finite floats (`math.isfinite`), each within the joint's `[lower, upper]` limit.
   - `duration_sec`: finite float within `[min_duration_sec, max_duration_sec]`.
5. Estimated joint speed between consecutive steps $|\Delta q_i| / \text{duration\_sec} \le \text{max\_joint\_velocity\_rad\_s}$ (and $\le 2.0\text{ rad/s}$ URDF limit).

### 4.2 Runtime Readiness & Execution Checks
1. **Joint States Check:** Wait up to `joint_state_wait_timeout_sec` for a `/joint_states` message containing all 6 expected joints with finite position values within URDF bounds. Validate initial transition speed from the current robot pose to step 1 (`HOME`).
2. **Controller Availability Check:** Call `wait_for_server(timeout_sec=controller_wait_timeout_sec)` on `/mycobot280m5_group_controller/follow_joint_trajectory`. If unavailable, log a clear error and abort.
3. **Sequential Step Execution:**
   - Send `FollowJointTrajectory.Goal` with `positions`, `velocities=[0.0]*6`, and `time_from_start=duration_sec`.
   - Wait for goal acceptance within `goal_response_timeout_sec`.
   - Wait for result within `duration_sec + execution_timeout_margin_sec`.
   - Verify `status == GoalStatus.STATUS_SUCCEEDED` and `result.error_code == FollowJointTrajectory.Result.SUCCESSFUL`.
   - Verify latest `/joint_states` max absolute error against target positions is within `position_tolerance_rad`.
   - On any failure or timeout, cancel any active goal, log the failing stage and reason, and stop immediately with non-zero exit code.
