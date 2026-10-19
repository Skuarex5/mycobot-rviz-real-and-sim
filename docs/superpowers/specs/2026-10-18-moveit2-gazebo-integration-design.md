# Milestone B1: MoveIt 2 Integration with myCobot 280 M5 Gazebo Simulation

- **Date:** 2026-10-18
- **Status:** Approved (Revision 2 — Incorporating mandatory adjustments on joint limits, URDF/SRDF collision & chain consistency, and strict Gazebo integration)
- **Repository:** `UruBots/mycobot_ros2_packages` (`dev/limo-cobot-humble`)
- **Target Package:** `mycobot_280/mycobot_280_moveit2`
- **Governing Policy:** [`AGENTS.md`](../../../AGENTS.md)

## 1. Objective and Acceptance Criterion

Integrate MoveIt 2 with the existing myCobot 280 M5 Gazebo Classic simulation (`mycobot_280_gazebo`) so that the user can interactively plan reachable poses in RViz and execute the planned trajectories directly on the simulated robot over `/mycobot280m5_group_controller/follow_joint_trajectory` using **Plan & Execute**.

### Acceptance Criterion
1. Terminal 1 runs `ros2 launch mycobot_280_gazebo gazebo.launch.py`.
2. Terminal 2 runs `ros2 launch mycobot_280_moveit2 moveit_gazebo.launch.py`.
3. MoveIt 2 starts with **no automatic robot motion on startup**, connects to the existing `/joint_states` and `/mycobot280m5_group_controller/follow_joint_trajectory`, and in RViz (`MotionPlanning` display) the user defines a reachable target pose and executes it in Gazebo via **Plan & Execute**.

---

## 2. Verified URDF / SRDF / Controller Consistency

### 2.1 URDF Joint Limits Inspection (`mycobot_280_gazebo/urdf/mycobot_280_m5_gazebo.urdf.xacro`)
Inspection of lines 178–230 of `mycobot_280_m5_gazebo.urdf.xacro` confirms:
- `<robot name="mycobot_280_m5">`
- Kinematic chain: `world` -> (fixed `world_to_g_base`) -> `g_base` -> (fixed `g_base_to_joint1`) -> `joint1` -> `joint2` -> `joint3` -> `joint4` -> `joint5` -> `joint6` -> `joint6_flange`.
- 6 revolute joints in exact controller order (`mycobot_280_gazebo/config/ros2_controllers.yaml`):
  1. `joint2_to_joint1`: `lower="-2.9321"`, `upper="2.9321"`, `effort="1000.0"`, `velocity="2.0"`
  2. `joint3_to_joint2`: `lower="-2.4434"`, `upper="2.4434"`, `effort="1000.0"`, `velocity="2.0"`
  3. `joint4_to_joint3`: `lower="-2.6179"`, `upper="2.6179"`, `effort="1000.0"`, `velocity="2.0"`
  4. `joint5_to_joint4`: `lower="-2.6179"`, `upper="2.6179"`, `effort="1000.0"`, `velocity="2.0"`
  5. `joint6_to_joint5`: `lower="-2.7052"`, `upper="2.7925"`, `effort="1000.0"`, `velocity="2.0"`
  6. `joint6output_to_joint6`: `lower="-3.14159"`, `upper="3.14159"`, `effort="1000.0"`, `velocity="2.0"`

### 2.2 Conservative MoveIt 2 Joint Limits (`config/joint_limits.yaml`)
- Instead of assuming `2.0 rad/s` as a safe operating speed, `joint_limits.yaml` enforces conservative MoveIt 2 bounds (`max_velocity: 1.0` rad/s, `max_acceleration: 1.0` rad/s², which are $\le$ the URDF ceiling of `2.0` rad/s) combined with conservative default scaling factors:
  - `default_velocity_scaling_factor: 0.1`
  - `default_acceleration_scaling_factor: 0.1`
- Explicit `min_position` and `max_position` match the Gazebo URDF limits exactly.
- Programmatic consistency tests verify that `joint_limits.yaml` bounds never exceed the URDF limits in `mycobot_280_m5_gazebo.urdf.xacro`.

### 2.3 SRDF Planning Group & Collision Matrix (`config/mycobot_280_m5.srdf`)
- `<robot name="mycobot_280_m5">` matches `mycobot_280_m5_gazebo.urdf.xacro`.
- `<group name="arm_group">` uses `<chain base_link="g_base" tip_link="joint6_flange"/>`, which spans `g_base_to_joint1` (fixed) and the exact 6 movable revolute joints (`joint2_to_joint1`, `joint3_to_joint2`, `joint4_to_joint3`, `joint5_to_joint4`, `joint6_to_joint5`, `joint6output_to_joint6`).
- **Collision Matrix Review:**
  - `firefighter.srdf` previously disabled collisions between `g_base` and **all** links (`joint1` through `joint6_flange` marked as `"Never"`), which prevented MoveIt 2 from detecting collisions between the distal arm (`joint4`, `joint5`, `joint6`, `joint6_flange`) and the base mount (`g_base`).
  - In `mycobot_280_m5.srdf`, we disable only physically adjacent or kinematic-geometry-guaranteed non-colliding pairs:
    - Adjacent link pairs: `(g_base, joint1)`, `(joint1, joint2)`, `(joint2, joint3)`, `(joint3, joint4)`, `(joint4, joint5)`, `(joint5, joint6)`, `(joint6, joint6_flange)`.
    - Never-colliding close pairs due to link geometry/mesh overlap: `(g_base, joint2)`, `(g_base, joint3)`, `(joint5, joint6_flange)`.
  - Distal links (`joint4`, `joint5`, `joint6`, `joint6_flange`) remain collision-checked against `g_base` and `joint1`/`joint2`.

### 2.4 Gazebo Integration (`launch/moveit_gazebo.launch.py` & `config/moveit_controllers_gazebo.yaml`)
- Connects to existing `/joint_states` and `/mycobot280m5_group_controller/follow_joint_trajectory`.
- Parses `mycobot_280_gazebo/urdf/mycobot_280_m5_gazebo.urdf.xacro` directly so `move_group` and `rviz2` share the exact kinematic model with Gazebo.
- Launches **only** `move_group` and `rviz2` with `use_sim_time: true`.
- Does **not** start Gazebo, `controller_manager`, `robot_state_publisher`, `joint_state_publisher`, spawners, or `mock_components`.
- Does **not** send any motion command on startup.
