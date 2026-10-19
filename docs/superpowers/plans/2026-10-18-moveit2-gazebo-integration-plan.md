# Milestone B1: MoveIt 2 + Gazebo Integration Implementation Plan

- **Date:** 2026-10-18
- **Spec:** [`docs/superpowers/specs/2026-10-18-moveit2-gazebo-integration-design.md`](../specs/2026-10-18-moveit2-gazebo-integration-design.md)
- **Target Package:** `mycobot_280/mycobot_280_moveit2`

## Step 1: Create Gazebo-Aligned SRDF (`config/mycobot_280_m5.srdf`)
- Set `<robot name="mycobot_280_m5">` to match `mycobot_280_gazebo/urdf/mycobot_280_m5_gazebo.urdf.xacro`.
- Define `<group name="arm_group">` with `<chain base_link="g_base" tip_link="joint6_flange"/>`, covering the exact 6 revolute joints in controller order.
- Define `<group_state>` presets (`init_pose`, `home`, `ready`).
- Configure reviewed `<disable_collisions>` pairs (keeping distal link vs. base collision checking active).

## Step 2: Create Gazebo Controller Mapping (`config/moveit_controllers_gazebo.yaml`)
- Map `MoveItSimpleControllerManager` to `mycobot280m5_group_controller` (`FollowJointTrajectory`, `action_ns: follow_joint_trajectory`, `default: true`) with the 6 joints in exact order matching `mycobot_280_gazebo/config/ros2_controllers.yaml`.

## Step 3: Update `config/joint_limits.yaml` and `config/kinematics.yaml`
- Cross-check all 6 joints against `mycobot_280_m5_gazebo.urdf.xacro`.
- Set conservative scaling factors (`default_velocity_scaling_factor: 0.1`, `default_acceleration_scaling_factor: 0.1`) and conservative per-joint limits (`max_velocity: 1.0`, `max_acceleration: 1.0`, within the `2.0` rad/s URDF limit) plus explicit `min_position`/`max_position` from the Gazebo URDF.
- Configure `kdl_kinematics_plugin/KDLKinematicsPlugin` for `arm_group` in `config/kinematics.yaml`.

## Step 4: Create Standalone MoveIt 2 Gazebo Launcher (`launch/moveit_gazebo.launch.py`)
- Load `robot_description` from `mycobot_280_gazebo/urdf/mycobot_280_m5_gazebo.urdf.xacro` and `robot_description_semantic` from `mycobot_280_moveit2/config/mycobot_280_m5.srdf`.
- Start **only** `move_group` and `rviz2` with `use_sim_time: true`.
- Ensure no automatic motion is triggered on startup and no duplicate Gazebo, `robot_state_publisher`, `controller_manager`, `joint_state_publisher`, or `mock_components` are launched.

## Step 5: Update `package.xml` and Run Automated Consistency Tests
- Add `<exec_depend>mycobot_280_gazebo</exec_depend>` to `mycobot_280_moveit2/package.xml`.
- Create and execute `mycobot_280/mycobot_280_moveit2/test/test_moveit2_gazebo_config.py` verifying all URDF/SRDF/controller/joint-limit/launcher requirements.
