# mycobot_280_hardware

Hardware integration package for the physical **Elephant Robotics myCobot 280 M5** on ROS 2 Humble.

Provides:
1. **`robot_monitor`** (`robot_monitor.launch.py`): strictly read-only `/joint_states` monitor.
2. **`robot_control`** (`robot_control.launch.py` & `moveit_real.launch.py`): single-owner USB serial node that publishes `/joint_states` and serves `/mycobot280m5_group_controller/follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`) for MoveIt 2 **Plan & Execute**.

## Safety Guarantees

- **Motion disabled by default**: `allow_motion` defaults to `false` in `config/robot_control.yaml`, `robot_control.launch.py`, and `moveit_real.launch.py`. When `allow_motion:=false`, MoveIt 2 can plan from the real robot state, but any execution goal is rejected.
- **No startup motion or servo release**: neither node ever calls `release_all_servos`, `release_servo`, `power_on`, or sends angles on startup.
- **Single serial owner**: a single `ControlledArm` instance guarded by a reentrant lock owns `/dev/ttyACM0`, avoiding serial collisions between `/joint_states` polling and trajectory execution.
- **Pre-execution validation**: checks joint names, finiteness, URDF position limits, velocity limits, monotonic timestamps, and start-state deviation against a fresh measured pose.
- **Smooth point-to-point trajectory coalescing & closed-loop supervision**: coalesces collinear/micro-step MoveIt 2 TOTG subdivisions to prevent servo stop-and-go and firmware queue saturation while preserving corner/reversal via-points, dispatches non-blocking serial commands (`_async=True`), publishes time-interpolated `FollowJointTrajectory.Feedback`, verifies final arrival within `goal_tolerance_rad`, and runs a verified `stop()` procedure on cancellation, timeout, or serial failure.
- **No simulation or duplicate publishers**: uses `use_sim_time=false` and starts no Gazebo, `ros2_control`, or `joint_state_publisher(_gui)`.

## Usage (Physical Robot — Operator Supervised)

Before launching with `allow_motion:=true`, confirm:
1. Workspace around the manipulator is clear.
2. Emergency stop / power switch is within reach.
3. Joint offsets/signs and conservative speed limits (`max_speed:=30`) are reviewed.

### 1. Read-Only Monitor (`robot_monitor.launch.py`)

```bash
source /opt/ros/humble/setup.bash
source /home/igor/cobotM5/install/setup.bash
ros2 launch mycobot_280_hardware robot_monitor.launch.py port:=/dev/ttyACM0
```

### 2. MoveIt 2 Plan-Only Mode (Safe Dry-Run Default)

Starts `robot_control` (`allow_motion:=false`), `robot_state_publisher`, `move_group`, and `rviz2`:

```bash
source /opt/ros/humble/setup.bash
source /home/igor/cobotM5/install/setup.bash
ros2 launch mycobot_280_hardware moveit_real.launch.py port:=/dev/ttyACM0
```

### 3. MoveIt 2 Plan & Execute on Physical Robot (Explicit Opt-In)

```bash
source /opt/ros/humble/setup.bash
source /home/igor/cobotM5/install/setup.bash
ros2 launch mycobot_280_hardware moveit_real.launch.py \
  port:=/dev/ttyACM0 allow_motion:=true max_speed:=25
```

Or run the hardware controller and MoveIt 2 in separate terminals:

```bash
# Terminal 1: Hardware controller + robot_state_publisher
ros2 launch mycobot_280_hardware robot_control.launch.py \
  port:=/dev/ttyACM0 allow_motion:=true max_speed:=25

# Terminal 2: MoveIt 2 + RViz connected to the running controller
ros2 launch mycobot_280_hardware moveit_real.launch.py launch_driver:=false
```

## Automated Tests

```bash
colcon build --symlink-install --packages-select mycobot_280_hardware
colcon test --packages-select mycobot_280_hardware
colcon test-result --verbose
```

Mock/unit tests never count as physical hardware validation.
