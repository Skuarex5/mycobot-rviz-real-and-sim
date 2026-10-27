from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[3]


def test_gripper_controller_has_only_principal_joint():
    path = ROOT / "mycobot_280/mycobot_280_gazebo/config/ros2_controllers_gripper.yaml"
    assert path.is_file(), "gripper controllers configuration is missing"
    data = yaml.safe_load(path.read_text())
    assert data["mycobot280m5_gripper_controller"]["ros__parameters"]["joints"] == [
        "gripper_controller"
    ]
    assert len(data["mycobot280m5_group_controller"]["ros__parameters"]["joints"]) == 6


def test_live_controllers_match_trajectory_joints_in_each_variant():
    for gripper in (False, True):
        suffix = "_gripper" if gripper else ""
        data = yaml.safe_load((ROOT / f"mycobot_280/mycobot_280_gazebo/config/ros2_controllers{suffix}.yaml").read_text())
        kinds = data["controller_manager"]["ros__parameters"]
        for live, trajectory in [("mycobot280m5_live_arm_controller", "mycobot280m5_group_controller")] + (
            [("mycobot280m5_live_gripper_controller", "mycobot280m5_gripper_controller")] if gripper else []
        ):
            assert kinds[live]["type"] == "position_controllers/JointGroupPositionController"
            assert data[live]["ros__parameters"]["joints"] == data[trajectory]["ros__parameters"]["joints"]
        assert ("mycobot280m5_live_gripper_controller" in kinds) == gripper


def test_controller_yaml_is_accepted_by_ros_parameter_parser():
    import pytest
    pytest.importorskip("rclpy")
    import subprocess
    import sys
    for suffix in ("", "_gripper"):
        path = ROOT / f"mycobot_280/mycobot_280_gazebo/config/ros2_controllers{suffix}.yaml"
        result = subprocess.run([sys.executable, "-c", "import rclpy,sys; rclpy.init(args=['--ros-args','--params-file',sys.argv[1]]); rclpy.shutdown()", str(path)], capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
