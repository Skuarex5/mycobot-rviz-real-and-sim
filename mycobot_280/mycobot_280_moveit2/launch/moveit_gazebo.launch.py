"""Standalone MoveIt 2 + RViz launch file connecting to a running Gazebo simulation.

Assumes Gazebo Classic and ros2_control controllers are already running via:
    ros2 launch mycobot_280_gazebo gazebo.launch.py

Launches only move_group and rviz2 with simulation time and POSIX numeric locale enabled.
"""

import os
import re
from typing import Any, Dict

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
import yaml


def _strip_xml_comments(xml_text: str) -> str:
    """Remove XML comments so MoveIt and ros2_control parse robot_description cleanly."""
    return re.sub(r"<!--(.*?)-->", "", xml_text, flags=re.DOTALL)


def _load_yaml(file_path: str) -> Dict[str, Any]:
    with open(file_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data if isinstance(data, dict) else {}


def _load_text(file_path: str) -> str:
    with open(file_path, "r", encoding="utf-8") as f:
        return f.read()


def generate_launch_description() -> LaunchDescription:
    moveit_pkg_share = FindPackageShare(package="mycobot_280_moveit2").find(
        "mycobot_280_moveit2"
    )
    gazebo_pkg_share = FindPackageShare(package="mycobot_280_gazebo").find(
        "mycobot_280_gazebo"
    )

    urdf_xacro_path = os.path.join(
        gazebo_pkg_share, "urdf", "mycobot_280_m5_gazebo.urdf.xacro"
    )
    srdf_path = os.path.join(moveit_pkg_share, "config", "mycobot_280_m5.srdf")
    kinematics_yaml_path = os.path.join(moveit_pkg_share, "config", "kinematics.yaml")
    joint_limits_yaml_path = os.path.join(
        moveit_pkg_share, "config", "joint_limits.yaml"
    )
    moveit_controllers_yaml_path = os.path.join(
        moveit_pkg_share, "config", "moveit_controllers_gazebo.yaml"
    )
    default_rviz_config_path = os.path.join(moveit_pkg_share, "config", "moveit.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    # Force LC_NUMERIC=C so Qt/urdfdom inside rviz2 and move_group always parse '.' decimals
    set_lc_numeric = SetEnvironmentVariable(name="LC_NUMERIC", value="C")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true.",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz2 with the MoveIt MotionPlanning plugin if true.",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config_path,
        description="Absolute path to the RViz configuration file.",
    )

    # Parse the exact URDF used by the Gazebo simulation
    urdf_doc = xacro.process_file(urdf_xacro_path)
    robot_description = {
        "robot_description": _strip_xml_comments(urdf_doc.toxml())
    }
    robot_description_semantic = {
        "robot_description_semantic": _load_text(srdf_path)
    }
    robot_description_kinematics = {
        "robot_description_kinematics": _load_yaml(kinematics_yaml_path)
    }
    robot_description_planning = {
        "robot_description_planning": _load_yaml(joint_limits_yaml_path)
    }

    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        },
    }

    moveit_controllers_dict = _load_yaml(moveit_controllers_yaml_path)
    trajectory_execution_config = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.5,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
        "trajectory_execution.allowed_start_tolerance": 0.05,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        additional_env={"LC_NUMERIC": "C"},
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution_config,
            moveit_controllers_dict,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        additional_env={"LC_NUMERIC": "C"},
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            set_lc_numeric,
            declare_use_sim_time,
            declare_use_rviz,
            declare_rviz_config,
            move_group_node,
            rviz_node,
        ]
    )
