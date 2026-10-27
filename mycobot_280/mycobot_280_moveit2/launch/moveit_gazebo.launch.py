"""Standalone MoveIt 2 + RViz launch file connecting to a running Gazebo simulation.

Assumes Gazebo Classic and ros2_control controllers are already running via:
    ros2 launch mycobot_280_gazebo gazebo.launch.py

Launches move_group, live following and rviz2 with simulation time and POSIX numeric locale enabled.
"""

import os
import re
from typing import Any, Dict

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
import yaml
from ament_index_python.packages import get_package_prefix


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


def _launch_setup(context):
    integrated = LaunchConfiguration("integrated_control", default="false").perform(context).lower() == "true"
    gripper = LaunchConfiguration("adaptive_gripper").perform(context).lower() == "true"

    moveit_pkg_share = FindPackageShare(package="mycobot_280_moveit2").find(
        "mycobot_280_moveit2"
    )
    gazebo_pkg_share = FindPackageShare(package="mycobot_280_gazebo").find(
        "mycobot_280_gazebo"
    )

    urdf_xacro_path = os.path.join(
        gazebo_pkg_share, "urdf", "mycobot_280_m5_gazebo.urdf.xacro"
    )
    srdf_path = os.path.join(moveit_pkg_share, "config", "mycobot_280_m5_gripper.srdf" if gripper else "mycobot_280_m5.srdf")
    kinematics_yaml_path = os.path.join(moveit_pkg_share, "config", "kinematics.yaml")
    joint_limits_yaml_path = os.path.join(
        moveit_pkg_share, "config", "joint_limits_gripper.yaml" if gripper else "joint_limits.yaml"
    )
    moveit_controllers_yaml_path = os.path.join(
        moveit_pkg_share, "config", "moveit_controllers_gripper.yaml" if gripper else "moveit_controllers_gazebo.yaml"
    )
    default_rviz_config_path = os.path.join(moveit_pkg_share, "config", "moveit_live.rviz")

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
    urdf_doc = xacro.process_file(urdf_xacro_path, mappings={"adaptive_gripper": str(gripper).lower()})
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

    # In integrated mode the coordinator scales physical planning requests.
    # Applying physical caps here would also slow every Gazebo trajectory.

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
    if integrated:
        manager = moveit_controllers_dict['moveit_simple_controller_manager']
        names = manager['controller_names']
        manager['controller_names'] = ['/' + name for name in names]
        for name in names:
            manager['/' + name] = manager.pop(name)

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
        namespace="internal" if integrated else "",
        remappings=[(name, "/" + name) for name in (
            "joint_states", "monitored_planning_scene", "planning_scene", "display_planned_path",
            "get_planning_scene", "apply_planning_scene", "compute_fk", "compute_ik", "check_state_validity",
            "plan_kinematic_path", "compute_cartesian_path", "query_planner_interface",
            "get_planner_params", "set_planner_params", "clear_octomap", "load_map", "save_map"
        )] if integrated else [],
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

    live_node = Node(
        package="mycobot_280_live_control", executable="live_follow_node",
        output="screen", parameters=[robot_description, robot_description_semantic,
            robot_description_kinematics, robot_description_planning,
            {"use_sim_time": use_sim_time, "adaptive_gripper": gripper,
             "planning_action": "/mycobot_live/move_action" if integrated else "/move_action"}],
    )

    # Humble can unload MoveIt's libraries before rclcpp destroys callbacks
    # registered by them (rviz#1858). Keep these two libraries referenced until
    # process exit; scope the workaround to RViz, preserving caller preloads.
    moveit_lib = os.path.join(get_package_prefix("moveit_ros_visualization"), "lib")
    rviz_preload = ":".join(filter(None, [
        os.path.join(moveit_lib, "libmoveit_motion_planning_rviz_plugin.so"),
        os.path.join(get_package_prefix("moveit_core"), "lib", "libmoveit_utils.so"),
        os.environ.get("LD_PRELOAD", ""),
    ]))
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        additional_env={"LC_NUMERIC": "C", "LD_PRELOAD": rviz_preload},
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
            live_node,
            rviz_node,
        ]
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("integrated_control", default_value="false"),
        DeclareLaunchArgument("adaptive_gripper", default_value="false",
                              choices=["true", "false"],
                              description="Use the myCobot 280 M5 adaptive gripper model and controller."),
        OpaqueFunction(function=lambda context: _launch_setup(context).entities),
    ])
