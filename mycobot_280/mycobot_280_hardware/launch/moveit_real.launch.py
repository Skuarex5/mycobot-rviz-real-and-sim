"""
MoveIt 2 + RViz Plan & Execute launch file for the physical myCobot 280 M5.

Connects ``move_group`` to ``/mycobot280m5_group_controller/follow_joint_trajectory``
served by ``mycobot_280_hardware/robot_control`` with ``use_sim_time=False``.

SAFETY:
- ``allow_motion`` defaults to ``false``. MoveIt 2 can plan from the measured
  robot state immediately, but execution is rejected until launched with
  ``allow_motion:=true``.
- Does not start Gazebo, ros2_control, or any simulation publishers.
"""

import os
import re
from typing import Any, Dict

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def _strip_xml_comments(xml_text: str) -> str:
    return re.sub(r'<!--(.*?)-->', '', xml_text, flags=re.DOTALL)


def _load_yaml(file_path: str) -> Dict[str, Any]:
    with open(file_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    return data if isinstance(data, dict) else {}


def _load_text(file_path: str) -> str:
    with open(file_path, 'r', encoding='utf-8') as f:
        return f.read()


def generate_launch_description() -> LaunchDescription:
    hw_share = get_package_share_directory('mycobot_280_hardware')
    moveit_share = get_package_share_directory('mycobot_280_moveit2')
    gazebo_share = get_package_share_directory('mycobot_280_gazebo')

    urdf_xacro_path = os.path.join(
        gazebo_share, 'urdf', 'mycobot_280_m5_gazebo.urdf.xacro'
    )
    srdf_path = os.path.join(moveit_share, 'config', 'mycobot_280_m5.srdf')
    kinematics_yaml_path = os.path.join(moveit_share, 'config', 'kinematics.yaml')
    joint_limits_yaml_path = os.path.join(moveit_share, 'config', 'joint_limits.yaml')
    moveit_controllers_yaml_path = os.path.join(
        moveit_share, 'config', 'moveit_controllers_gazebo.yaml'
    )
    default_rviz_config_path = os.path.join(moveit_share, 'config', 'moveit.rviz')
    robot_control_launch_path = os.path.join(
        hw_share, 'launch', 'robot_control.launch.py'
    )

    set_lc_numeric = SetEnvironmentVariable(name='LC_NUMERIC', value='C')

    args = [
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Serial port of the physical myCobot 280 M5.',
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Serial baudrate for pymycobot.',
        ),
        DeclareLaunchArgument(
            'update_rate',
            default_value='10.0',
            description='Joint state polling and publish rate in Hz.',
        ),
        DeclareLaunchArgument(
            'allow_motion',
            default_value='false',
            description=(
                'Explicit opt-in to enable physical trajectory execution. '
                'Defaults to false (plan-only / dry-run safety).'
            ),
        ),
        DeclareLaunchArgument(
            'max_speed',
            default_value='30',
            description='Conservative ceiling for pymycobot speed (1..100).',
        ),
        DeclareLaunchArgument(
            'launch_driver',
            default_value='true',
            description=(
                'Launch robot_control.launch.py automatically. Set to false '
                'if robot_control.launch.py is already running in another terminal.'
            ),
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 with MoveIt MotionPlanning display.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config_path,
            description='Path to the RViz configuration file.',
        ),
    ]

    urdf_doc = xacro.process_file(urdf_xacro_path)
    robot_description = {
        'robot_description': _strip_xml_comments(urdf_doc.toxml())
    }
    robot_description_semantic = {
        'robot_description_semantic': _load_text(srdf_path)
    }
    robot_description_kinematics = {
        'robot_description_kinematics': _load_yaml(kinematics_yaml_path)
    }
    robot_description_planning = {
        'robot_description_planning': _load_yaml(joint_limits_yaml_path)
    }

    ompl_planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ),
            'start_state_max_bounds_error': 0.1,
        },
    }

    moveit_controllers_dict = _load_yaml(moveit_controllers_yaml_path)
    trajectory_execution_config = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 2.0,
        'trajectory_execution.allowed_goal_duration_margin': 2.0,
        'trajectory_execution.allowed_start_tolerance': 0.15,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Isolate the included launch in a scoped GroupAction so its internal
    # 'use_rviz:=false' argument never clobbers the parent's LaunchConfiguration('use_rviz').
    driver_include = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_control_launch_path),
                launch_arguments={
                    'port': LaunchConfiguration('port'),
                    'baudrate': LaunchConfiguration('baudrate'),
                    'update_rate': LaunchConfiguration('update_rate'),
                    'allow_motion': LaunchConfiguration('allow_motion'),
                    'max_speed': LaunchConfiguration('max_speed'),
                    'use_rviz': 'false',
                }.items(),
            )
        ],
        scoped=True,
        condition=IfCondition(LaunchConfiguration('launch_driver')),
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        additional_env={'LC_NUMERIC': 'C'},
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution_config,
            moveit_controllers_dict,
            planning_scene_monitor_parameters,
            {'use_sim_time': False},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        additional_env={'LC_NUMERIC': 'C'},
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            {'use_sim_time': False},
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(
        [set_lc_numeric] + args + [driver_include, move_group_node, rviz_node]
    )
