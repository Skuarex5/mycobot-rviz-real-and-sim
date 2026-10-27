"""
Launch the single-owner hardware controller (robot_control) and robot_state_publisher.

SAFETY:
- ``allow_motion`` defaults to ``false`` (dry-run / read-only mode). Physical
  movement is rejected until ``allow_motion:=true`` is explicitly passed.
- Does not start Gazebo, ros2_control, or duplicate /joint_states publishers.
- Uses real wall-clock time (``use_sim_time: False``).
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _strip_xml_comments(xml_text: str) -> str:
    """Remove XML comments so MoveIt and robot_state_publisher parse URDF cleanly."""
    return re.sub(r'<!--(.*?)-->', '', xml_text, flags=re.DOTALL)


def _launch_setup(context):
    gripper = LaunchConfiguration("adaptive_gripper").perform(context).lower() == "true"

    hw_share = get_package_share_directory('mycobot_280_hardware')
    gazebo_share = get_package_share_directory('mycobot_280_gazebo')

    urdf_xacro_path = os.path.join(
        gazebo_share, 'urdf', 'mycobot_280_m5_gazebo.urdf.xacro'
    )
    urdf_doc = xacro.process_file(urdf_xacro_path, mappings={"adaptive_gripper": str(gripper).lower()})
    robot_description = {
        'robot_description': _strip_xml_comments(urdf_doc.toxml())
    }

    params_file = os.path.join(hw_share, 'config', 'robot_control.yaml')
    rviz_config = os.path.join(hw_share, 'config', 'robot_monitor.rviz')

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
                'Explicit opt-in to allow physical robot motion. '
                'Must be true for FollowJointTrajectory goals to execute.'
            ),
        ),
        DeclareLaunchArgument(
            'max_speed',
            default_value='30',
            description='Conservative ceiling for pymycobot speed (1..100).',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start standalone RViz2 monitor view if true.',
        ),
    ]

    controller = Node(
        package='mycobot_280_hardware',
        executable='robot_control',
        name='robot_control',
        output='screen',
        parameters=[
            params_file,
            {
                'adaptive_gripper': gripper,
                'gripper_stop_confirmed': LaunchConfiguration('gripper_stop_confirmed').perform(context).lower() == 'true',
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'update_rate_hz': LaunchConfiguration('update_rate'),
                'allow_motion': LaunchConfiguration('allow_motion'),
                'max_pymycobot_speed': LaunchConfiguration('max_speed'),
                'use_sim_time': False,
            },
        ],
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        additional_env={'LC_NUMERIC': 'C'},
        parameters=[robot_description, {'use_sim_time': False}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        additional_env={'LC_NUMERIC': 'C'},
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='log',
    )

    return LaunchDescription([set_lc_numeric] + args + [controller, rsp, rviz])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gripper_stop_confirmed", default_value="false",
                              choices=["true", "false"],
                              description="Enable gripper execution only after verifying its firmware-specific stop on the real device."),
        DeclareLaunchArgument("adaptive_gripper", default_value="false",
                              choices=["true", "false"],
                              description="Use the myCobot 280 M5 adaptive gripper model and controller."),
        OpaqueFunction(function=lambda context: _launch_setup(context).entities),
    ])
