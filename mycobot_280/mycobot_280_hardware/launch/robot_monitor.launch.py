"""
Launch the READ-ONLY hardware monitor, robot_state_publisher and RViz.

Starts no Gazebo, no ros2_control, no joint_state_publisher(_gui), no
move_group. /joint_states is published only by robot_monitor.
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
    return re.sub(r'<!--(.*?)-->', '', xml_text, flags=re.DOTALL)


def _launch_setup(context):
    gripper = LaunchConfiguration("adaptive_gripper").perform(context).lower() == "true"

    share = get_package_share_directory('mycobot_280_hardware')
    gazebo_share = get_package_share_directory('mycobot_280_gazebo')

    urdf_xacro_path = os.path.join(
        gazebo_share, 'urdf', 'mycobot_280_m5_gazebo.urdf.xacro'
    )
    urdf_doc = xacro.process_file(urdf_xacro_path, mappings={"adaptive_gripper": str(gripper).lower()})
    robot_description = {
        'robot_description': _strip_xml_comments(urdf_doc.toxml())
    }

    params_file = os.path.join(share, 'config', 'robot_monitor.yaml')
    rviz_config = os.path.join(share, 'config', 'robot_monitor.rviz')

    set_lc_numeric = SetEnvironmentVariable(name='LC_NUMERIC', value='C')

    args = [
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='Serial port of the myCobot.'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('update_rate', default_value='10.0',
                              description='Polling/publish rate in Hz.'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
    ]

    monitor = Node(
        package='mycobot_280_hardware',
        executable='robot_monitor',
        name='robot_monitor',
        output='screen',
        parameters=[
            params_file,
            {
                'adaptive_gripper': gripper,
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'update_rate_hz': LaunchConfiguration('update_rate'),
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

    return LaunchDescription([set_lc_numeric] + args + [monitor, rsp, rviz])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("adaptive_gripper", default_value="false",
                              choices=["true", "false"],
                              description="Use the myCobot 280 M5 adaptive gripper model and controller."),
        OpaqueFunction(function=lambda context: _launch_setup(context).entities),
    ])
