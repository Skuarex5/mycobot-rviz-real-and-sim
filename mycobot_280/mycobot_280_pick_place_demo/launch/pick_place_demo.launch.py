"""Standalone launch file for the myCobot 280 M5 simulated pick-and-place sequence.

Assumes Gazebo Classic and ros2_control controllers are already running via:
    ros2 launch mycobot_280_gazebo gazebo.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare(package="mycobot_280_pick_place_demo").find(
        "mycobot_280_pick_place_demo"
    )
    default_config_path = os.path.join(pkg_share, "config", "pick_place_waypoints.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    config_file = LaunchConfiguration("config_file")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true.",
    )
    declare_config_file = DeclareLaunchArgument(
        "config_file",
        default_value=default_config_path,
        description="Absolute path to the pick-and-place waypoints YAML file.",
    )

    pick_place_node = Node(
        package="mycobot_280_pick_place_demo",
        executable="pick_place_demo_node",
        name="pick_place_demo_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "config_file": config_file,
                "simulation_only": True,
            }
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_config_file,
            pick_place_node,
        ]
    )
