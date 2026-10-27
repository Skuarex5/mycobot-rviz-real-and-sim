"""One RViz with an explicit, revocable Gazebo/physical destination selector."""

import os
import re
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        return value(name).lower() == "true"

    hardware = get_package_share_directory("mycobot_280_hardware")
    gazebo = get_package_share_directory("mycobot_280_gazebo")
    moveit = get_package_share_directory("mycobot_280_moveit2")
    gripper = flag("adaptive_gripper")
    model = xacro.process_file(
        os.path.join(gazebo, "urdf", "mycobot_280_m5_gazebo.urdf.xacro"),
        mappings={"adaptive_gripper": str(gripper).lower()},
    ).toxml()
    model = re.sub(r"<!--(.*?)-->", "", model, flags=re.DOTALL)
    with open(os.path.join(hardware, "config", "robot_control.yaml")) as stream:
        parameters = yaml.safe_load(stream)["robot_control"]["ros__parameters"]
    parameters.update(
        adaptive_gripper=gripper,
        require_session=True,
        gripper_stop_confirmed=flag("gripper_stop_confirmed"),
        port=value("port"),
        baudrate=int(value("baudrate")),
        allow_motion=flag("allow_motion"),
        max_pymycobot_speed=10,
        default_pymycobot_speed=5,
        gripper_speed=10,
        max_joint_velocity_rad_s=0.1,
        use_sim_time=False,
    )
    driver = Node(
        package="mycobot_280_hardware",
        executable="robot_control",
        name="robot_control",
        namespace="physical",
        parameters=[parameters],
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_driver")),
    )
    coordinator = Node(
        package="mycobot_280_hardware",
        executable="control_coordinator",
        parameters=[{"adaptive_gripper": gripper, "use_sim_time": False,
                     "physical_state_timeout_s": parameters["physical_state_timeout_s"]}],
        output="screen",
    )
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": model, "use_sim_time": False}],
        output="screen",
    )
    simulation = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo, "launch", "gazebo.launch.py")
                ),
                launch_arguments={
                    "adaptive_gripper": str(gripper).lower(),
                    "control_namespace": "simulation",
                    "gui": value("gui"),
                    "use_sim_time": "true",
                }.items(),
            )
        ],
        scoped=True,
    )
    planning = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(moveit, "launch", "moveit_gazebo.launch.py")
                ),
                launch_arguments={
                    "adaptive_gripper": str(gripper).lower(),
                    "integrated_control": "true",
                    "use_sim_time": "false",
                    "use_rviz": value("use_rviz"),
                }.items(),
            )
        ],
        scoped=True,
    )
    return [simulation, driver, coordinator, rsp, planning]


def generate_launch_description():
    defaults = {
        "adaptive_gripper": "true",
        "port": "/dev/ttyACM0",
        "baudrate": "115200",
        "allow_motion": "false",
        "gripper_stop_confirmed": "false",
        "launch_driver": "true",
        "gui": "true",
        "use_rviz": "true",
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument(name, default_value=default)
            for name, default in defaults.items()
        ]
        + [OpaqueFunction(function=setup)]
    )
