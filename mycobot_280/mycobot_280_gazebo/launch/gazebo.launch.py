"""Simulation-only Gazebo Classic launch file for the myCobot 280 M5."""

import os
import re
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
import xacro


def _strip_xml_comments(xml_text: str) -> str:
    """Remove XML comments so gazebo_ros2_control parses robot_description cleanly."""
    return re.sub(r"<!--(.*?)-->", "", xml_text, flags=re.DOTALL)


def _launch_setup(context):
    gripper = LaunchConfiguration("adaptive_gripper").perform(context).lower() == "true"

    pkg_share = FindPackageShare(package="mycobot_280_gazebo").find("mycobot_280_gazebo")
    default_model_path = os.path.join(pkg_share, "urdf", "mycobot_280_m5_gazebo.urdf.xacro")

    namespace = LaunchConfiguration("control_namespace", default="").perform(context).strip("/")
    manager = "/" + namespace + "/controller_manager" if namespace else "/controller_manager"
    controller_params = ""
    if namespace:
        with open(os.path.join(pkg_share, "config", "ros2_controllers_gripper.yaml" if gripper else "ros2_controllers.yaml")) as stream:
            config = yaml.safe_load(stream)
        config = {"/" + namespace + "/" + key: value for key, value in config.items()}
        with tempfile.NamedTemporaryFile(mode="w", suffix="-mycobot-controllers.yaml", delete=False) as stream:
            yaml.safe_dump(config, stream)
            controller_params = stream.name

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    verbose = LaunchConfiguration("verbose")
    entity_name = LaunchConfiguration("entity_name")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true.",
    )
    declare_gui = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start Gazebo GUI client (gzclient) if true.",
    )
    declare_verbose = DeclareLaunchArgument(
        "verbose",
        default_value="true",
        description="Run Gazebo with verbose output.",
    )
    declare_entity_name = DeclareLaunchArgument(
        "entity_name",
        default_value="mycobot_280_m5",
        description="Entity name spawned in Gazebo.",
    )

    doc = xacro.process_file(default_model_path, mappings={"adaptive_gripper": str(gripper).lower(), "control_namespace": namespace, "controller_params": controller_params})
    robot_description_xml = _strip_xml_comments(doc.toxml())

    gazebo_ros_share = FindPackageShare(package="gazebo_ros").find("gazebo_ros")
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "gui": gui,
            "verbose": verbose,
        }.items(),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        namespace=namespace,
        remappings=[("/tf", "/" + namespace + "/tf"), ("/tf_static", "/" + namespace + "/tf_static")] if namespace else [],
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description_xml},
            {"publish_frequency": 30.0},
        ],
    )

    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            entity_name,
            "-topic",
            "/" + namespace + "/robot_description" if namespace else "robot_description",
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            manager,
        ],
        output="screen",
    )

    arm_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mycobot280m5_group_controller",
            "--controller-manager",
            manager,
        ],
        output="screen",
    )

    gripper_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["mycobot280m5_gripper_controller", "--controller-manager", manager],
        output="screen", condition=IfCondition(LaunchConfiguration("adaptive_gripper")),
    )
    live_arm_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["mycobot280m5_live_arm_controller", "--inactive", "--controller-manager", manager],
        output="screen",
    )
    live_gripper_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["mycobot280m5_live_gripper_controller", "--inactive", "--controller-manager", manager],
        output="screen", condition=IfCondition(LaunchConfiguration("adaptive_gripper")),
    )
    start_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    start_jtc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_trajectory_controller_spawner, gripper_spawner, live_arm_spawner, live_gripper_spawner],
        )
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_gui,
            declare_verbose,
            declare_entity_name,
            gazebo_launch,
            robot_state_publisher_node,
            spawn_entity_node,
            start_jsb_after_spawn,
            start_jtc_after_jsb,
        ]
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("control_namespace", default_value=""),
        DeclareLaunchArgument("adaptive_gripper", default_value="false",
                              choices=["true", "false"],
                              description="Use the myCobot 280 M5 adaptive gripper model and controller."),
        OpaqueFunction(function=lambda context: _launch_setup(context).entities),
    ])
