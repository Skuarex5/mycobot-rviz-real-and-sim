"""The optional integration must not change legacy simulation names."""

from pathlib import Path
import xml.etree.ElementTree as ET
import pytest

xacro = pytest.importorskip("xacro")
ROOT = Path(__file__).resolve().parents[3]
MODEL = (
    ROOT
    / "mycobot_280"
    / "mycobot_280_gazebo"
    / "urdf"
    / "mycobot_280_m5_gazebo.urdf.xacro"
)


def test_simulation_plugin_has_explicit_optional_namespace():
    tree = ET.fromstring(
        xacro.process_file(
            str(MODEL), mappings={"control_namespace": "simulation"}
        ).toxml()
    )
    plugin = tree.find("./gazebo/plugin[@name='gazebo_ros2_control']")
    assert plugin.findtext("./ros/namespace") == "/simulation"
    assert plugin.findtext("robot_param_node") == "/simulation/robot_state_publisher"


def test_legacy_plugin_parameter_node_remains_unchanged():
    tree = ET.fromstring(xacro.process_file(str(MODEL)).toxml())
    plugin = tree.find("./gazebo/plugin[@name='gazebo_ros2_control']")
    assert plugin.findtext("robot_param_node") == "robot_state_publisher"
