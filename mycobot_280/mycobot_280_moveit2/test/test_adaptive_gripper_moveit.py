from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_gripper_planning_and_execution_contract():
    path = ROOT / "config/mycobot_280_m5_gripper.srdf"
    assert path.is_file(), "gripper SRDF is missing"
    srdf = ET.parse(path).getroot()
    assert (
        srdf.find("group[@name='gripper_group']/joint").get("name")
        == "gripper_controller"
    )
    assert (
        srdf.find("group[@name='arm_group']/chain").get("tip_link")
        == "gripper_tool_frame"
    )
    for name, value in [("open", 0.15), ("closed", -0.5)]:
        assert (
            float(srdf.find(f"group_state[@name='{name}']/joint").get("value")) == value
        )
    controllers = yaml.safe_load(
        (ROOT / "config/moveit_controllers_gripper.yaml").read_text()
    )
    pin = controllers["moveit_simple_controller_manager"][
        "mycobot280m5_gripper_controller"
    ]
    assert pin["action_ns"] == "follow_joint_trajectory"
    assert pin["joints"] == ["gripper_controller"]
    assert not any(
        x.get("link1") == "g_base" and x.get("link2", "").startswith("gripper_")
        for x in srdf.findall("disable_collisions")
    )


def test_only_documented_internal_linkage_overlap_is_excluded():
    srdf = ET.parse(ROOT / "config/mycobot_280_m5_gripper.srdf").getroot()
    pairs = {
        frozenset((x.get("link1"), x.get("link2")))
        for x in srdf.findall("disable_collisions")
    }
    assert frozenset(("gripper_left3", "gripper_right3")) in pairs
    assert frozenset(("gripper_left1", "gripper_right1")) not in pairs
    assert frozenset(("g_base", "gripper_base")) not in pairs


def test_live_panel_is_only_in_simulation_configuration():
    live = yaml.safe_load((ROOT / "config/moveit_live.rviz").read_text())
    original = yaml.safe_load((ROOT / "config/moveit.rviz").read_text())
    name = "mycobot_280_live_control/LiveFollowPanel"
    assert name in [p["Class"] for p in live["Panels"]]
    assert name not in [p["Class"] for p in original["Panels"]]
    markers = [d for d in live["Visualization Manager"]["Displays"] if d["Class"] == "rviz_default_plugins/InteractiveMarkers"]
    assert markers[0]["Interactive Markers Namespace"] == "/mycobot_live/marker"


def test_automatic_execution_launch_uses_robot_variant_without_servo(monkeypatch):
    import pytest
    pytest.importorskip("launch_ros")
    import importlib.util
    from launch import LaunchContext
    spec = importlib.util.spec_from_file_location("live_sim_launch", ROOT / "launch/moveit_gazebo.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    captured = []
    original = module.Node
    def capture(**kwargs):
        captured.append(kwargs)
        return original(**kwargs)
    monkeypatch.setattr(module, "Node", capture)
    for value in ["false", "true"]:
        captured.clear()
        context = LaunchContext()
        context.launch_configurations["adaptive_gripper"] = value
        module._launch_setup(context)
        node = next(n for n in captured if n.get("executable") == "live_follow_node")
        params = {key: value for p in node["parameters"] for key, value in p.items()}
        assert params["adaptive_gripper"] == (value == "true")
        assert "robot_description" in params and "robot_description_semantic" in params
        assert "moveit_servo" not in params
