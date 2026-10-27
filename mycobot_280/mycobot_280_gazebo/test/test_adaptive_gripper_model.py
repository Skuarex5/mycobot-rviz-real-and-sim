"""Model contract for the optional physical/simulated adaptive gripper."""

from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import xacro

ROOT = Path(__file__).resolve().parents[3]
MODEL = ROOT / "mycobot_280/mycobot_280_gazebo/urdf/mycobot_280_m5_gazebo.urdf.xacro"


def model(enabled):
    return ET.fromstring(
        xacro.process_file(
            str(MODEL), mappings={"adaptive_gripper": str(enabled).lower()}
        ).toxml()
    )


def test_default_model_remains_without_gripper():
    assert model(False).find("joint[@name='gripper_controller']") is None


def test_gripper_model_has_compatible_limits_and_mimic_in_simulator():
    robot = model(True)
    principal = robot.find("joint[@name='gripper_controller']")
    assert principal is not None, "optional gripper is missing"
    lo, hi = (float(principal.find("limit").get(k)) for k in ("lower", "upper"))
    assert (lo, hi) == (-0.5, 0.15)
    followers = [j for j in robot.findall("joint") if j.find("mimic") is not None]
    assert len(followers) == 5
    for j in [principal] + followers:
        assert float(j.find("limit").get("velocity")) > 0
        link = robot.find(f"link[@name='{j.find('child').get('link')}']")
        assert float(link.find("inertial/mass").get("value")) > 0
    for j in followers:
        mul = float(j.find("mimic").get("multiplier"))
        limits = j.find("limit")
        for q in (lo, 0, hi):
            assert float(limits.get("lower")) <= mul * q <= float(limits.get("upper"))
        hw = robot.find(f"ros2_control/joint[@name='{j.get('name')}']")
        assert hw.find("param[@name='mimic']").text == "gripper_controller"
    assert robot.find("link[@name='gripper_tool_frame']") is not None
    assert "ros2_controllers_gripper.yaml" in ET.tostring(robot, encoding="unicode")


def test_all_gripper_meshes_exist():
    robot = model(True)
    meshes = [
        m.get("filename")
        for m in robot.findall(".//mesh")
        if "adaptive_gripper/" in m.get("filename")
    ]
    assert len(meshes) == 14
    for mesh in meshes:
        assert (
            ROOT / "mycobot_description/urdf/adaptive_gripper" / mesh.split("/")[-1]
        ).is_file()


def test_collada_units_are_normalized_for_moveit_and_gazebo():
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    for mesh in (ROOT / "mycobot_description/urdf/adaptive_gripper").glob("*.dae"):
        asset = ET.parse(mesh).getroot()
        assert float(asset.find("c:asset/c:unit", ns).get("meter")) == 1.0, (
            "mm metadata is not consistently applied by all mesh loaders"
        )


def test_gazebo_filters_mechanical_contacts_without_disabling_fingertips():
    robot = model(True)
    masks = {}
    for g in robot.findall("gazebo"):
        link = g.get("reference", "")
        if link.startswith("gripper_"):
            mask = g.find("collision/surface/contact/collide_bitmask")
            assert mask is not None, "Gazebo still collides overlapping linkage parts"
            masks[link] = int(mask.text, 0)
            assert masks[link] & 0xFFFF, "External collisions must remain enabled"
            assert g.find("selfCollide").text == "true"
    assert len(masks) == 7
    for left, right in [
        ("gripper_left1", "gripper_left2"),
        ("gripper_right1", "gripper_right2"),
        ("gripper_left3", "gripper_right3"),
        ("gripper_left2", "gripper_left3"),
        ("gripper_right2", "gripper_right3"),
    ]:
        assert not masks[left] & masks[right], (left, right)
    assert masks["gripper_left1"] & masks["gripper_right1"]


def test_collision_masks_match_reviewed_srdf_graph_and_survive_sdf_conversion(tmp_path):
    import itertools
    import subprocess

    robot = model(True)
    masks = {
        g.get("reference"): int(
            g.find("collision/surface/contact/collide_bitmask").text, 0
        )
        for g in robot.findall("gazebo")
        if g.get("reference", "").startswith("gripper_")
    }
    srdf = ET.parse(
        ROOT / "mycobot_280/mycobot_280_moveit2/config/mycobot_280_m5_gripper.srdf"
    ).getroot()
    excluded = {
        frozenset((j.get("link1"), j.get("link2")))
        for j in srdf.findall("disable_collisions")
    }
    for a, b in itertools.combinations(masks, 2):
        assert bool(masks[a] & masks[b]) == (frozenset((a, b)) not in excluded)
    path = tmp_path / "gripper.urdf"
    path.write_text(ET.tostring(robot, encoding="unicode"))
    sdf = ET.fromstring(
        subprocess.check_output(["gz", "sdf", "-p", str(path)], text=True)
    )
    for link, mask in masks.items():
        collision = next(
            c
            for c in sdf.findall(".//collision")
            if c.get("name", "").startswith(link + "_collision")
            or ("__" + link + "_collision") in c.get("name", "")
        )
        assert int(collision.find("surface/contact/collide_bitmask").text, 0) == mask
