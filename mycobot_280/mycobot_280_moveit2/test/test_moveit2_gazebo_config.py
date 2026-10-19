"""Unit tests for Milestone B1: MoveIt 2 + Gazebo integration in mycobot_280_moveit2."""

import os
from typing import Dict, List, Tuple
import unittest
import xml.etree.ElementTree as ET
import yaml


MOVEIT_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_ROOT = os.path.dirname(os.path.dirname(MOVEIT_PKG_ROOT))
GAZEBO_PKG_ROOT = os.path.join(REPO_ROOT, "mycobot_280", "mycobot_280_gazebo")

GAZEBO_URDF_PATH = os.path.join(
    GAZEBO_PKG_ROOT, "urdf", "mycobot_280_m5_gazebo.urdf.xacro"
)
GAZEBO_CONTROLLERS_PATH = os.path.join(
    GAZEBO_PKG_ROOT, "config", "ros2_controllers.yaml"
)

SRDF_PATH = os.path.join(MOVEIT_PKG_ROOT, "config", "mycobot_280_m5.srdf")
MOVEIT_CONTROLLERS_GAZEBO_PATH = os.path.join(
    MOVEIT_PKG_ROOT, "config", "moveit_controllers_gazebo.yaml"
)
JOINT_LIMITS_PATH = os.path.join(MOVEIT_PKG_ROOT, "config", "joint_limits.yaml")
KINEMATICS_PATH = os.path.join(MOVEIT_PKG_ROOT, "config", "kinematics.yaml")
MOVEIT_RVIZ_PATH = os.path.join(MOVEIT_PKG_ROOT, "config", "moveit.rviz")
LAUNCH_PATH = os.path.join(MOVEIT_PKG_ROOT, "launch", "moveit_gazebo.launch.py")


def _trace_chain_joints(
    urdf_root: ET.Element, base_link: str, tip_link: str
) -> List[ET.Element]:
    """Walk parent<-child links from tip_link back to base_link and return joints in base->tip order."""
    child_to_joint: Dict[str, ET.Element] = {}
    for joint in urdf_root.findall("joint"):
        child_elem = joint.find("child")
        if child_elem is not None and "link" in child_elem.attrib:
            child_to_joint[child_elem.attrib["link"]] = joint

    chain_reversed: List[ET.Element] = []
    curr_link = tip_link
    visited = set()
    while curr_link != base_link:
        if curr_link in visited or curr_link not in child_to_joint:
            raise AssertionError(
                f"Broken kinematic chain between '{base_link}' and '{tip_link}' at '{curr_link}'."
            )
        visited.add(curr_link)
        joint = child_to_joint[curr_link]
        chain_reversed.append(joint)
        parent_elem = joint.find("parent")
        assert parent_elem is not None
        curr_link = parent_elem.attrib["link"]

    return list(reversed(chain_reversed))


class TestMoveIt2GazeboIntegration(unittest.TestCase):
    """Verify URDF/SRDF consistency, joint limits, KDL kinematics, controller mapping, and launcher safety."""

    def setUp(self) -> None:
        self.urdf_root = ET.parse(GAZEBO_URDF_PATH).getroot()
        self.srdf_root = ET.parse(SRDF_PATH).getroot()
        with open(GAZEBO_CONTROLLERS_PATH, "r", encoding="utf-8") as f:
            self.gazebo_controllers = yaml.safe_load(f)
        with open(MOVEIT_CONTROLLERS_GAZEBO_PATH, "r", encoding="utf-8") as f:
            self.moveit_controllers = yaml.safe_load(f)
        with open(JOINT_LIMITS_PATH, "r", encoding="utf-8") as f:
            self.joint_limits_cfg = yaml.safe_load(f)
        with open(KINEMATICS_PATH, "r", encoding="utf-8") as f:
            self.kinematics_cfg = yaml.safe_load(f)

    def test_robot_name_and_srdf_chain_consistency(self) -> None:
        # 1. Robot name matches between Gazebo URDF and MoveIt SRDF
        self.assertEqual(self.urdf_root.attrib["name"], "mycobot_280_m5")
        self.assertEqual(self.srdf_root.attrib["name"], self.urdf_root.attrib["name"])

        # 2. Planning group arm_group uses chain g_base -> joint6_flange
        groups = {g.attrib["name"]: g for g in self.srdf_root.findall("group")}
        self.assertIn("arm_group", groups)
        chain_elem = groups["arm_group"].find("chain")
        self.assertIsNotNone(chain_elem)
        self.assertEqual(chain_elem.attrib["base_link"], "g_base")
        self.assertEqual(chain_elem.attrib["tip_link"], "joint6_flange")

        # 3. Kinematic chain from g_base to joint6_flange contains exact 6 movable joints in order
        chain_joints = _trace_chain_joints(
            self.urdf_root, base_link="g_base", tip_link="joint6_flange"
        )
        movable_joints = [
            j.attrib["name"] for j in chain_joints if j.attrib["type"] != "fixed"
        ]
        gazebo_jtc_joints = self.gazebo_controllers["mycobot280m5_group_controller"][
            "ros__parameters"
        ]["joints"]
        self.assertEqual(len(movable_joints), 6)
        self.assertEqual(movable_joints, gazebo_jtc_joints)

    def test_collision_matrix_validity_and_distal_protection(self) -> None:
        urdf_links = {link.attrib["name"] for link in self.urdf_root.findall("link")}
        disabled_pairs: List[Tuple[str, str]] = []
        for dc in self.srdf_root.findall("disable_collisions"):
            l1 = dc.attrib["link1"]
            l2 = dc.attrib["link2"]
            self.assertIn(l1, urdf_links)
            self.assertIn(l2, urdf_links)
            disabled_pairs.append((l1, l2))

        # Adjacent pairs must have collision disabled
        self.assertIn(("g_base", "joint1"), disabled_pairs)
        self.assertIn(("joint6", "joint6_flange"), disabled_pairs)

        # Distal links (joint4, joint5, joint6, joint6_flange) must NOT have collisions disabled with g_base
        for distal in ("joint4", "joint5", "joint6", "joint6_flange"):
            self.assertNotIn(("g_base", distal), disabled_pairs)
            self.assertNotIn((distal, "g_base"), disabled_pairs)

    def test_moveit_controller_matches_gazebo_controller(self) -> None:
        mgr = self.moveit_controllers["moveit_simple_controller_manager"]
        self.assertEqual(mgr["controller_names"], ["mycobot280m5_group_controller"])

        ctrl = mgr["mycobot280m5_group_controller"]
        self.assertEqual(ctrl["type"], "FollowJointTrajectory")
        self.assertEqual(ctrl["action_ns"], "follow_joint_trajectory")
        self.assertTrue(ctrl["default"])

        gazebo_joints = self.gazebo_controllers["mycobot280m5_group_controller"][
            "ros__parameters"
        ]["joints"]
        self.assertEqual(ctrl["joints"], gazebo_joints)

    def test_joint_limits_consistent_with_gazebo_urdf_and_conservative(self) -> None:
        vel_scale = float(self.joint_limits_cfg["default_velocity_scaling_factor"])
        acc_scale = float(self.joint_limits_cfg["default_acceleration_scaling_factor"])
        self.assertGreater(vel_scale, 0.0)
        self.assertLessEqual(vel_scale, 0.25)
        self.assertGreater(acc_scale, 0.0)
        self.assertLessEqual(acc_scale, 0.25)

        urdf_joints = {
            j.attrib["name"]: j
            for j in self.urdf_root.findall("joint")
            if j.attrib["type"] == "revolute"
        }
        jl_map = self.joint_limits_cfg["joint_limits"]
        self.assertEqual(set(jl_map.keys()), set(urdf_joints.keys()))

        for jname, j_elem in urdf_joints.items():
            limit_elem = j_elem.find("limit")
            assert limit_elem is not None
            urdf_lower = float(limit_elem.attrib["lower"])
            urdf_upper = float(limit_elem.attrib["upper"])
            urdf_vel = float(limit_elem.attrib["velocity"])
            self.assertLess(urdf_lower, 0.0)
            self.assertGreater(urdf_upper, 0.0)

            cfg_j = jl_map[jname]
            # Position limits must come exclusively from the Gazebo URDF to avoid MoveIt 2 parameter type mismatch
            self.assertNotIn("min_position", cfg_j)
            self.assertNotIn("max_position", cfg_j)
            self.assertTrue(cfg_j["has_velocity_limits"])
            self.assertIsInstance(cfg_j["max_velocity"], float)
            self.assertGreater(float(cfg_j["max_velocity"]), 0.0)
            self.assertLessEqual(float(cfg_j["max_velocity"]), urdf_vel)
            self.assertTrue(cfg_j["has_acceleration_limits"])
            self.assertIsInstance(cfg_j["max_acceleration"], float)
            self.assertGreater(float(cfg_j["max_acceleration"]), 0.0)

    def test_kdl_kinematics_and_rviz_frame(self) -> None:
        self.assertIn("arm_group", self.kinematics_cfg)
        arm_kin = self.kinematics_cfg["arm_group"]
        self.assertEqual(
            arm_kin["kinematics_solver"], "kdl_kinematics_plugin/KDLKinematicsPlugin"
        )
        self.assertGreater(float(arm_kin["kinematics_solver_timeout"]), 0.0)

        with open(MOVEIT_RVIZ_PATH, "r", encoding="utf-8") as f:
            rviz_text = f.read()
        self.assertIn("moveit_rviz_plugin/MotionPlanning", rviz_text)
        self.assertIn("rviz_default_plugins/RobotModel", rviz_text)
        self.assertIn("Fixed Frame: g_base", rviz_text)
        self.assertNotIn("base_link", rviz_text)

    def test_moveit_gazebo_launch_isolation_and_sim_only_safety(self) -> None:
        with open(LAUNCH_PATH, "r", encoding="utf-8") as f:
            raw_src = f.read()

        # Strip comments and docstrings to inspect active Python statements
        code_lines = [
            line for line in raw_src.splitlines() if not line.strip().startswith("#")
        ]
        code_only = "\n".join(code_lines)

        # Must start move_group and rviz2 with use_sim_time, LC_NUMERIC=C, and Gazebo URDF + SRDF
        self.assertIn("moveit_ros_move_group", code_only)
        self.assertIn("rviz2", code_only)
        self.assertIn("use_sim_time", code_only)
        self.assertIn('"LC_NUMERIC"', code_only)
        self.assertIn("mycobot_280_m5_gazebo.urdf.xacro", code_only)
        self.assertIn("mycobot_280_m5.srdf", code_only)
        self.assertIn("moveit_controllers_gazebo.yaml", code_only)

        # Must NOT start Gazebo, duplicate robot_state_publisher, controller_manager, mock hardware, or hardware drivers
        self.assertNotIn('executable="robot_state_publisher"', code_only)
        self.assertNotIn('executable="ros2_control_node"', code_only)
        self.assertNotIn('executable="spawner"', code_only)
        self.assertNotIn("joint_state_publisher", code_only)
        self.assertNotIn("mock_components", code_only)
        self.assertNotIn("spawn_entity", code_only)
        self.assertNotIn("pymycobot", raw_src)
        self.assertNotIn("/dev/tty", raw_src)


if __name__ == "__main__":
    unittest.main()
