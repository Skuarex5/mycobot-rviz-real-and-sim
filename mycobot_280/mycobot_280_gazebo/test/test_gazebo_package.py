"""Unit tests for mycobot_280_gazebo URDF, controller config, and launch safety."""

import os
import unittest
import xml.etree.ElementTree as ET


PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF_PATH = os.path.join(PKG_ROOT, "urdf", "mycobot_280_m5_gazebo.urdf.xacro")
CONTROLLERS_YAML_PATH = os.path.join(PKG_ROOT, "config", "ros2_controllers.yaml")
LAUNCH_PATH = os.path.join(PKG_ROOT, "launch", "gazebo.launch.py")
SETUP_PY_PATH = os.path.join(PKG_ROOT, "setup.py")

EXPECTED_REVOLUTE_JOINTS = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]


class TestMycobot280GazeboPackage(unittest.TestCase):
    """Validate URDF structure, ros2_control interfaces, and simulation-only constraints."""

    def test_urdf_structure_and_ros2_control(self) -> None:
        with open(URDF_PATH, "r", encoding="utf-8") as f:
            raw_xml = f.read()

        # Must not reference nonexistent 'mybot' package
        self.assertNotIn("$(find mybot)", raw_xml)
        self.assertIn("$(find mycobot_280_gazebo)/config/ros2_controllers.yaml", raw_xml)
        self.assertIn("$(find mycobot_description)/urdf/mycobot_280_m5/", raw_xml)

        root = ET.fromstring(raw_xml)
        self.assertEqual(root.tag, "robot")

        # Check joints in URDF
        urdf_joints = {j.attrib["name"]: j for j in root.findall("joint")}
        self.assertEqual(urdf_joints["g_base_to_joint1"].attrib["type"], "fixed")
        self.assertIsNone(urdf_joints["g_base_to_joint1"].find("limit"))

        for jname in EXPECTED_REVOLUTE_JOINTS:
            self.assertIn(jname, urdf_joints)
            j_elem = urdf_joints[jname]
            self.assertEqual(j_elem.attrib["type"], "revolute")
            limit = j_elem.find("limit")
            self.assertIsNotNone(limit)
            lower = float(limit.attrib["lower"])
            upper = float(limit.attrib["upper"])
            velocity = float(limit.attrib["velocity"])
            self.assertLess(lower, 0.0)
            self.assertGreater(upper, 0.0)
            self.assertGreater(velocity, 0.0)

        # Check ros2_control section
        rc = root.find("ros2_control")
        self.assertIsNotNone(rc)
        plugin = rc.find("./hardware/plugin")
        self.assertIsNotNone(plugin)
        self.assertEqual((plugin.text or "").strip(), "gazebo_ros2_control/GazeboSystem")

        rc_joints = [j.attrib["name"] for j in rc.findall("joint")]
        self.assertEqual(rc_joints, EXPECTED_REVOLUTE_JOINTS)
        self.assertNotIn("g_base_to_joint1", rc_joints)
        self.assertNotIn("world_to_g_base", rc_joints)

    def test_controllers_yaml(self) -> None:
        with open(CONTROLLERS_YAML_PATH, "r", encoding="utf-8") as f:
            text = f.read()
        self.assertIn("joint_trajectory_controller/JointTrajectoryController", text)
        self.assertIn("joint_state_broadcaster/JointStateBroadcaster", text)
        for jname in EXPECTED_REVOLUTE_JOINTS:
            self.assertIn(jname, text)
        self.assertNotIn("g_base_to_joint1", text)

    def test_launch_file_is_simulation_only(self) -> None:
        with open(LAUNCH_PATH, "r", encoding="utf-8") as f:
            src = f.read()
        self.assertNotIn("joint_state_publisher_gui", src)
        self.assertNotIn("pymycobot", src)
        self.assertNotIn("/dev/tty", src)
        self.assertIn("OnProcessExit", src)
        self.assertIn("joint_state_broadcaster", src)
        self.assertIn("mycobot280m5_group_controller", src)

    def test_setup_py_has_no_setup_cfg_side_effect(self) -> None:
        with open(SETUP_PY_PATH, "r", encoding="utf-8") as f:
            src = f.read()
        self.assertNotIn("setup.cfg", src)
        self.assertNotIn("open(", src)


if __name__ == "__main__":
    unittest.main()
