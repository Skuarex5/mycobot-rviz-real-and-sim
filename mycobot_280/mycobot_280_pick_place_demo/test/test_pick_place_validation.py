"""Unit tests for mycobot_280_pick_place_demo validation, launch separation, and safety."""

import copy
import os
import sys
import unittest
import yaml


PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from mycobot_280_pick_place_demo.models_and_validators import (  # noqa: E402
    CANONICAL_JOINT_NAMES,
    REQUIRED_SEQUENCE,
    ValidationError,
    compute_max_position_error,
    extract_ordered_joint_positions,
    load_and_validate_config,
    validate_config_dict,
)


CONFIG_YAML_PATH = os.path.join(PKG_ROOT, "config", "pick_place_waypoints.yaml")
LAUNCH_PATH = os.path.join(PKG_ROOT, "launch", "pick_place_demo.launch.py")
SETUP_PY_PATH = os.path.join(PKG_ROOT, "setup.py")
NODE_PY_PATH = os.path.join(PKG_ROOT, "mycobot_280_pick_place_demo", "pick_place_node.py")


class TestPickPlaceDemoPackage(unittest.TestCase):
    """Verify configuration validation, safety limits, and strict launch file separation."""

    def setUp(self) -> None:
        with open(CONFIG_YAML_PATH, "r", encoding="utf-8") as f:
            self.raw_yaml = yaml.safe_load(f)

    def test_default_yaml_loads_and_validates(self) -> None:
        cfg = load_and_validate_config(CONFIG_YAML_PATH)
        self.assertTrue(cfg.simulation_only)
        self.assertEqual(cfg.joint_names, CANONICAL_JOINT_NAMES)
        self.assertEqual(cfg.sequence_order, REQUIRED_SEQUENCE)
        self.assertEqual(
            cfg.controller_action_name,
            "/mycobot280m5_group_controller/follow_joint_trajectory",
        )
        for stage in REQUIRED_SEQUENCE:
            self.assertIn(stage, cfg.waypoints)
            self.assertEqual(len(cfg.waypoints[stage].positions), 6)

    def test_rejects_simulation_only_false(self) -> None:
        mutated = copy.deepcopy(self.raw_yaml)
        mutated["pick_place_demo_node"]["ros__parameters"]["simulation_only"] = False
        with self.assertRaisesRegex(ValidationError, "simulation_only"):
            validate_config_dict(mutated)

    def test_rejects_wrong_joint_names_or_order(self) -> None:
        mutated = copy.deepcopy(self.raw_yaml)
        joints = list(CANONICAL_JOINT_NAMES)
        joints[0], joints[1] = joints[1], joints[0]
        mutated["pick_place_demo_node"]["ros__parameters"]["joint_names"] = joints
        with self.assertRaisesRegex(ValidationError, "joint_names"):
            validate_config_dict(mutated)

    def test_rejects_wrong_waypoint_dimension(self) -> None:
        mutated = copy.deepcopy(self.raw_yaml)
        mutated["pick_place_demo_node"]["ros__parameters"]["waypoints"]["PICK"][
            "positions"
        ] = [0.0, 0.0, 0.0, 0.0, 0.0]
        with self.assertRaisesRegex(ValidationError, "6 joint positions"):
            validate_config_dict(mutated)

    def test_rejects_out_of_bounds_joint_position(self) -> None:
        mutated = copy.deepcopy(self.raw_yaml)
        # joint2_to_joint1 limit is [-2.9321, 2.9321]
        mutated["pick_place_demo_node"]["ros__parameters"]["waypoints"]["PRE_PICK"][
            "positions"
        ][0] = 3.05
        with self.assertRaisesRegex(ValidationError, "outside limits"):
            validate_config_dict(mutated)

    def test_rejects_invalid_duration(self) -> None:
        mutated = copy.deepcopy(self.raw_yaml)
        mutated["pick_place_demo_node"]["ros__parameters"]["waypoints"]["LIFT"][
            "duration_sec"
        ] = 0.2
        with self.assertRaisesRegex(ValidationError, "duration_sec"):
            validate_config_dict(mutated)

    def test_rejects_excessive_transition_velocity(self) -> None:
        mutated = copy.deepcopy(self.raw_yaml)
        # Set a large jump across a 1.0s duration (> 1.5 rad/s max_joint_velocity_rad_s)
        mutated["pick_place_demo_node"]["ros__parameters"]["waypoints"]["PRE_PICK"][
            "positions"
        ][0] = 2.0
        mutated["pick_place_demo_node"]["ros__parameters"]["waypoints"]["PRE_PICK"][
            "duration_sec"
        ] = 1.0
        with self.assertRaisesRegex(ValidationError, "estimated velocity"):
            validate_config_dict(mutated)

    def test_rejects_altered_sequence_order(self) -> None:
        mutated = copy.deepcopy(self.raw_yaml)
        mutated["pick_place_demo_node"]["ros__parameters"]["sequence_order"] = [
            "HOME",
            "PICK",
            "HOME",
        ]
        with self.assertRaisesRegex(ValidationError, "sequence_order"):
            validate_config_dict(mutated)

    def test_joint_state_extraction_and_error_calculation(self) -> None:
        cfg = load_and_validate_config(CONFIG_YAML_PATH)
        reversed_names = list(reversed(CANONICAL_JOINT_NAMES))
        reversed_positions = [0.6, 0.5, 0.4, 0.3, 0.2, 0.1]

        ordered = extract_ordered_joint_positions(
            msg_joint_names=reversed_names,
            msg_positions=reversed_positions,
            expected_joint_names=cfg.joint_names,
            joint_limits=cfg.joint_limits,
        )
        self.assertEqual(ordered, (0.1, 0.2, 0.3, 0.4, 0.5, 0.6))

        max_err, worst_joint = compute_max_position_error(
            current_positions=ordered,
            target_positions=(0.1, 0.2, 0.3, 0.4, 0.5, 0.55),
        )
        self.assertAlmostEqual(max_err, 0.05, places=6)
        self.assertEqual(worst_joint, "joint6output_to_joint6")

    def test_launch_file_is_strictly_standalone(self) -> None:
        with open(LAUNCH_PATH, "r", encoding="utf-8") as f:
            src = f.read()
        # Strip comments and docstrings so we inspect actual launch code/arguments
        code_lines = [
            line for line in src.splitlines() if not line.strip().startswith("#")
        ]
        code_only = "\n".join(code_lines)

        self.assertNotIn("start_gazebo", src)
        self.assertNotIn("spawn_entity", code_only)
        self.assertNotIn("controller_manager", code_only)
        self.assertNotIn("IncludeLaunchDescription", code_only)
        self.assertNotIn("pymycobot", src)
        self.assertNotIn("/dev/tty", src)
        self.assertIn("pick_place_demo_node", code_only)

    def test_setup_py_and_node_have_no_hardware_or_setup_cfg_side_effects(self) -> None:
        with open(SETUP_PY_PATH, "r", encoding="utf-8") as f:
            setup_src = f.read()
        self.assertNotIn("setup.cfg", setup_src)
        self.assertNotIn("open(", setup_src)

        with open(NODE_PY_PATH, "r", encoding="utf-8") as f:
            node_src = f.read()
        self.assertNotIn("pymycobot", node_src)
        self.assertNotIn("/dev/tty", node_src)


if __name__ == "__main__":
    unittest.main()
