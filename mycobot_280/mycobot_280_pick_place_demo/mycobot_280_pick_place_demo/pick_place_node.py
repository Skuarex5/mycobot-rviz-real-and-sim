"""Orchestrator node for the simulation-only myCobot 280 M5 pick-and-place demonstration."""

import os
import sys
import time
from typing import Optional, Tuple

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from mycobot_280_pick_place_demo.models_and_validators import (
    PickPlaceConfig,
    ValidationError,
    compute_max_position_error,
    extract_ordered_joint_positions,
    load_and_validate_config,
    validate_transition_velocity,
)
from mycobot_280_pick_place_demo.motion_executor import (
    FollowJointTrajectoryExecutor,
    MotionExecutor,
)


class PickPlaceDemoNode(Node):
    """Executes the validated pick-and-place sequence once and exits cleanly."""

    def __init__(
        self,
        config_file_override: Optional[str] = None,
        executor_override: Optional[MotionExecutor] = None,
    ) -> None:
        super().__init__("pick_place_demo_node")

        default_config_path = self._resolve_default_config_path()
        self.declare_parameter("config_file", default_config_path)
        self.declare_parameter("simulation_only", True)

        config_path = config_file_override or str(
            self.get_parameter("config_file").value
        )
        sim_only_param = bool(self.get_parameter("simulation_only").value)
        if not sim_only_param:
            raise ValidationError(
                "Parameter 'simulation_only' was set to False. "
                "Milestone A is strictly restricted to simulation."
            )

        self.get_logger().info(f"Loading pick-and-place configuration from: {config_path}")
        self._config: PickPlaceConfig = load_and_validate_config(config_path)

        self._latest_joint_state: Optional[JointState] = None
        self._joint_state_sub = self.create_subscription(
            JointState,
            self._config.joint_states_topic,
            self._on_joint_state,
            10,
        )

        self._marker_pub = None
        if self._config.publish_rviz_markers:
            marker_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self._marker_pub = self.create_publisher(
                MarkerArray,
                self._config.marker_topic,
                marker_qos,
            )

        self._executor: MotionExecutor = executor_override or FollowJointTrajectoryExecutor(
            node=self,
            action_name=self._config.controller_action_name,
        )

    @staticmethod
    def _resolve_default_config_path() -> str:
        try:
            share_dir = get_package_share_directory("mycobot_280_pick_place_demo")
            return os.path.join(share_dir, "config", "pick_place_waypoints.yaml")
        except Exception:
            pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            return os.path.join(pkg_root, "config", "pick_place_waypoints.yaml")

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _wait_for_valid_joint_states(self, timeout_sec: float) -> Tuple[float, ...]:
        deadline = time.monotonic() + timeout_sec
        last_error: Optional[str] = None
        while rclpy.ok():
            if self._latest_joint_state is not None:
                try:
                    return extract_ordered_joint_positions(
                        msg_joint_names=self._latest_joint_state.name,
                        msg_positions=self._latest_joint_state.position,
                        expected_joint_names=self._config.joint_names,
                        joint_limits=self._config.joint_limits,
                    )
                except ValidationError as exc:
                    last_error = str(exc)

            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))

        detail = f" Last validation detail: {last_error}" if last_error else ""
        raise ValidationError(
            f"Timed out after {timeout_sec:.1f}s waiting for valid joint states on "
            f"'{self._config.joint_states_topic}'. Verify that Gazebo and "
            f"'joint_state_broadcaster' are running.{detail}"
        )

    def _publish_nominal_markers(self) -> None:
        if self._marker_pub is None:
            return

        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        zones = [
            (0, "NOMINAL_PICK", self._config.nominal_pick_xyz, (0.1, 0.8, 0.2, 0.85)),
            (1, "NOMINAL_PLACE", self._config.nominal_place_xyz, (0.1, 0.4, 0.9, 0.85)),
        ]
        for marker_id, label, xyz, rgba in zones:
            cylinder = Marker()
            cylinder.header.frame_id = self._config.marker_frame_id
            cylinder.header.stamp = stamp
            cylinder.ns = "pick_place_zones"
            cylinder.id = marker_id * 2
            cylinder.type = Marker.CYLINDER
            cylinder.action = Marker.ADD
            cylinder.pose.position.x = xyz[0]
            cylinder.pose.position.y = xyz[1]
            cylinder.pose.position.z = xyz[2]
            cylinder.pose.orientation.w = 1.0
            cylinder.scale.x = 0.04
            cylinder.scale.y = 0.04
            cylinder.scale.z = 0.01
            cylinder.color.r = rgba[0]
            cylinder.color.g = rgba[1]
            cylinder.color.b = rgba[2]
            cylinder.color.a = rgba[3]
            marker_array.markers.append(cylinder)

            text = Marker()
            text.header.frame_id = self._config.marker_frame_id
            text.header.stamp = stamp
            text.ns = "pick_place_labels"
            text.id = marker_id * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = xyz[0]
            text.pose.position.y = xyz[1]
            text.pose.position.z = xyz[2] + 0.035
            text.pose.orientation.w = 1.0
            text.scale.z = 0.02
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = label
            marker_array.markers.append(text)

        self._marker_pub.publish(marker_array)
        self.get_logger().info(
            f"Published nominal PICK/PLACE RViz markers on '{self._config.marker_topic}' "
            f"(frame '{self._config.marker_frame_id}')."
        )

    def _pause_with_spin(self, duration_sec: float) -> None:
        if duration_sec <= 0.0:
            return
        deadline = time.monotonic() + duration_sec
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))

    def run_sequence_once(self) -> bool:
        """Verify readiness and execute the configured 9-step sequence once."""
        self.get_logger().info(
            "Pre-flight check 1/2: Checking controller availability on "
            f"'{self._config.controller_action_name}' (timeout={self._config.controller_wait_timeout_sec:.1f}s)..."
        )
        if not self._executor.wait_for_ready(self._config.controller_wait_timeout_sec):
            self.get_logger().error(
                f"Controller action server '{self._config.controller_action_name}' is not "
                "available. Make sure Gazebo is running ('ros2 launch mycobot_280_gazebo "
                "gazebo.launch.py') before launching the pick-and-place demo. "
                "Exiting without sending any motion commands."
            )
            return False

        self.get_logger().info(
            f"Pre-flight check 2/2: Waiting for valid '/joint_states' on "
            f"'{self._config.joint_states_topic}' (timeout={self._config.joint_state_wait_timeout_sec:.1f}s)..."
        )
        try:
            current_positions = self._wait_for_valid_joint_states(
                self._config.joint_state_wait_timeout_sec
            )
        except ValidationError as exc:
            self.get_logger().error(f"Joint state readiness failed: {exc}")
            return False

        first_stage = self._config.sequence_order[0]
        first_wp = self._config.waypoints[first_stage]
        try:
            validate_transition_velocity(
                start_positions=current_positions,
                target_waypoint=first_wp,
                max_velocity_rad_s=self._config.max_joint_velocity_rad_s,
                context_label=f"initial_state -> {first_stage}",
            )
        except ValidationError as exc:
            self.get_logger().error(
                f"Initial transition safety check failed before starting sequence: {exc}"
            )
            return False

        self._publish_nominal_markers()

        total_steps = len(self._config.sequence_order)
        self.get_logger().info(
            f"All pre-flight checks passed. Starting {total_steps}-step pick-and-place sequence: "
            f"{' -> '.join(self._config.sequence_order)}"
        )

        for idx, stage_name in enumerate(self._config.sequence_order, start=1):
            wp = self._config.waypoints[stage_name]
            formatted_target = ", ".join(f"{p:.3f}" for p in wp.positions)
            self.get_logger().info(
                f"[Step {idx}/{total_steps}] Stage '{stage_name}': moving to "
                f"[{formatted_target}] rad in {wp.duration_sec:.2f}s..."
            )

            result = self._executor.execute_waypoint(
                waypoint=wp,
                joint_names=self._config.joint_names,
                goal_response_timeout_sec=self._config.goal_response_timeout_sec,
                execution_timeout_margin_sec=self._config.execution_timeout_margin_sec,
            )
            if not result.success:
                self.get_logger().error(
                    f"[Step {idx}/{total_steps}] Sequence aborted at stage '{stage_name}': "
                    f"{result.message}"
                )
                return False

            # Refresh and verify post-step joint state tolerance
            try:
                self._latest_joint_state = None
                post_positions = self._wait_for_valid_joint_states(
                    timeout_sec=min(2.0, self._config.joint_state_wait_timeout_sec)
                )
                max_err, worst_joint = compute_max_position_error(
                    current_positions=post_positions,
                    target_positions=wp.positions,
                )
                if max_err > self._config.position_tolerance_rad:
                    self.get_logger().error(
                        f"[Step {idx}/{total_steps}] Stage '{stage_name}' exceeded position "
                        f"tolerance on '{worst_joint}': error={max_err:.4f} rad > "
                        f"tolerance={self._config.position_tolerance_rad:.4f} rad. "
                        "Stopping sequence."
                    )
                    return False
                self.get_logger().info(
                    f"[Step {idx}/{total_steps}] Stage '{stage_name}' succeeded "
                    f"(max joint error={max_err:.4f} rad on '{worst_joint}')."
                )
            except ValidationError as exc:
                self.get_logger().error(
                    f"[Step {idx}/{total_steps}] Post-step state verification failed at "
                    f"stage '{stage_name}': {exc}"
                )
                return False

            if idx < total_steps:
                self._pause_with_spin(self._config.pause_between_steps_sec)

        self.get_logger().info(
            "Pick-and-place demonstration sequence completed successfully (HOME -> ... -> HOME)."
        )
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[PickPlaceDemoNode] = None
    exit_code = 1
    try:
        node = PickPlaceDemoNode()
        if node.run_sequence_once():
            exit_code = 0
    except ValidationError as exc:
        print(f"[ERROR] [pick_place_demo_node]: Configuration/validation error: {exc}", file=sys.stderr)
        exit_code = 2
    except KeyboardInterrupt:
        print("[INFO] [pick_place_demo_node]: Interrupted by user.", file=sys.stderr)
        exit_code = 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
