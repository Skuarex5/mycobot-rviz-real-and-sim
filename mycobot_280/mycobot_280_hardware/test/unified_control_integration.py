"""Run against isolated control.launch + fake_hardware; never against real USB."""

import copy
import json
import math
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64, Bool, String
from std_srvs.srv import SetBool, Empty as EmptyService
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import InteractiveMarkerFeedback
from mycobot_280_control_interfaces.msg import HardwareStatus
from mycobot_280_hardware.robot_monitor_node import DEFAULT_JOINT_NAMES


def wait(future, timeout=20.0):
    end = time.monotonic() + timeout
    while not future.done() and time.monotonic() < end:
        time.sleep(0.01)
    assert future.done(), "future timeout"
    return future.result()


def until(predicate, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.03)
    raise AssertionError("condition timeout")


class Probe(Node):
    def __init__(self):
        super().__init__("unified_integration_probe")
        self.states = {}
        self.selected = None
        self.hw = HardwareStatus()
        self.status = ""
        self.active = False
        self.subs = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name in ["simulation", "physical", "selected"]:
            topic = (
                "/joint_states" if name == "selected" else "/" + name + "/joint_states"
            )
            self.subs.append(
                self.create_subscription(
                    JointState,
                    topic,
                    lambda m, n=name: self.states.__setitem__(
                        n, dict(zip(m.name, m.position))
                    ),
                    10,
                )
            )
        self.subs += [
            self.create_subscription(
                HardwareStatus,
                "/physical/control/status",
                lambda m: setattr(self, "hw", m),
                qos,
            ),
            self.create_subscription(
                Bool,
                "/mycobot_control/physical",
                lambda m: setattr(self, "selected", m.data),
                qos,
            ),
            self.create_subscription(
                String,
                "/mycobot_control/status",
                lambda m: setattr(self, "status", m.data),
                qos,
            ),
            self.create_subscription(
                Bool,
                "/mycobot_live/enabled",
                lambda m: setattr(self, "active", m.data),
                qos,
            ),
        ]
        self.beat = self.create_publisher(Empty, "/mycobot_live/heartbeat", 1)
        self.timer = self.create_timer(0.1, lambda: self.beat.publish(Empty()))
        self.gripper = self.create_publisher(Float64, "/mycobot_live/gripper_target", 1)
        self.marker = self.create_publisher(
            InteractiveMarkerFeedback, "/mycobot_live/marker/feedback", 1
        )

    def service(self, name, flag):
        client = self.create_client(SetBool, name)
        assert client.wait_for_service(timeout_sec=5.0)
        return wait(client.call_async(SetBool.Request(data=flag)))

    def action(self, kind, name, request, accepted=True, aborted=False):
        client = ActionClient(self, kind, name)
        assert client.wait_for_server(timeout_sec=5.0)
        handle = wait(client.send_goal_async(request))
        assert handle.accepted == accepted, (
            name,
            "unexpected acceptance",
            handle.accepted,
            self.status,
        )
        if not accepted:
            return None
        result = wait(handle.get_result_async(), 35.0)
        assert result.status == (6 if aborted else 4), (name, result)
        return result.result

    def cancel_action(self, kind, name, request):
        client = ActionClient(self, kind, name)
        assert client.wait_for_server(timeout_sec=5.0)
        handle = wait(client.send_goal_async(request))
        assert handle.accepted
        time.sleep(0.15)
        cancel = wait(handle.cancel_goal_async())
        assert cancel.goals_canceling
        assert wait(handle.get_result_async(), 8.0).status == 5

    def simulation_live(self):
        assert self.service("/mycobot_live/set_enabled", True).success
        until(lambda: self.active)
        start = [self.states["simulation"][n] for n in DEFAULT_JOINT_NAMES]
        fk = self.create_client(GetPositionFK, "/compute_fk")
        assert fk.wait_for_service(timeout_sec=3.0)
        req = GetPositionFK.Request()
        req.header.frame_id = "g_base"
        req.fk_link_names = ["gripper_tool_frame"]
        req.robot_state.joint_state.name = list(DEFAULT_JOINT_NAMES) + [
            "gripper_controller"
        ]
        req.robot_state.joint_state.position = start + [
            self.states["simulation"]["gripper_controller"]
        ]
        req.robot_state.joint_state.position[0] += 0.04
        pose = wait(fk.call_async(req)).pose_stamped[0].pose
        feedback = InteractiveMarkerFeedback()
        feedback.header.frame_id = "g_base"
        feedback.marker_name = "tool_target"
        feedback.event_type = feedback.POSE_UPDATE
        feedback.pose = pose
        feedback.client_id = "integration_probe"
        feedback.control_name = "rotate_0"
        feedback.event_type = feedback.MOUSE_DOWN
        self.marker.publish(feedback)
        time.sleep(.05)
        feedback.event_type = feedback.POSE_UPDATE
        self.marker.publish(feedback)
        time.sleep(.05)
        feedback.event_type = feedback.MOUSE_UP
        self.marker.publish(feedback)
        self.gripper.publish(Float64(data=0.1))
        until(
            lambda: (
                max(
                    abs(self.states["simulation"][n] - q)
                    for n, q in zip(DEFAULT_JOINT_NAMES, start)
                )
                > 0.008
            ),
            12.0,
        )
        until(
            lambda: abs(self.states["simulation"]["gripper_controller"] - 0.1) < 0.01,
            8.0,
        )
        assert self.service("/mycobot_live/set_enabled", False).success
        until(lambda: not self.active)

    def plan(self, q, execute=False):
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm_group"
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 1.0
        goal.request.max_acceleration_scaling_factor = 1.0
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [
            Constraints(
                joint_constraints=[
                    JointConstraint(
                        joint_name=n,
                        position=float(v),
                        tolerance_above=0.001,
                        tolerance_below=0.001,
                        weight=1.0,
                    )
                    for n, v in zip(DEFAULT_JOINT_NAMES, q)
                ]
            )
        ]
        goal.planning_options.plan_only = not execute
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        result = self.action(MoveGroup, "/move_action", goal)
        assert result.error_code.val == 1, result.error_code
        return result.planned_trajectory


def main():
    rclpy.init()
    node = Probe()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        until(lambda: len(node.states) == 3 and node.hw.connected)
        assert node.service("/mycobot_control/select_physical", False).success
        assert node.service("/mycobot_live/set_enabled", False).success
        until(lambda: node.selected is False)
        before_real = copy.deepcopy(node.states["physical"])
        q = [-0.35, 0.8, -1.2, 0.4, 0.8, 0.0]
        prep = FollowJointTrajectory.Goal()
        prep.trajectory.joint_names = list(DEFAULT_JOINT_NAMES)
        point = JointTrajectoryPoint(positions=q)
        point.time_from_start.sec = 4
        prep.trajectory.points = [point]
        preparation = ActionClient(
            node,
            FollowJointTrajectory,
            "/mycobot280m5_group_controller/follow_joint_trajectory",
        )
        assert preparation.wait_for_server(timeout_sec=5.0)
        handle = wait(preparation.send_goal_async(prep))
        assert handle.accepted
        assert not node.service("/mycobot_control/select_physical", True).success
        result = wait(handle.get_result_async())
        assert result.status == 4 and result.result.error_code == 0
        cancel_goal = copy.deepcopy(prep)
        cancel_goal.trajectory.points[0].positions[0] += 0.1
        cancel_goal.trajectory.points[0].time_from_start.sec = 2
        node.cancel_action(
            FollowJointTrajectory,
            "/mycobot280m5_group_controller/follow_joint_trajectory",
            cancel_goal,
        )
        pause = node.create_client(EmptyService, "/pause_physics")
        resume = node.create_client(EmptyService, "/unpause_physics")
        assert pause.wait_for_service(timeout_sec=3.0) and resume.wait_for_service(
            timeout_sec=3.0
        )
        wait(pause.call_async(EmptyService.Request()))
        try:
            time.sleep(0.7)
            node.action(
                FollowJointTrajectory,
                "/mycobot280m5_group_controller/follow_joint_trajectory",
                prep,
                accepted=False,
            )
        finally:
            wait(resume.call_async(EmptyService.Request()))
        time.sleep(0.2)
        print(
            "PASS busy destination rejection and paused simulation freshness",
            flush=True,
        )
        assert node.states["physical"] == before_real
        node.plan([-0.33, 0.8, -1.2, 0.4, 0.8, 0.0], execute=True)
        simulation_plan = node.plan([-0.31, 0.8, -1.2, 0.4, 0.8, 0.0])
        simulation_result = node.action(
            ExecuteTrajectory,
            "/execute_trajectory",
            ExecuteTrajectory.Goal(trajectory=simulation_plan),
        )
        assert simulation_result.error_code.val == 1
        node.simulation_live()
        assert node.states["physical"] == before_real
        print(
            "PASS simulation Plan & Execute, separate execution, live arm/gripper",
            flush=True,
        )
        old_plan = node.plan([-0.30, 0.8, -1.2, 0.4, 0.8, 0.0])
        reply = node.service("/mycobot_control/select_physical", True)
        assert reply.success, reply.message
        until(
            lambda: (
                node.selected
                and abs(node.states["selected"]["gripper_controller"] - 0.15) < 0.001
            )
        )
        old = ExecuteTrajectory.Goal(trajectory=old_plan)
        node.action(ExecuteTrajectory, "/execute_trajectory", old, aborted=True)
        print(
            "PASS selection uses measured state; stale simulation plan rejected",
            flush=True,
        )
        sim_before = copy.deepcopy(node.states["simulation"])
        q = [-0.38, 0.8, -1.2, 0.4, 0.8, 0.0]
        node.plan(q, execute=True)
        until(
            lambda: abs(node.states["physical"][DEFAULT_JOINT_NAMES[0]] + 0.38) < 0.005
        )
        q[0] = -0.36
        trajectory = node.plan(q)
        result = node.action(
            ExecuteTrajectory,
            "/execute_trajectory",
            ExecuteTrajectory.Goal(trajectory=trajectory),
        )
        assert result.error_code.val == 1
        assert (
            abs(
                node.states["simulation"][DEFAULT_JOINT_NAMES[0]]
                - sim_before[DEFAULT_JOINT_NAMES[0]]
            )
            < 0.002
        )
        print(
            "PASS physical Plan & Execute and separate Plan then Execute; Gazebo holds",
            flush=True,
        )
        reply = node.service("/mycobot_live/set_enabled", True)
        assert reply.success, reply.message
        until(lambda: node.active and node.hw.live)
        assert not node.service("/mycobot_control/select_physical", False).success
        node.action(
            FollowJointTrajectory,
            "/mycobot280m5_group_controller/follow_joint_trajectory",
            prep,
            accepted=False,
        )
        node.action(
            FollowJointTrajectory,
            "/physical/mycobot280m5_group_controller/follow_joint_trajectory",
            prep,
            accepted=False,
        )
        start = [node.states["physical"][n] for n in DEFAULT_JOINT_NAMES]
        fk = node.create_client(GetPositionFK, "/compute_fk")
        assert fk.wait_for_service(timeout_sec=3.0)
        req = GetPositionFK.Request()
        req.header.frame_id = "g_base"
        req.fk_link_names = ["gripper_tool_frame"]
        req.robot_state.joint_state.name = list(DEFAULT_JOINT_NAMES) + [
            "gripper_controller"
        ]
        req.robot_state.joint_state.position = [
            node.states["physical"][n] for n in DEFAULT_JOINT_NAMES
        ] + [0.15]
        req.robot_state.joint_state.position[0] += 0.04
        target = wait(fk.call_async(req)).pose_stamped[0]
        feedback = InteractiveMarkerFeedback()
        feedback.header.frame_id = "g_base"
        feedback.marker_name = "tool_target"
        feedback.event_type = feedback.POSE_UPDATE
        feedback.pose = target.pose
        feedback.client_id = "integration_probe"
        feedback.control_name = "rotate_0"
        feedback.event_type = feedback.MOUSE_DOWN
        node.marker.publish(feedback)
        time.sleep(.05)
        feedback.event_type = feedback.POSE_UPDATE
        node.marker.publish(feedback)
        time.sleep(.05)
        feedback.event_type = feedback.MOUSE_UP
        node.marker.publish(feedback)
        node.gripper.publish(Float64(data=0.1))
        until(
            lambda: (
                max(
                    abs(node.states["physical"][n] - q)
                    for n, q in zip(DEFAULT_JOINT_NAMES, start)
                )
                > 0.008
            ),
            12.0,
        )
        until(
            lambda: abs(node.states["physical"]["gripper_controller"] - 0.1) < 0.015,
            12.0,
        )
        assert node.active, node.status
        time.sleep(1.0)
        req.robot_state.joint_state.position = [
            node.states["physical"][n] for n in DEFAULT_JOINT_NAMES
        ] + [node.states["physical"]["gripper_controller"]]
        actual = wait(fk.call_async(req)).pose_stamped[0].pose.position
        expected = target.pose.position
        tip_error = math.sqrt(
            sum(
                (getattr(actual, axis) - getattr(expected, axis)) ** 2
                for axis in ["x", "y", "z"]
            )
        )
        assert tip_error < 0.003, tip_error
        print(
            "PASS physical live arm/gripper, action exclusion and FK error="
            + str(tip_error),
            flush=True,
        )
        reply = node.service("/mycobot_live/set_enabled", False)
        assert reply.success, reply.message
        until(lambda: not node.active and not node.hw.live)
        cancel_grip = FollowJointTrajectory.Goal()
        cancel_grip.trajectory.joint_names = ["gripper_controller"]
        cancel_point = JointTrajectoryPoint(positions=[0.0])
        cancel_point.time_from_start.sec = 2
        cancel_grip.trajectory.points = [cancel_point]
        node.cancel_action(
            FollowJointTrajectory,
            "/mycobot280m5_gripper_controller/follow_joint_trajectory",
            cancel_grip,
        )
        print(
            "PASS simulation and physical gripper cancellation propagation", flush=True
        )
        grip_goal = FollowJointTrajectory.Goal()
        grip_goal.trajectory.joint_names = ["gripper_controller"]
        grip_point = JointTrajectoryPoint(positions=[0.05])
        grip_point.time_from_start.sec = 1
        grip_goal.trajectory.points = [grip_point]
        assert (
            node.action(
                FollowJointTrajectory,
                "/mycobot280m5_gripper_controller/follow_joint_trajectory",
                grip_goal,
            ).error_code
            == 0
        )
        print("PASS physical gripper trajectory after live mode stops", flush=True)
        node.timer.cancel()
        until(lambda: not node.hw.selected, 4.0)
        assert node.selected is True  # A fault never silently changes destination.
        print(
            "PASS heartbeat loss revokes physical session without fallback", flush=True
        )
        node.timer.reset()
        time.sleep(1.0)
        assert not node.hw.selected and not node.active
        print("PASS reconnecting heartbeat does not rearm hardware", flush=True)
        node.timer.reset()
        reply = node.service("/mycobot_control/select_physical", False)
        assert reply.success, reply.message
        until(lambda: node.selected is False)
        q = [node.states['simulation'][name] for name in DEFAULT_JOINT_NAMES]
        q[0] += .03
        trajectory = node.plan(q)
        result = node.action(
            ExecuteTrajectory, '/execute_trajectory',
            ExecuteTrajectory.Goal(trajectory=trajectory))
        assert result.error_code.val == 1
        q[0] += .03
        node.plan(q, execute=True)
        print('PASS return to Gazebo after revoked physical session: Plan, Execute, Plan & Execute', flush=True)
        node.timer.cancel()
        print(
            json.dumps({"physical": node.states["physical"], "status": node.status}),
            flush=True,
        )
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
