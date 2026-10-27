#!/usr/bin/env python3
"""Explicitly opt-in physical validation of the unified myCobot 280 controller.

Requires an empty gripper, a firmly mounted arm, clear reachable workspace,
and the running unified ROS session. This MOVES the real arm to ready, rotates
its base by small amounts and slightly opens the gripper. It ends live mode
and returns the destination to Gazebo, including on failure. It never opens
USB directly, initializes servos, or releases torque.
"""


def main(report_path):
    import json, threading, time
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.action import ActionClient
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Empty
    from std_srvs.srv import SetBool
    from mycobot_280_control_interfaces.msg import HardwareStatus, HardwareCommand
    from mycobot_280_control_interfaces.srv import SetHardwareMode

    NAMES = [
        "joint2_to_joint1",
        "joint3_to_joint2",
        "joint4_to_joint3",
        "joint5_to_joint4",
        "joint6_to_joint5",
        "joint6output_to_joint6",
    ]
    rclpy.init()
    n = Node("operator_authorized_physical_probe")
    state = {}
    hw = [None]
    samples = []
    report = {}
    handle = None
    n.create_subscription(
        JointState,
        "/physical/joint_states",
        lambda m: (
            state.update(dict(zip(m.name, m.position))),
            samples.append({"t": time.monotonic(), "q": list(m.position)}),
        ),
        10,
    )
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(
        HardwareStatus, "/physical/control/status", lambda m: hw.__setitem__(0, m), qos
    )
    beat = n.create_publisher(Empty, "/mycobot_live/heartbeat", 1)
    timer = n.create_timer(0.1, lambda: beat.publish(Empty()))
    e = MultiThreadedExecutor(num_threads=4)
    e.add_node(n)
    thread = threading.Thread(target=e.spin, daemon=True)
    thread.start()

    def wait(f, t=10):
        deadline = time.monotonic() + t
        while not f.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not f.done():
            raise RuntimeError("ROS request timeout")
        return f.result()

    def service(path, value):
        c = n.create_client(SetBool, path)
        if not c.wait_for_service(timeout_sec=5):
            raise RuntimeError("Missing service " + path)
        result = wait(c.call_async(SetBool.Request(data=value)))
        print(path, result, flush=True)
        if not result.success:
            raise RuntimeError(result.message)
        return result

    try:
        from moveit_msgs.action import MoveGroup, ExecuteTrajectory
        from moveit_msgs.msg import Constraints, JointConstraint
        from moveit_msgs.srv import GetPositionFK
        from visualization_msgs.msg import InteractiveMarkerFeedback

        def action(kind, path, goal, timeout=60):
            nonlocal handle
            client = ActionClient(n, kind, path)
            if not client.wait_for_server(timeout_sec=5):
                raise RuntimeError("Missing action " + path)
            handle = wait(client.send_goal_async(goal), 8)
            if not handle.accepted:
                raise RuntimeError("Rejected " + path)
            result = wait(handle.get_result_async(), timeout)
            if result.status != 4 or result.result.error_code.val != 1:
                raise RuntimeError(path + " failed " + str(result))
            return result.result

        def plan(q, execute=False):
            goal = MoveGroup.Goal()
            goal.request.group_name = "arm_group"
            goal.request.num_planning_attempts = 5
            goal.request.allowed_planning_time = 5.0
            goal.request.max_velocity_scaling_factor = 0.5
            goal.request.max_acceleration_scaling_factor = 0.5
            goal.request.start_state.is_diff = True
            goal.request.goal_constraints = [
                Constraints(
                    joint_constraints=[
                        JointConstraint(
                            joint_name=name,
                            position=float(value),
                            tolerance_above=0.002,
                            tolerance_below=0.002,
                            weight=1.0,
                        )
                        for name, value in zip(NAMES, q)
                    ]
                )
            ]
            goal.planning_options.plan_only = not execute
            goal.planning_options.planning_scene_diff.is_diff = True
            goal.planning_options.planning_scene_diff.robot_state.is_diff = True
            return action(MoveGroup, "/move_action", goal)

        deadline = time.monotonic() + 12
        while (hw[0] is None or len(state) < 7) and time.monotonic() < deadline:
            time.sleep(0.05)
        service("/mycobot_live/set_enabled", False)
        service("/mycobot_control/select_physical", True)
        before = [state[x] for x in NAMES]
        ready = [before[0], -0.4, -0.5, -0.2, 0.0, before[5]]
        report.update(before=before, ready_target=ready)
        planned = plan(ready)
        trajectory = planned.planned_trajectory
        for point in trajectory.joint_trajectory.points:
            for name, value in zip(
                trajectory.joint_trajectory.joint_names, point.positions
            ):
                index = NAMES.index(name)
                if (
                    abs(value - before[index]) > 0.65
                    or not min(before[index], ready[index]) - 0.08
                    <= value
                    <= max(before[index], ready[index]) + 0.08
                ):
                    raise RuntimeError("Planned path exceeds bounded ready envelope")
        report["ready_plan_points"] = len(trajectory.joint_trajectory.points)
        report["ready_plan_duration"] = (
            trajectory.joint_trajectory.points[-1].time_from_start.sec
            + trajectory.joint_trajectory.points[-1].time_from_start.nanosec * 1e-9
        )
        print(
            "REAL READY PLAN",
            report["ready_plan_points"],
            report["ready_plan_duration"],
            flush=True,
        )
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        action(ExecuteTrajectory, "/execute_trajectory", goal)
        report["ready_measured"] = [state[x] for x in NAMES]
        report["ready_max_error"] = max(
            abs(a - b) for a, b in zip(report["ready_measured"], ready)
        )
        if report["ready_max_error"] > 0.04:
            raise RuntimeError("Ready endpoint inaccurate")
        print("PASS REAL SEPARATE PLAN / EXECUTE", report["ready_measured"], flush=True)
        next_q = [state[x] for x in NAMES]
        next_q[0] += 0.05
        plan(next_q, execute=True)
        report["plan_execute_measured"] = [state[x] for x in NAMES]
        print("PASS REAL PLAN & EXECUTE", report["plan_execute_measured"], flush=True)
        commands = []
        n.create_subscription(
            HardwareCommand,
            "/mycobot_live/physical_command",
            lambda msg: commands.append(
                {
                    "t": time.monotonic(),
                    "desired": list(msg.positions[:6]),
                    "measured": [state[x] for x in NAMES],
                }
            ),
            1,
        )
        start = [state[x] for x in NAMES]
        fk = n.create_client(GetPositionFK, "/compute_fk")
        if not fk.wait_for_service(timeout_sec=3):
            raise RuntimeError("Missing FK")
        req = GetPositionFK.Request()
        req.header.frame_id = "g_base"
        req.fk_link_names = ["gripper_tool_frame"]
        req.robot_state.joint_state.name = NAMES + ["gripper_controller"]
        req.robot_state.joint_state.position = start + [state["gripper_controller"]]
        req.robot_state.joint_state.position[0] += 0.08
        pose = wait(fk.call_async(req)).pose_stamped[0].pose
        marker = n.create_publisher(
            InteractiveMarkerFeedback, "/mycobot_live/marker/feedback", 1
        )
        service("/mycobot_live/set_enabled", True)
        time.sleep(2.0)
        report["idle_live_start"] = start
        report["idle_live_end"] = [state[x] for x in NAMES]
        report["idle_max_change"] = max(
            abs(a - b) for a, b in zip(start, report["idle_live_end"])
        )
        if report["idle_max_change"] > 0.025:
            raise RuntimeError("Physical arm drifted without a marker command")
        print("PASS REAL LIVE IDLE", report["idle_max_change"], flush=True)
        feedback = InteractiveMarkerFeedback()
        feedback.header.frame_id = "g_base"
        feedback.marker_name = "tool_target"
        feedback.event_type = feedback.MOUSE_DOWN
        feedback.pose = pose
        feedback.client_id = "physical_validation"
        feedback.control_name = "rotate_2"
        feedback.header.stamp = n.get_clock().now().to_msg()
        marker.publish(feedback)
        time.sleep(.05)
        feedback.event_type = feedback.POSE_UPDATE
        marker.publish(feedback)
        time.sleep(.05)
        feedback.event_type = feedback.MOUSE_UP
        marker.publish(feedback)
        end = time.monotonic() + 6
        while time.monotonic() < end:
            if hw[0].fault or not hw[0].selected:
                raise RuntimeError("Live physical session failed: " + hw[0].message)
            if any(abs(state[x] - q) > 0.13 for x, q in zip(NAMES, start)):
                raise RuntimeError("Live test exceeded small motion envelope")
            time.sleep(0.05)
        report["live_start"] = start
        report["live_end"] = [state[x] for x in NAMES]
        report["live_max_change"] = max(
            abs(a - b) for a, b in zip(report["live_end"], start)
        )
        report["live_commands"] = commands
        report["live_max_command_lead"] = max(
            (
                max(abs(a - b) for a, b in zip(row["desired"], row["measured"]))
                for row in commands
            ),
            default=0,
        )
        if report["live_end"][0] - start[0] < 0.025:
            raise RuntimeError(
                "Base did not follow the live marker by at least 1.4 degrees"
            )
        print(
            "REAL LIVE CHANGE",
            report["live_max_change"],
            "MAX COMMAND LEAD",
            report["live_max_command_lead"],
            flush=True,
        )
        from std_msgs.msg import Float64

        gripper = n.create_publisher(Float64, "/mycobot_live/gripper_target", 1)
        grip_start = state["gripper_controller"]
        grip_target = min(0.10, grip_start + 0.06)
        end = time.monotonic() + 3
        while gripper.get_subscription_count() == 0 and time.monotonic() < end:
            time.sleep(0.02)
        if gripper.get_subscription_count() == 0:
            raise RuntimeError("Gripper target subscriber did not connect")
        gripper.publish(Float64(data=grip_target))
        end = time.monotonic() + 4
        while time.monotonic() < end:
            if hw[0].fault or not hw[0].selected:
                raise RuntimeError("Gripper live session lost: " + hw[0].message)
            if any(abs(state[x] - q) > 0.13 for x, q in zip(NAMES, start)):
                raise RuntimeError("Physical motion exceeded validation envelope")
            time.sleep(0.05)
        report["gripper_live_start"] = grip_start
        report["gripper_live_end"] = state["gripper_controller"]
        report["gripper_live_target"] = grip_target
        if state["gripper_controller"] - grip_start < 0.02:
            raise RuntimeError("Physical gripper did not follow live slider")
        print("PASS REAL LIVE GRIPPER", report["gripper_live_end"], flush=True)
        service("/mycobot_live/set_enabled", False)
    except Exception as exc:
        report["failure"] = str(exc)[:300]
        if "commands" in locals():
            report["live_commands"] = commands
    finally:
        try:
            service("/mycobot_live/set_enabled", False)
        except Exception:
            pass
        if handle is not None and handle.accepted:
            try:
                wait(handle.cancel_goal_async(), 2)
            except Exception:
                pass
        try:
            service("/mycobot_control/select_physical", False)
        except Exception as exc:
            report["deselect_error"] = str(exc)
            if hw[0] is not None and hw[0].selected:
                c = n.create_client(SetHardwareMode, "/physical/control/set_mode")
                if c.wait_for_service(timeout_sec=2):
                    result = wait(
                        c.call_async(
                            SetHardwareMode.Request(
                                selected=False, session_id=hw[0].session_id
                            )
                        ),
                        4,
                    )
                    report["forced_stop_confirmed"] = result.success
        report["samples"] = samples
        with open(report_path, "w") as stream:
            json.dump(report, stream, indent=2)
        print(
            json.dumps(
                {
                    k: v
                    for k, v in report.items()
                    if k not in ("samples", "live_commands")
                },
                indent=2,
            ),
            flush=True,
        )
        e.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        n.destroy_node()
        rclpy.shutdown()

    return 1 if "failure" in report else 0


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--execute-on-real-robot",
        action="store_true",
        help="Explicitly allow the bounded physical movements described above",
    )
    parser.add_argument("--report", default="/tmp/mycobot-physical-validation.json")
    args = parser.parse_args()
    if not args.execute_on_real_robot:
        parser.error("No movement sent: --execute-on-real-robot is required")
    raise SystemExit(main(args.report))
