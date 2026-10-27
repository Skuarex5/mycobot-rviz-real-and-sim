#!/usr/bin/env python3
"""Opt-in real robot regression: rejected marker, then three small live moves.

Requires a mounted arm, empty gripper, clear workspace and a running unified
session. The default check only submits an unreachable marker, which must be
rejected without moving. --moves additionally requests three small base turns.
Only ROS commands are used; the driver remains the sole USB owner.
"""


def main(report_path, moves):
    import copy
    import json
    import math
    import threading
    import time
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Empty, String
    from std_srvs.srv import SetBool
    from visualization_msgs.msg import InteractiveMarkerFeedback
    from visualization_msgs.srv import GetInteractiveMarkers
    from moveit_msgs.srv import GetPositionFK
    from mycobot_280_control_interfaces.msg import HardwareStatus
    from mycobot_280_hardware.robot_monitor_node import DEFAULT_JOINT_NAMES

    rclpy.init()
    node = rclpy.create_node("physical_following_regression")
    state, samples, statuses = {}, [], []
    hardware = [None]
    target = [None]
    report = {}
    names = list(DEFAULT_JOINT_NAMES)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(
        JointState,
        "/physical/joint_states",
        lambda m: (
            state.update(zip(m.name, m.position)),
            samples.append({"t": time.monotonic(), "q": dict(zip(m.name, m.position))}),
        ),
        10,
    )
    node.create_subscription(
        HardwareStatus,
        "/physical/control/status",
        lambda m: hardware.__setitem__(0, m),
        qos,
    )
    node.create_subscription(
        String, "/mycobot_live/status", lambda m: statuses.append(m.data), qos
    )
    node.create_subscription(
        PoseStamped, "/target_pose", lambda m: target.__setitem__(0, m.pose), 1
    )
    beat = node.create_publisher(Empty, "/mycobot_live/heartbeat", 1)
    timer = node.create_timer(0.1, lambda: beat.publish(Empty()))
    marker = node.create_publisher(
        InteractiveMarkerFeedback, "/mycobot_live/marker/feedback", 1
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def wait(future, timeout=10.0):
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError("ROS request timed out")
        return future.result()

    def service(path, value):
        client = node.create_client(SetBool, path)
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError("Missing service " + path)
        result = wait(client.call_async(SetBool.Request(data=value)))
        print(path, result, flush=True)
        if not result.success:
            raise RuntimeError(result.message)

    def marker_pose():
        client = node.create_client(
            GetInteractiveMarkers, "/mycobot_live/marker/get_interactive_markers"
        )
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError("Missing marker service")
        result = wait(client.call_async(GetInteractiveMarkers.Request()))
        return next(m.pose for m in result.markers if m.name == "tool_target")

    def drag(pose):
        end = time.monotonic() + 3
        while marker.get_subscription_count() == 0 and time.monotonic() < end:
            time.sleep(0.02)
        if not marker.get_subscription_count():
            raise RuntimeError("Marker subscriber unavailable")
        feedback = InteractiveMarkerFeedback()
        feedback.header.frame_id = "g_base"
        feedback.marker_name = "tool_target"
        feedback.client_id = "physical_following_regression"
        feedback.control_name = "rotate_2"
        feedback.pose = pose
        for event in [feedback.MOUSE_DOWN, feedback.POSE_UPDATE, feedback.MOUSE_UP]:
            feedback.event_type = event
            marker.publish(feedback)
            time.sleep(0.08)

    def healthy():
        hw = hardware[0]
        if hw is None or not hw.selected or not hw.live or hw.fault:
            raise RuntimeError(
                "Live session lost: " + (hw.message if hw else "no status")
            )

    try:
        end = time.monotonic() + 8
        while (len(state) < 7 or hardware[0] is None) and time.monotonic() < end:
            time.sleep(0.05)
        if len(state) < 7:
            raise RuntimeError("Missing complete physical state")
        service("/mycobot_live/set_enabled", False)
        service("/mycobot_control/select_physical", True)
        service("/mycobot_live/set_enabled", True)
        time.sleep(0.5)
        before = copy.deepcopy(marker_pose())
        invalid = copy.deepcopy(before)
        invalid.position.z += 10.0
        drag(invalid)
        time.sleep(0.4)
        restored = marker_pose()
        error = math.dist(
            (before.position.x, before.position.y, before.position.z),
            (restored.position.x, restored.position.y, restored.position.z),
        )
        report["rejected_marker_distance_from_start_m"] = error
        if target[0] is None:
            raise RuntimeError("No live target telemetry")
        p = target[0].position
        target_error = math.dist(
            (before.position.x, before.position.y, before.position.z), (p.x, p.y, p.z)
        )
        report["rejected_target_distance_from_start_m"] = target_error
        healthy()
        if error > 0.01 or target_error > 0.01:
            raise RuntimeError(
                "Rejected target left marker stranded away from the robot"
            )
        print("PASS UNREACHABLE TARGET RECOVERY", error, flush=True)
        report["moves"] = []
        initial = [state[n] for n in names]
        for offset in [0.06, -0.06, 0.06] if moves else []:
            healthy()
            start = [state[n] for n in names]
            client = node.create_client(GetPositionFK, "/compute_fk")
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError("Missing FK service")
            req = GetPositionFK.Request()
            req.header.frame_id = "g_base"
            req.fk_link_names = ["gripper_tool_frame"]
            req.robot_state.joint_state.name = names + ["gripper_controller"]
            req.robot_state.joint_state.position = start + [state["gripper_controller"]]
            req.robot_state.joint_state.position[0] += offset
            response = wait(client.call_async(req))
            if response.error_code.val != 1:
                raise RuntimeError("FK failed")
            drag(response.pose_stamped[0].pose)
            end = time.monotonic() + 4.0
            while time.monotonic() < end:
                healthy()
                if any(abs(state[n] - q) > 0.15 for n, q in zip(names, initial)):
                    raise RuntimeError("Movement exceeded bounded test envelope")
                time.sleep(0.05)
            change = state[names[0]] - start[0]
            report["moves"].append({"requested_rad": offset, "measured_rad": change})
            if change * offset <= 0 or abs(change) < 0.02:
                raise RuntimeError("Repeated live move did not follow the target")
            print("PASS REPEATED PHYSICAL MOVE", offset, change, flush=True)
            if len(report["moves"]) == 1:
                req.robot_state.joint_state.position = [state[n] for n in names] + [
                    state["gripper_controller"]
                ]
                current_pose = wait(client.call_async(req)).pose_stamped[0].pose
                invalid = copy.deepcopy(current_pose)
                invalid.position.z += 10.0
                drag(invalid)
                time.sleep(0.4)
                healthy()
                restored = marker_pose()
                recovery_error = math.dist(
                    (
                        current_pose.position.x,
                        current_pose.position.y,
                        current_pose.position.z,
                    ),
                    (restored.position.x, restored.position.y, restored.position.z),
                )
                report["recovery_after_first_move_m"] = recovery_error
                if recovery_error > 0.01:
                    raise RuntimeError(
                        "Marker failed to recover after the first live move"
                    )
                print("PASS REJECTION AFTER FIRST MOVE", recovery_error, flush=True)
        service("/mycobot_live/set_enabled", False)
        time.sleep(1.2)
        hw = hardware[0]
        report["selected_after_stop"] = bool(hw and hw.selected)
        if hw is None or not hw.selected or hw.live or hw.fault:
            raise RuntimeError("Stopping live mode lost the physical selection")
        service("/mycobot_live/set_enabled", True)
        time.sleep(0.5)
        healthy()
        report["reactivated_without_reselecting"] = True
        service("/mycobot_live/set_enabled", False)
    except Exception as exc:
        report["failure"] = str(exc)
    finally:
        for path, value in [
            ("/mycobot_live/set_enabled", False),
            ("/mycobot_control/select_physical", False),
        ]:
            try:
                service(path, value)
            except Exception as exc:
                report.setdefault("cleanup_errors", []).append(str(exc))
        report["statuses"] = list(dict.fromkeys(statuses))
        report["samples"] = samples
        with open(report_path, "w") as stream:
            json.dump(report, stream, indent=2)
        print(json.dumps({k: v for k, v in report.items() if k != "samples"}, indent=2))
        node.destroy_timer(timer)
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
    return 1 if report.get("failure") or report.get("cleanup_errors") else 0


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute-on-real-robot", action="store_true")
    parser.add_argument(
        "--moves", action="store_true", help="Also perform three small base turns"
    )
    parser.add_argument("--report", default="/tmp/physical-following-regression.json")
    args = parser.parse_args()
    if not args.execute_on_real_robot:
        parser.error("Requires --execute-on-real-robot and clear physical workspace")
    raise SystemExit(main(args.report, args.moves))
