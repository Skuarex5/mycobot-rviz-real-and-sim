#!/usr/bin/env python3
"""Validate automatic plans against running Gazebo/MoveIt (no physical driver)."""
import copy
import json
import math
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64, String
from std_srvs.srv import SetBool
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate


def main():
    rclpy.init()
    node = rclpy.create_node('automatic_execution_integration')
    measured, markers, status = {}, [], ['']
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    heartbeat = node.create_publisher(Empty, '/mycobot_live/heartbeat', 1)
    node.create_timer(.1, lambda: heartbeat.publish(Empty()))
    node.create_subscription(JointState, '/joint_states', lambda m: measured.update(zip(m.name, m.position)), 10)
    node.create_subscription(String, '/mycobot_live/status', lambda m: status.__setitem__(0, m.data), qos)
    node.create_subscription(InteractiveMarkerUpdate, '/mycobot_live/marker/update', lambda m: markers.extend(m.markers), 10)
    feedback = node.create_publisher(InteractiveMarkerFeedback, '/mycobot_live/marker/feedback', 10)
    grip = node.create_publisher(Float64, '/mycobot_live/gripper_target', 1)
    enabled = node.create_client(SetBool, '/mycobot_live/set_enabled')
    fk = node.create_client(GetPositionFK, '/compute_fk')
    move = ActionClient(node, MoveGroup, '/move_action')
    names = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
             'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
    def until(predicate, timeout=30):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        assert predicate(), 'Timed out'
    def call(client, req):
        assert client.wait_for_service(timeout_sec=10)
        f = client.call_async(req)
        until(f.done)
        return f.result()
    until(lambda: all(n in measured for n in names), 20)
    spin(1)
    # Establish a bent, nonsingular pose with the ordinary public planner.
    initial = MoveGroup.Goal()
    initial.request.group_name = 'arm_group'
    initial.request.allowed_planning_time = 5.
    initial.request.num_planning_attempts = 3
    initial.request.max_velocity_scaling_factor = .3
    initial.request.max_acceleration_scaling_factor = .3
    initial.request.start_state.is_diff = True
    initial.planning_options.planning_scene_diff.is_diff = True
    c = Constraints()
    for name, value in zip(names, [-.4, .8, -1.2, .4, .8, 0.]):
        c.joint_constraints.append(JointConstraint(joint_name=name, position=value,
            tolerance_above=.001, tolerance_below=.001, weight=1.))
    initial.request.goal_constraints = [c]
    assert move.wait_for_server(timeout_sec=10)
    f = move.send_goal_async(initial); until(f.done)
    handle = f.result(); assert handle.accepted
    f = handle.get_result_async(); until(f.done, 90)
    assert f.result().result.error_code.val == 1, f.result()
    spin(.5)
    before = copy.deepcopy(measured)
    result = call(enabled, SetBool.Request(data=True))
    assert result.success, result.message
    until(lambda: bool(markers))
    spin(.5)
    assert max(abs(measured[n] - before[n]) for n in names) < .005
    # Manual Plan & Execute cannot preempt the active automatic session.
    f = move.send_goal_async(initial); until(f.done)
    assert f.result().accepted
    manual = f.result().get_result_async(); until(manual.done)
    assert manual.result().result.error_code.val != 1
    q = [measured[n] for n in names]; q[0] += .04
    req = GetPositionFK.Request()
    req.header.frame_id = 'g_base'
    req.fk_link_names = ['gripper_tool_frame']
    req.robot_state.joint_state.name = names + ['gripper_controller']
    req.robot_state.joint_state.position = q + [measured['gripper_controller']]
    result = call(fk, req); assert result.error_code.val == 1
    pose = result.pose_stamped[0].pose
    msg = InteractiveMarkerFeedback(header=copy.deepcopy(markers[-1].header),
        marker_name='tool_target', client_id='integration', control_name='move_0', pose=pose)
    msg.event_type = msg.MOUSE_DOWN; feedback.publish(msg); spin(.1)
    msg.event_type = msg.POSE_UPDATE; feedback.publish(msg); spin(.1)
    msg.event_type = msg.MOUSE_UP; feedback.publish(msg)
    until(lambda: abs(measured[names[0]] - q[0]) < .005, 40)
    arm_delta = measured[names[0]] - before[names[0]]
    grip.publish(Float64(data=-.15))
    until(lambda: abs(measured['gripper_controller'] + .15) < .01, 30)
    until(lambda: 'movimiento ejecutado' in status[0], 30)
    result = call(enabled, SetBool.Request(data=False)); assert result.success
    until(lambda: 'cancelando' not in status[0], 10)
    print(json.dumps({'automatic_arm_delta_rad': arm_delta,
                      'gripper_position': measured['gripper_controller'],
                      'activation_without_movement': True,
                      'manual_preemption_blocked': True,
                      'destination': 'Gazebo through ControlCoordinator',
                      'stop': status[0]}))
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
