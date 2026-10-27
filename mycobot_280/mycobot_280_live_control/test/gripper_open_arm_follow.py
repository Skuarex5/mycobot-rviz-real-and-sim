#!/usr/bin/env python3
"""Regression: arm follows a single live goal when activated with the gripper fully open.

Requires running Gazebo/MoveIt with the gripper and a bent, collision-free arm.
The RViz panel supplies the normal heartbeat. Requests a tool pose from a 0.04 rad joint-1 offset; IK may distribute motion among joints.
"""
import time
import math
import rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from std_msgs.msg import String
from visualization_msgs.msg import InteractiveMarkerFeedback
from moveit_msgs.srv import GetPositionFK


def main():
    rclpy.init()
    node = rclpy.create_node('gripper_open_arm_follow')
    measured, status = {}, []
    node.create_subscription(JointState, '/joint_states', lambda m: measured.update(zip(m.name, m.position)), 10)
    node.create_subscription(String, '/mycobot_live/status', lambda m: status.append(m.data), 10)
    feedback = node.create_publisher(InteractiveMarkerFeedback, '/mycobot_live/marker/feedback', 1)
    mode = node.create_client(SetBool, '/mycobot_live/set_enabled')
    fk = node.create_client(GetPositionFK, '/compute_fk')

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)

    def call(client, request):
        assert client.wait_for_service(timeout_sec=8)
        future = client.call_async(request)
        end = time.monotonic() + 10
        while not future.done() and time.monotonic() < end:
            spin(.02)
        assert future.done(), 'Service timeout'
        return future.result()

    end = time.monotonic() + 10
    while 'gripper_controller' not in measured and time.monotonic() < end:
        spin(.1)
    assert abs(measured['gripper_controller'] - .15) < .001, 'Open gripper before running this test'
    assert call(mode, SetBool.Request(data=False)).success
    assert call(mode, SetBool.Request(data=True)).success
    spin(.5)
    names = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
    before = {name: measured[name] for name in names}
    request = GetPositionFK.Request()
    request.header.frame_id = 'g_base'
    request.fk_link_names = ['gripper_tool_frame']
    request.robot_state.joint_state.name = names + ['gripper_controller']
    request.robot_state.joint_state.position = [measured[n] for n in names] + [.15]
    request.robot_state.joint_state.position[0] += .04
    response = call(fk, request)
    assert response.error_code.val == 1
    message = InteractiveMarkerFeedback()
    message.header.frame_id = 'g_base'
    message.client_id = 'gripper_open_arm_follow'
    message.marker_name = 'tool_target'
    message.event_type = InteractiveMarkerFeedback.POSE_UPDATE
    message.pose = response.pose_stamped[0].pose
    end = time.monotonic() + 5
    while feedback.get_subscription_count() == 0 and time.monotonic() < end:
        spin(.1)
    assert feedback.get_subscription_count()
    feedback.publish(message)
    spin(5)
    delta = max(abs(measured[name] - before[name]) for name in names)
    request.robot_state.joint_state.position = [measured[name] for name in names] + [.15]
    actual = call(fk, request).pose_stamped[0].pose
    target = message.pose
    error = math.dist([actual.position.x, actual.position.y, actual.position.z], [target.position.x, target.position.y, target.position.z])
    print({'arm_delta_rad': delta, 'tool_position_error_m': error, 'open_gripper_rad': measured['gripper_controller'], 'status': status[-1] if status else '?'}, flush=True)
    assert call(mode, SetBool.Request(data=False)).success
    node.destroy_node()
    rclpy.shutdown()
    assert delta > .01, 'Fully-open gripper blocked the arm live command'
    assert error < .002, 'Tool did not approach the live target'


if __name__ == '__main__':
    main()
