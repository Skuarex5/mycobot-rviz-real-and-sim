#!/usr/bin/env python3
"""Exercise the public live interface in an isolated running Gazebo/MoveIt session."""
import argparse
import json
import time
import math
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_srvs.srv import SetBool, Empty as EmptyService
from std_msgs.msg import Empty, Float64, Bool, String, Float64MultiArray
from sensor_msgs.msg import JointState
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate
from controller_manager_msgs.srv import ListControllers, SwitchController, LoadController, UnloadController, ConfigureController
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionFK, ApplyPlanningScene
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from copy import deepcopy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--adaptive-gripper', choices=['true', 'false'], default='true')
    args = parser.parse_args()
    gripper = args.adaptive_gripper == 'true'
    rclpy.init()
    node = rclpy.create_node('live_follow_integration')
    heartbeat = node.create_publisher(Empty, '/mycobot_live/heartbeat', 1)
    grip = node.create_publisher(Float64, '/mycobot_live/gripper_target', 1)
    feedback = node.create_publisher(InteractiveMarkerFeedback, '/mycobot_live/marker/feedback', 1)
    measured = {}
    markers = []
    enabled = [False]
    status = ['']
    command_times = []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(JointState, '/joint_states', lambda m: measured.update(zip(m.name, m.position)), 10)
    node.create_subscription(Bool, '/mycobot_live/enabled', lambda m: enabled.__setitem__(0, m.data), qos)
    node.create_subscription(String, '/mycobot_live/status', lambda m: status.__setitem__(0, m.data), qos)
    node.create_subscription(InteractiveMarkerUpdate, '/mycobot_live/marker/update', lambda m: markers.extend(m.markers), 100)
    node.create_subscription(Float64MultiArray, "/mycobot280m5_live_arm_controller/commands", lambda m: command_times.append(time.monotonic()), 10)
    service = node.create_client(SetBool, '/mycobot_live/set_enabled')
    fk = node.create_client(GetPositionFK, '/compute_fk')
    action = ActionClient(node, FollowJointTrajectory, '/mycobot280m5_group_controller/follow_joint_trajectory')
    controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
    live_names = ['mycobot280m5_live_arm_controller'] + (['mycobot280m5_live_gripper_controller'] if gripper else [])
    planned_names = ['mycobot280m5_group_controller'] + (['mycobot280m5_gripper_controller'] if gripper else [])
    arm_names = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
    def spin(seconds, beat=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if beat:
                heartbeat.publish(Empty())
            rclpy.spin_once(node, timeout_sec=.02)
    def call(client, req, timeout=12):
        future = client.call_async(req)
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            spin(.03)
        assert future.done(), 'Service timed out'
        return future.result()
    def mode(live):
        result = call(controllers, ListControllers.Request())
        states = {c.name: c.state for c in result.controller}
        assert all(states[n] == ('active' if live else 'inactive') for n in live_names), states
        assert all(states[n] == ('inactive' if live else 'active') for n in planned_names), states
        assert ('mycobot280m5_live_gripper_controller' in states) == gripper
    assert service.wait_for_service(timeout_sec=5), 'Live service missing'
    spin(1)
    assert all(n in measured for n in arm_names)
    mode(False)
    initial = {n: measured[n] for n in arm_names}
    response = call(service, SetBool.Request(data=True))
    assert response.success, response.message
    spin(1)
    assert enabled[0], status[0]
    mode(True)
    assert max(abs(measured[n] - initial[n]) for n in arm_names) < .01, 'Startup movement'
    assert markers, 'Missing interactive marker'
    base = markers[-1]
    # Validate the singular startup independently, then prepare a bent pose using
    # the existing trajectory action. Only the following setup uses a trajectory.
    assert call(service, SetBool.Request(data=False)).success
    seed = [-.4, .8, -1.2, .4, .8, 0.]
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = arm_names
    point = JointTrajectoryPoint()
    point.positions = seed
    point.time_from_start.sec = 4
    goal.trajectory.points = [point]
    assert action.wait_for_server(timeout_sec=3)
    accepted = action.send_goal_async(goal)
    while not accepted.done(): spin(.03)
    handle = accepted.result()
    assert handle.accepted
    spin(.2)
    # No live takeover while a planned trajectory is executing.
    busy_response = call(service, SetBool.Request(data=True))
    assert not busy_response.success, 'Live mode stole an executing trajectory'
    result = handle.get_result_async()
    while not result.done(): spin(.03)
    assert result.result().result.error_code == 0
    spin(.3)
    assert call(service, SetBool.Request(data=True)).success
    spin(.5)
    mode(True)
    rejected = action.send_goal_async(goal)
    while not rejected.done(): spin(.03)
    assert not rejected.result().accepted, 'Inactive trajectory controller accepted an action'
    initial = {n: measured[n] for n in arm_names}
    def fk_pose(q, link=None):
        req = GetPositionFK.Request()
        req.header.frame_id = 'g_base'
        req.fk_link_names = [link or ('gripper_tool_frame' if gripper else 'joint6_flange')]
        req.robot_state.joint_state.name = arm_names + (['gripper_controller'] if gripper else [])
        req.robot_state.joint_state.position = list(q) + ([measured['gripper_controller']] if gripper else [])
        response = call(fk, req)
        assert response.error_code.val == 1
        return response.pose_stamped[0].pose
    target_q = [measured[n] for n in arm_names]
    target_q[0] += .04
    target_pose = fk_pose(target_q)
    msg = InteractiveMarkerFeedback()
    msg.header.frame_id = 'g_base'
    msg.client_id = 'live_follow_integration'
    msg.marker_name = 'tool_target'
    msg.event_type = InteractiveMarkerFeedback.POSE_UPDATE
    msg.pose = target_pose
    invalid = deepcopy(msg)
    invalid.pose.position.x = float('nan')
    before_invalid = {n: measured[n] for n in arm_names}
    feedback.publish(invalid)
    spin(.3)
    assert max(abs(measured[n] - before_invalid[n]) for n in arm_names) < .01
    invalid = deepcopy(msg)
    invalid.pose.orientation.x = invalid.pose.orientation.y = invalid.pose.orientation.z = invalid.pose.orientation.w = 0.
    feedback.publish(invalid)
    spin(.2)
    assert max(abs(measured[n] - before_invalid[n]) for n in arm_names) < .01
    feedback.publish(msg)
    command_start = len(command_times)
    begin = time.monotonic()
    rate_start = begin
    latency = None
    for _ in range(200):
        spin(.03)
        if latency is None and max(abs(measured[n] - initial[n]) for n in arm_names) > .0001:
            latency = time.monotonic() - begin
    assert latency is not None, "The first stationary marker goal was skipped"
    latencies = [latency]
    # Ten distinct marker targets exercise continuous updates, not a stored path.
    for i in range(9):
        target_q[0] = initial[arm_names[0]] + (.04 if i % 2 else -.04)
        target_pose = fk_pose(target_q)
        before = measured[arm_names[0]]
        msg.pose = target_pose
        feedback.publish(msg)
        begin = time.monotonic()
        first = None
        for _ in range(20):
            spin(.03)
            if first is None and abs(measured[arm_names[0]] - before) > .0001:
                first = time.monotonic() - begin
        latencies.append(first)
    spin(5)
    moved = max(abs(measured[n] - initial[n]) for n in arm_names)
    actual = fk_pose([measured[n] for n in arm_names])
    error = math.dist([actual.position.x, actual.position.y, actual.position.z],
                      [target_pose.position.x, target_pose.position.y, target_pose.position.z])
    assert moved > .01, ('Arm did not follow', moved, status[0])
    assert error < .002, ('Target not reached', error, status[0])
    publish_hz = (len(command_times) - command_start) / (time.monotonic() - rate_start)
    if gripper:
        scene_client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        obstacle = CollisionObject()
        obstacle.id = 'live_integration_obstacle'
        obstacle.header.frame_id = 'g_base'
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.04, .04, .04])
        obstacle.primitives = [box]
        obstacle.primitive_poses = [fk_pose([measured[n] for n in arm_names], 'gripper_left1')]
        obstacle.operation = CollisionObject.ADD
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.robot_state.is_diff = True
        req.scene.world.collision_objects = [obstacle]
        assert call(scene_client, req).success
        spin(.4)
        held = measured['gripper_controller']
        grip.publish(Float64(data=-.2))
        spin(1)
        assert abs(measured['gripper_controller'] - held) < .01, 'Gripper moved through an obstacle'
        obstacle.operation = CollisionObject.REMOVE
        req.scene.world.collision_objects = [obstacle]
        assert call(scene_client, req).success
        spin(.3)
        for value in (-.15, .15, -.5, .1):
            grip.publish(Float64(data=value))
            spin(3)
            assert abs(measured['gripper_controller'] - value) < .03, (value, measured['gripper_controller'], status[0])
            for name, multiplier in [('gripper_base_to_gripper_left2', 1), ('gripper_left3_to_gripper_left1', -1), ('gripper_base_to_gripper_right3', -1), ('gripper_base_to_gripper_right2', -1), ('gripper_right3_to_gripper_right1', 1)]:
                assert abs(measured[name + '_mimic'] - multiplier*value) < .03, (name, measured[name + '_mimic'], value)
    # An unreachable target remains velocity bounded; it is not treated as reached.
    unreachable = deepcopy(msg)
    unreachable.pose.position.x += 2.
    previous = {n: measured[n] for n in arm_names}
    feedback.publish(unreachable)
    spin(.5)
    assert all(math.isfinite(measured[n]) and abs(measured[n]-previous[n]) < .2 for n in arm_names)
    response = call(service, SetBool.Request(data=False))
    assert response.success, response.message
    mode(False)
    # Re-enable starts from measured state rather than the previous unreachable target.
    reenable = {n: measured[n] for n in arm_names}
    assert call(service, SetBool.Request(data=True)).success
    spin(.5)
    assert max(abs(measured[n]-reenable[n]) for n in arm_names) < .003, 'Re-enable used an old target' 
    mode(True)
    spin(2, beat=False)
    assert not enabled[0], 'Lost heartbeat did not stop following'
    mode(False)
    # Pausing physics must trip a wall-clock watchdog even while ROS time stops.
    assert call(service, SetBool.Request(data=True)).success
    pause = node.create_client(EmptyService, '/pause_physics')
    unpause = node.create_client(EmptyService, '/unpause_physics')
    call(pause, EmptyService.Request())
    spin(.9)
    assert not enabled[0], 'Paused simulation did not stop live mode'
    call(unpause, EmptyService.Request())
    spin(3)
    mode(False)
    assert call(service, SetBool.Request(data=False)).success
    # Loss of measured joints, while controller manager and physics still run.
    assert call(service, SetBool.Request(data=True)).success
    switch = node.create_client(SwitchController, '/controller_manager/switch_controller')
    req = SwitchController.Request()
    req.strictness = SwitchController.Request.STRICT
    req.deactivate_controllers = ['joint_state_broadcaster']
    req.timeout.sec = 2
    assert call(switch, req).ok
    spin(1.5)
    assert not enabled[0], 'Missing joint states did not stop live mode'
    req.deactivate_controllers = []
    req.activate_controllers = ['joint_state_broadcaster']
    assert call(switch, req).ok
    spin(.5)
    mode(False)
    # Missing live controller: activation must fail closed, then recover after reloading it.
    unload = node.create_client(UnloadController, '/controller_manager/unload_controller')
    load = node.create_client(LoadController, '/controller_manager/load_controller')
    configure = node.create_client(ConfigureController, '/controller_manager/configure_controller')
    assert call(unload, UnloadController.Request(name=live_names[0])).ok
    before = {n: measured[n] for n in arm_names}
    assert not call(service, SetBool.Request(data=True)).success
    spin(.4)
    assert not enabled[0]
    assert max(abs(measured[n] - before[n]) for n in arm_names) < .01
    assert call(load, LoadController.Request(name=live_names[0])).ok
    assert call(configure, ConfigureController.Request(name=live_names[0])).ok
    assert call(service, SetBool.Request(data=False)).success
    mode(False)
    print(json.dumps({'variant_gripper': gripper, 'max_arm_delta': moved, 'position_error_m': error, 'command_hz': publish_hz, 'first_motion_latency_s': latencies, 'status': status[0], 'watchdog': 'passed'}))
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
