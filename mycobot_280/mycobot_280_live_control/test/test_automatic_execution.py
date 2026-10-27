"""Public ROS regression: real backend, controlled planner, no USB or Gazebo.

Run in an isolated ROS_DOMAIN_ID after building the live control package.
"""
import copy
from pathlib import Path
import subprocess
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from controller_manager_msgs.srv import ListControllers, SwitchController
from controller_manager_msgs.msg import ControllerState
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPlanningScene
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Bool, Float64, String, Header
from std_srvs.srv import SetBool
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate


@pytest.fixture
def rig(tmp_path, request):
    has_gripper = getattr(request, "param", True)
    rclpy.init()
    node = rclpy.create_node('automatic_execution_test')
    group = ReentrantCallbackGroup()
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    data = dict(goals=[], release=False, fail=False, canceled=0, markers=[],
                enabled=False, status='', beat=True, joints=True, delay_accept=False,
                accept_release=False, switches=0)
    names = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
             'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6',
             'gripper_controller']
    beat = node.create_publisher(Empty, '/mycobot_live/heartbeat', 1)
    joints = node.create_publisher(JointState, '/joint_states', 10)
    feedback = node.create_publisher(InteractiveMarkerFeedback, '/mycobot_live/marker/feedback', 10)
    grip = node.create_publisher(Float64, '/mycobot_live/gripper_target', 1)
    physical = node.create_publisher(Bool, '/mycobot_control/physical', qos)
    node.create_subscription(Bool, '/mycobot_live/enabled', lambda m: data.update(enabled=m.data), qos)
    node.create_subscription(String, '/mycobot_live/status', lambda m: data.update(status=m.data), qos)
    node.create_subscription(InteractiveMarkerUpdate, '/mycobot_live/marker/update',
                             lambda m: data['markers'].extend(m.markers), 10)
    def publish():
        if data['beat']:
            beat.publish(Empty())
        if data['joints']:
            joints.publish(JointState(header=Header(
                stamp=node.get_clock().now().to_msg()), name=names, position=[0.] * 6 + [-.1]))
    node.create_timer(.04, publish)
    def controllers(req, res):
        for name in ['mycobot280m5_group_controller', 'mycobot280m5_gripper_controller',
                     'mycobot280m5_live_arm_controller', 'mycobot280m5_live_gripper_controller']:
            res.controller.append(ControllerState(name=name, state='inactive' if '_live_' in name else 'active'))
        return res
    node.create_service(ListControllers, '/controller_manager/list_controllers', controllers,
                        callback_group=group)
    node.create_service(GetPlanningScene, '/get_planning_scene', lambda req, res: res,
                        callback_group=group)
    def switch(req, res):
        data['switches'] += 1
        res.ok = True
        return res
    node.create_service(SwitchController, '/controller_manager/switch_controller', switch,
                        callback_group=group)
    def accept(goal):
        while data['delay_accept'] and not data['accept_release']:
            time.sleep(.01)
        from rclpy.action import GoalResponse
        return GoalResponse.ACCEPT
    def execute(handle):
        data['goals'].append(copy.deepcopy(handle.request))
        while not data['release'] and not handle.is_cancel_requested:
            time.sleep(.01)
        result = MoveGroup.Result()
        if handle.is_cancel_requested:
            data['canceled'] += 1
            handle.canceled()
            result.error_code.val = -7
        elif data['fail']:
            handle.abort()
            result.error_code.val = -1
        else:
            handle.succeed()
            result.error_code.val = 1
        return result
    server = ActionServer(node, MoveGroup, '/move_action', execute,
                          goal_callback=accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
                          callback_group=group)
    client = node.create_client(SetBool, '/mycobot_live/set_enabled')
    gazebo = Path(get_package_share_directory('mycobot_280_gazebo'))
    moveit = Path(get_package_share_directory('mycobot_280_moveit2'))
    params = {'robot_description': xacro.process_file(str(gazebo / 'urdf/mycobot_280_m5_gazebo.urdf.xacro'),
              mappings={'adaptive_gripper': str(has_gripper).lower()}).toxml(),
              'robot_description_semantic': (moveit / ('config/mycobot_280_m5_gripper.srdf' if has_gripper else 'config/mycobot_280_m5.srdf')).read_text(),
              'robot_description_kinematics': yaml.safe_load((moveit / 'config/kinematics.yaml').read_text()),
              'adaptive_gripper': has_gripper, 'use_sim_time': False}
    live = Path(get_package_share_directory('mycobot_280_live_control'))
    params.update(yaml.safe_load((live / 'config/servo.yaml').read_text()))
    params['moveit_servo'].update(yaml.safe_load((live / 'config/pose_tracking.yaml').read_text())['moveit_servo'])
    param_file = tmp_path / 'params.yaml'
    param_file.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    log = (tmp_path / 'node.log').open('w+')
    binary = Path(get_package_prefix('mycobot_280_live_control')) / 'lib/mycobot_280_live_control/live_follow_node'
    process = subprocess.Popen([str(binary), '--ros-args', '--params-file', str(param_file)], stdout=log, stderr=log)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    def until(predicate, timeout=6):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            if process.poll() is not None:
                log.flush(); log.seek(0)
                pytest.fail('Backend exited: ' + log.read())
            time.sleep(.02)
        pytest.fail('Condition timed out: ' + data['status'])
    def enable(value):
        assert client.wait_for_service(timeout_sec=8)
        future = client.call_async(SetBool.Request(data=value))
        until(future.done)
        return future.result()
    def drag(x):
        msg = InteractiveMarkerFeedback(header=copy.deepcopy(data['markers'][-1].header),
            marker_name='tool_target', client_id='test', control_name='move_0',
            pose=copy.deepcopy(data['markers'][-1].pose))
        msg.pose.position.x += x
        msg.event_type = msg.MOUSE_DOWN
        feedback.publish(msg)
        time.sleep(.05)
        msg.event_type = msg.POSE_UPDATE
        feedback.publish(msg)
        time.sleep(.05)
        msg.event_type = msg.MOUSE_UP
        feedback.publish(msg)
    data.update(until=until, enable=enable, drag=drag, grip=grip, physical=physical,
                feedback=feedback)
    try:
        until(lambda: client.service_is_ready())
        time.sleep(.6)
        yield data
    finally:
        data['release'] = data['accept_release'] = True
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait()
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        server.destroy(); node.destroy_node(); rclpy.shutdown(); log.close()


def start(rig):
    response = rig['enable'](True)
    assert response.success, response.message
    rig['until'](lambda: rig['markers'] and rig['enabled'])
    time.sleep(.3)
    assert not rig['goals'], 'Activation must not execute a stored target'
    assert rig['switches'] == 0, 'Plan controllers must stay active'


def test_drag_automatically_plans_and_executes_latest_pending_pose(rig):
    start(rig)
    rig['drag'](.002)
    rig['until'](lambda: len(rig['goals']) == 1)
    first = rig['goals'][0]
    assert not first.planning_options.plan_only
    assert first.request.start_state.is_diff
    assert first.request.group_name == 'arm_group'
    assert first.request.goal_constraints[0].position_constraints[0].link_name == 'gripper_tool_frame'
    rig['drag'](.003)
    rig['drag'](.004)
    time.sleep(.3)
    assert len(rig['goals']) == 1, 'Must serialize execution'
    rig['release'] = True
    rig['until'](lambda: len(rig['goals']) == 2)
    position = rig['goals'][1].request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0].position
    assert position.x == pytest.approx(rig['markers'][-1].pose.position.x + .004)
    time.sleep(.5)
    assert len(rig['goals']) == 2


def test_gripper_automatically_uses_planning_without_authorizing_arm(rig):
    start(rig)
    rig['grip'].publish(Float64(data=.1))
    rig['until'](lambda: len(rig['goals']) == 1)
    goal = rig['goals'][0]
    assert not goal.planning_options.plan_only
    assert goal.request.group_name == 'gripper_group'
    constraint = goal.request.goal_constraints[0].joint_constraints[0]
    assert constraint.joint_name == 'gripper_controller' and constraint.position == .1


@pytest.mark.parametrize('cause', ['stop', 'heartbeat', 'joints'])
def test_stop_or_lost_connection_cancels_and_discards_pending(rig, cause):
    start(rig)
    rig['drag'](.002)
    rig['until'](lambda: len(rig['goals']) == 1)
    rig['drag'](.004)
    if cause == 'stop':
        assert rig['enable'](False).success
    else:
        rig['beat' if cause == 'heartbeat' else 'joints'] = False
    rig['until'](lambda: not rig['enabled'] and rig['canceled'] == 1)
    rig['release'] = True
    time.sleep(.4)
    assert len(rig['goals']) == 1


def test_planning_failure_does_not_retry_forever(rig):
    start(rig)
    rig['fail'] = rig['release'] = True
    rig['drag'](.002)
    rig['until'](lambda: len(rig['goals']) == 1)
    time.sleep(.6)
    assert len(rig['goals']) == 1
    assert 'fall' in rig['status'].lower() or 'no se pudo' in rig['status'].lower()


def test_late_goal_acceptance_after_stop_is_canceled(rig):
    start(rig)
    rig['delay_accept'] = True
    rig['drag'](.002)
    time.sleep(.3)
    assert rig['enable'](False).success
    assert not rig['enable'](True).success, 'Must wait for cancellation acknowledgement'
    rig['accept_release'] = True
    rig['until'](lambda: rig['canceled'] == 1)


def test_continuous_drag_starts_without_waiting_for_mouse_release(rig):
    start(rig)
    msg = InteractiveMarkerFeedback(header=copy.deepcopy(rig['markers'][-1].header),
        marker_name='tool_target', client_id='test', control_name='move_0',
        pose=copy.deepcopy(rig['markers'][-1].pose))
    msg.event_type = msg.MOUSE_DOWN
    rig['feedback'].publish(msg)
    time.sleep(.05)
    msg.event_type = msg.POSE_UPDATE
    for i in range(20):
        msg.pose.position.x += .0001
        rig['feedback'].publish(msg)
        time.sleep(.03)
    assert rig['goals'], 'Continuous drag must start planning before release'


def test_unsolicited_or_nonfinite_pose_updates_never_plan(rig):
    start(rig)
    msg = InteractiveMarkerFeedback(header=copy.deepcopy(rig['markers'][-1].header),
        marker_name='tool_target', client_id='test', control_name='move_0',
        pose=copy.deepcopy(rig['markers'][-1].pose))
    msg.event_type = msg.POSE_UPDATE
    msg.pose.position.x += .003
    rig['feedback'].publish(msg)
    time.sleep(.3)
    assert not rig['goals']
    msg.event_type = msg.MOUSE_DOWN
    rig['feedback'].publish(msg)
    time.sleep(.05)
    msg.event_type = msg.POSE_UPDATE
    msg.pose.position.x = float('nan')
    rig['feedback'].publish(msg)
    time.sleep(.3)
    assert not rig['goals']


def test_reactivation_starts_without_previous_pending_targets(rig):
    start(rig)
    rig['drag'](.002)
    rig['until'](lambda: len(rig['goals']) == 1)
    rig['drag'](.004)
    assert rig['enable'](False).success
    rig['until'](lambda: rig['canceled'] == 1)
    time.sleep(.3)
    assert rig['enable'](True).success
    time.sleep(.4)
    assert len(rig['goals']) == 1


@pytest.mark.parametrize('rig', [False], indirect=True)
def test_arm_without_gripper_uses_flange_goal(rig):
    start(rig)
    rig['drag'](.002)
    rig['until'](lambda: len(rig['goals']) == 1)
    goal = rig['goals'][0]
    assert goal.request.goal_constraints[0].position_constraints[0].link_name == 'joint6_flange'
    assert not goal.planning_options.plan_only


def test_selected_physical_destination_uses_same_public_planner(rig):
    rig['physical'].publish(Bool(data=True))
    time.sleep(.2)
    start(rig)
    rig['drag'](.002)
    rig['until'](lambda: len(rig['goals']) == 1)
    assert not rig['goals'][0].planning_options.plan_only
    assert rig['switches'] == 0
