"""Static safety checks for mycobot_280_hardware."""

import ast
import pathlib

PKG = pathlib.Path(__file__).resolve().parents[1]

# Methods that must NEVER appear anywhere in mycobot_280_hardware:
ALWAYS_FORBIDDEN = {
    'release_all_servos', 'release_servo', 'power_on', 'power_off',
    'send_angle', 'send_coords', 'send_coord', 'sync_send_angles',
    'sync_send_coords', 'jog_angle', 'jog_coord', 'jog_absolute', 'jog_increment_angle',
    'focus_servo', 'focus_all_servos', 'set_servo_data', 'set_color', 'set_gripper_state',
    'set_gripper_value', 'set_pwm_output', 'set_basic_output', 'pause', 'resume',
    'set_fresh_mode', 'set_free_mode', 'set_speed', 'set_servo_calibration',
    'set_encoder', 'set_encoders', 'set_pin_mode', 'set_digital_output',
}

# Methods additionally forbidden in the strictly read-only monitor module:
MONITOR_FORBIDDEN = ALWAYS_FORBIDDEN | {'send_angles', 'stop'}

SOURCES = list((PKG / 'mycobot_280_hardware').glob('*.py'))


def _identifiers(path):
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute):
            yield node.attr
        elif isinstance(node, ast.Name):
            yield node.id
        elif isinstance(node, ast.FunctionDef):
            yield node.name


def test_no_forbidden_servo_commands_in_any_source():
    assert SOURCES
    for path in SOURCES:
        used = set(_identifiers(path))
        # The approved physical session explicitly verifies last-command refresh;
        # it remains forbidden in the monitor and every unrelated module.
        allowed = {
            'robot_control_node.py': {'set_gripper_value', 'set_fresh_mode'},
            'gripper_execution.py': {'set_gripper_value'},
            'hardware_live.py': {'set_gripper_value', 'set_fresh_mode'},
        }
        forbidden = ALWAYS_FORBIDDEN - allowed.get(path.name, set())
        assert not (used & forbidden), (path.name, used & forbidden)


def test_monitor_node_is_strictly_read_only():
    monitor_path = PKG / 'mycobot_280_hardware' / 'robot_monitor_node.py'
    used = set(_identifiers(monitor_path))
    assert not (used & MONITOR_FORBIDDEN), used & MONITOR_FORBIDDEN
    text = monitor_path.read_text()
    assert '_device.get_angles()' in text
    assert '_arm.get_angles()' in text


def test_robot_monitor_launch_has_no_simulation_or_duplicate_publishers():
    path = PKG / 'launch' / 'robot_monitor.launch.py'
    tree = ast.parse(path.read_text())
    packages = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, 'id', '') == 'Node':
            for kw in node.keywords:
                if kw.arg == 'package':
                    packages.append(kw.value.value)
    assert sorted(packages) == [
        'mycobot_280_hardware', 'robot_state_publisher', 'rviz2']
    names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    assert 'IncludeLaunchDescription' not in names
    assert "'use_sim_time': False" in path.read_text()


def test_control_and_moveit_real_launches_enforce_safety_defaults():
    for launch_name in ('robot_control.launch.py', 'moveit_real.launch.py'):
        path = PKG / 'launch' / launch_name
        text = path.read_text()
        assert "'use_sim_time': False" in text
        # allow_motion must default to 'false' in launch arguments
        assert "'allow_motion'" in text
        assert "default_value='false'" in text
        for bad_pkg in (
            'gazebo_ros', 'joint_state_publisher', 'joint_state_publisher_gui',
            'controller_manager',
        ):
            assert f"package='{bad_pkg}'" not in text


def test_moveit_real_launch_scoping_keeps_rviz_enabled():
    text = (PKG / 'launch' / 'moveit_real.launch.py').read_text()
    assert 'GroupAction' in text
    assert 'scoped=True' in text
    for launch_name in ('robot_control.launch.py', 'moveit_real.launch.py'):
        ltext = (PKG / 'launch' / launch_name).read_text()
        assert 'mycobot_280_m5_gazebo.urdf.xacro' in ltext

    try:
        import importlib.util
        from launch import LaunchContext
        from launch_ros.actions import Node
    except ImportError:
        return

    spec = importlib.util.spec_from_file_location(
        'moveit_real', str(PKG / 'launch' / 'moveit_real.launch.py')
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ld = mod.generate_launch_description()

    ctx = LaunchContext()
    enabled_nodes = []
    queue = list(ld.entities)
    while queue:
        entity = queue.pop(0)
        if isinstance(entity, Node):
            cond = entity.condition
            if cond is None or cond.evaluate(ctx):
                enabled_nodes.append(
                    f'{entity._Node__package}/{entity._Node__node_executable}'
                )
            continue
        sub = entity.visit(ctx)
        if sub:
            queue = list(sub) + queue

    assert ctx.launch_configurations.get('use_rviz') == 'true'
    assert ctx.launch_configurations.get('allow_motion') == 'false'
    assert enabled_nodes == [
        'mycobot_280_hardware/robot_control',
        'robot_state_publisher/robot_state_publisher',
        'moveit_ros_move_group/move_group',
        'rviz2/rviz2',
    ]
