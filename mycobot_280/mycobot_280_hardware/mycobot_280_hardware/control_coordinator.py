"""Single RViz destination router; never owns or writes a USB connection."""

import copy
import hashlib
import json
import threading
import time

import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String, Float64MultiArray
from std_srvs.srv import SetBool
from mycobot_280_control_interfaces.msg import HardwareCommand, HardwareStatus
from mycobot_280_control_interfaces.srv import SetHardwareMode
from .robot_monitor_node import DEFAULT_JOINT_NAMES
from .selection_policy import SelectionPolicy, StateSource


def trajectory_signature(trajectory):
    value = copy.deepcopy(trajectory)
    value.joint_trajectory.header.stamp.sec = 0
    value.joint_trajectory.header.stamp.nanosec = 0
    canonical = json.dumps(
        message_to_ordereddict(value),
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def wait_future(future, timeout=3.0):
    ready = threading.Event()
    future.add_done_callback(lambda _: ready.set())
    if not ready.wait(timeout):
        raise TimeoutError("El controlador no respondió a tiempo")
    return future.result()


def planning_goal_for_destination(goal, physical):
    request = copy.deepcopy(goal)
    if physical:
        # The shared planner uses the normal model limits. Preserve the former
        # physical limits (.1 rad/s, .2 rad/s²) in physical requests only.
        velocity = .2 if request.request.group_name == 'gripper_group' else .1
        for field, factor in [('max_velocity_scaling_factor', velocity),
                              ('max_acceleration_scaling_factor', .2)]:
            value = getattr(request.request, field)
            if not 0. < value <= 1.:
                value = .1
            setattr(request.request, field, value * factor)
    return request


class ControlCoordinator(Node):
    def __init__(self):
        super().__init__("mycobot_control")
        self.gripper = self.declare_parameter("adaptive_gripper", False).value
        self.physical_state_timeout = float(
            self.declare_parameter("physical_state_timeout_s", 0.8).value)
        self.names = list(DEFAULT_JOINT_NAMES) + (
            ["gripper_controller"] if self.gripper else []
        )
        self.policy = SelectionPolicy()
        self.lock = threading.RLock()
        self.transition = threading.Lock()
        self.changing = False
        self.physical_stopping_until = 0.
        self.group = ReentrantCallbackGroup()
        self.sources = {
            name: StateSource(self.names, self.physical_state_timeout
                              if name == "physical" else .5)
            for name in ("simulation", "physical")
        }
        self.token = ""
        self.sequence = 0
        self.heartbeat = 0.0
        self.hardware = HardwareStatus()
        self.hardware_time = 0.0
        self.grip_command = None
        self.status = "Destino: Gazebo"
        self.enabled = False
        self.last_state = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.physical_pub = self.create_publisher(
            Bool, "/mycobot_control/physical", qos
        )
        self.status_pub = self.create_publisher(String, "/mycobot_control/status", qos)
        self.state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.command_pub = self.create_publisher(
            HardwareCommand, "/physical/control/command", 1
        )
        self.heartbeat_pub = self.create_publisher(
            String, "/physical/control/heartbeat", 1
        )
        self.stop_live_client = self.create_client(
            SetBool, "/mycobot_live/set_enabled", callback_group=self.group
        )
        self.mode_client = self.create_client(
            SetHardwareMode, "/physical/control/set_mode", callback_group=self.group
        )
        self.sim_list = self.create_client(
            ListControllers,
            "/simulation/controller_manager/list_controllers",
            callback_group=self.group,
        )
        self.sim_switch = self.create_client(
            SwitchController,
            "/simulation/controller_manager/switch_controller",
            callback_group=self.group,
        )
        self.selection = self.create_service(
            SetBool,
            "/mycobot_control/select_physical",
            self.select_physical,
            callback_group=self.group,
        )
        self.list_service = self.create_service(
            ListControllers,
            "/controller_manager/list_controllers",
            self.list_controllers,
            callback_group=self.group,
        )
        self.switch_service = self.create_service(
            SwitchController,
            "/controller_manager/switch_controller",
            self.switch_controllers,
            callback_group=self.group,
        )
        self.subs = []
        for source in self.sources:
            self.subs.append(
                self.create_subscription(
                    JointState,
                    f"/{source}/joint_states",
                    lambda msg, source=source: self.measured(source, msg),
                    10,
                    callback_group=self.group,
                )
            )
        self.subs += [
            self.create_subscription(
                HardwareStatus,
                "/physical/control/status",
                self.hardware_status,
                qos,
                callback_group=self.group,
            ),
            self.create_subscription(
                Empty,
                "/mycobot_live/heartbeat",
                self.panel_heartbeat,
                1,
                callback_group=self.group,
            ),
            self.create_subscription(
                Bool,
                "/mycobot_live/enabled",
                self.live_status,
                qos,
                callback_group=self.group,
            ),
            self.create_subscription(
                Float64MultiArray,
                "/mycobot280m5_live_arm_controller/commands",
                self.arm_command,
                1,
                callback_group=self.group,
            ),
            self.create_subscription(
                Float64MultiArray,
                "/mycobot280m5_live_gripper_controller/commands",
                self.gripper_command,
                1,
                callback_group=self.group,
            ),
        ]
        self.sim_arm = self.create_publisher(
            Float64MultiArray,
            "/simulation/mycobot280m5_live_arm_controller/commands",
            1,
        )
        self.sim_grip = self.create_publisher(
            Float64MultiArray,
            "/simulation/mycobot280m5_live_gripper_controller/commands",
            1,
        )
        self.actions = []
        self.action_clients = {}
        controllers = ["mycobot280m5_group_controller"]
        if self.gripper:
            controllers.append("mycobot280m5_gripper_controller")
        for controller in controllers:
            action = controller + "/follow_joint_trajectory"
            for destination in self.sources:
                self.action_clients[(action, destination)] = ActionClient(
                    self,
                    FollowJointTrajectory,
                    f"/{destination}/{action}",
                    callback_group=self.group,
                )
            self.actions.append(
                ActionServer(
                    self,
                    FollowJointTrajectory,
                    "/" + action,
                    execute_callback=lambda h, a=action: self.forward(
                        h, FollowJointTrajectory, a
                    ),
                    goal_callback=self.accept_job,
                    cancel_callback=lambda _: CancelResponse.ACCEPT,
                    callback_group=self.group,
                )
            )
        for action, kind in [
            ("move_action", MoveGroup),
            ("execute_trajectory", ExecuteTrajectory),
        ]:
            self.action_clients[(action, "internal")] = ActionClient(
                self, kind, "/internal/" + action, callback_group=self.group
            )
            self.actions.append(
                ActionServer(
                    self,
                    kind,
                    "/" + action,
                    execute_callback=lambda h, a=action, k=kind: self.execute_moveit(h, k, a),
                    goal_callback=self.accept_moveit,
                    cancel_callback=lambda _: CancelResponse.ACCEPT,
                    callback_group=self.group,
                )
            )
        # Automatic following owns a separate entry point so the ordinary RViz
        # buttons cannot preempt it. Both routes share planning and destination
        # limits; controller actions below remain available to MoveIt's executor.
        self.actions.append(ActionServer(
            self, MoveGroup, "/mycobot_live/move_action",
            execute_callback=lambda h: self.execute_moveit(
                h, MoveGroup, "move_action", automatic=True),
            goal_callback=self.accept_moveit,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.group,
        ))
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            0.1, self.tick, clock=self.watchdog_clock, callback_group=self.group
        )
        self.physical_command_sub = self.create_subscription(
            HardwareCommand, "/mycobot_live/physical_command", self.physical_command,
            1, callback_group=self.group)
        self.publish_status()

    def selected_source(self):
        return self.sources["physical" if self.policy.physical else "simulation"]

    def publish_status(self):
        self.physical_pub.publish(Bool(data=self.policy.physical))
        destination = (
            "Destino: brazo físico" if self.policy.physical else "Destino: Gazebo"
        )
        message = (
            self.status
            if self.status.startswith(destination)
            else destination + " · " + self.status
        )
        if (
            not self.policy.physical
            and self.status == destination
            and self.hardware.connected
        ):
            message += " · " + self.hardware.message
        self.status_pub.publish(String(data=message))

    def hardware_status(self, msg):
        with self.lock:
            if (
                msg.driver_id == self.hardware.driver_id
                and msg.status_sequence <= self.hardware.status_sequence
            ):
                return
            self.hardware, self.hardware_time = msg, time.monotonic()

    def panel_heartbeat(self, _):
        with self.lock:
            self.heartbeat = time.monotonic()

    def live_status(self, msg):
        with self.lock:
            self.enabled = msg.data

    def measured(self, source, msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            if not self.sources[source].update(msg.name, msg.position, stamp, now):
                return
            normalized = copy.deepcopy(msg)
            normalized.header.stamp = self.get_clock().now().to_msg()
            self.last_state[source] = normalized
            if source == ("physical" if self.policy.physical else "simulation"):
                self.state_pub.publish(normalized)

    def physical_mode(self, selected, live=False):
        if not self.mode_client.service_is_ready():
            raise ValueError("Controlador físico no disponible")
        request = SetHardwareMode.Request(
            selected=selected, live=live, session_id=self.token
        )
        with self.lock:
            stopping = bool(self.hardware.selected and self.hardware.live and not live)
            started = time.monotonic()
            if stopping:
                self.policy.live = False  # Close forwarding before waiting for USB stop.
                self.physical_stopping_until = started + 3.
        try:
            reply = wait_future(self.mode_client.call_async(request))
            if not reply.success:
                raise ValueError(reply.message)
            if stopping and selected:
                # The driver publishes a complete measured sample after stop.
                # Wait briefly for that DDS topic alongside the service reply.
                deadline = min(self.physical_stopping_until, time.monotonic() + .25)
                while True:
                    with self.lock:
                        received = self.sources['physical'].received
                    if received is not None and received > started:
                        break
                    if time.monotonic() >= deadline:
                        raise ValueError("No llegó una medición completa tras la parada")
                    time.sleep(.01)
            with self.lock:
                self.token = reply.session_id
                self.hardware = HardwareStatus(
                    connected=bool(selected or self.hardware.connected),
                    selected=selected,
                    live=bool(selected and live),
                    fault=False,
                    session_id=reply.session_id,
                    message=reply.message,
                    driver_id=reply.driver_id,
                    status_sequence=reply.status_sequence,
                )
                self.hardware_time = time.monotonic()
            return reply
        finally:
            if stopping:
                with self.lock:
                    self.physical_stopping_until = 0.

    def select_physical(self, req, res):
        if not self.transition.acquire(blocking=False):
            res.message = "Cambio de destino en curso"
            return res
        try:
            with self.lock:
                self.changing = True
                if self.policy.jobs or self.policy.live or self.enabled:
                    raise ValueError(
                        "Detené seguimiento y ejecución antes de cambiar destino"
                    )
                if req.data == self.policy.physical and self.policy.available:
                    self.status = "Destino: brazo físico" if req.data else "Destino: Gazebo"
                    res.success, res.message = True, self.status
                    return res
                if req.data and (
                    not self.sources["physical"].fresh(time.monotonic())
                    or not self.hardware.connected
                    or self.hardware.fault
                    or time.monotonic() - self.heartbeat > 0.75
                ):
                    raise ValueError(
                        "Brazo físico no disponible: revisá USB, mediciones, firmware y panel"
                    )
                detached = (not req.data and not self.hardware.selected
                            and not self.hardware.live)
            if detached:
                # Explicit user selection of simulation does not require USB.
                # Do not clear a physical fault or claim its stop was verified.
                with self.lock:
                    self.token = ""
                    self.sequence = 0
            elif req.data or self.token or self.hardware.selected:
                self.physical_mode(req.data)
            with self.lock:
                self.policy.select(req.data)
                self.grip_command = None
                self.status = "Destino: brazo físico" if req.data else "Destino: Gazebo"
                source = "physical" if req.data else "simulation"
                if (
                    self.sources[source].fresh(time.monotonic())
                    and source in self.last_state
                ):
                    current = copy.deepcopy(self.last_state[source])
                    current.header.stamp = self.get_clock().now().to_msg()
                    self.state_pub.publish(current)
                res.success = True
        except Exception as exc:
            self.status = str(exc)
        finally:
            with self.lock:
                self.changing = False
                res.message = self.status
                self.publish_status()
            self.transition.release()
        return res

    def accept_job(self, _):
        with self.lock:
            if self.changing or not self.selected_source().fresh(time.monotonic()):
                return GoalResponse.REJECT
            try:
                self.policy.begin_job()
                return GoalResponse.ACCEPT
            except ValueError:
                return GoalResponse.REJECT

    def accept_moveit(self, _):
        # Humble MoveGroupInterface dereferences an absent result on transport
        # rejection. Accept only the action envelope; execute_moveit must still
        # acquire ownership and validate provenance before forwarding anything.
        return GoalResponse.ACCEPT

    def execute_moveit(self, handle, kind, action, automatic=False):
        with self.lock:
            if (self.enabled and not automatic) or (automatic and not self.enabled):
                self.status = (
                    "Detené seguimiento antes de usar Plan/Execute manual"
                    if self.enabled else "Seguimiento desactivado"
                )
                self.publish_status()
                result = kind.Result()
                result.error_code.val = -1
                handle.abort()
                return result
            accept = self.accept_execute if kind is ExecuteTrajectory else self.accept_job
            if accept(handle.request) != GoalResponse.ACCEPT:
                if self.policy.live:
                    self.status = "Detené el seguimiento antes de Plan o Execute"
                elif kind is not ExecuteTrajectory:
                    self.status = "Plan no disponible: revisá el destino y su estado"
                self.publish_status()
                result = kind.Result()
                result.error_code.val = -1
                handle.abort()
                return result
        return self.forward(handle, kind, action)

    def accept_execute(self, request):
        with self.lock:
            try:
                signature = trajectory_signature(request.trajectory)
            except (ValueError, TypeError):
                return GoalResponse.REJECT
            if not self.policy.valid_plan(signature):
                self.status = (
                    "Plan descartado: volvé a planificar para el destino seleccionado"
                )
                self.publish_status()
                return GoalResponse.REJECT
            return self.accept_job(request)

    def forward(self, handle, kind, action):
        with self.lock:
            destination = "physical" if self.policy.physical else "simulation"
            generation = self.policy.generation
        client = self.action_clients[
            (
                action,
                "internal" if kind in (MoveGroup, ExecuteTrajectory) else destination,
            )
        ]
        child = None
        try:
            if not client.wait_for_server(timeout_sec=2.0):
                raise RuntimeError("Acción del destino no disponible")
            child = wait_future(
                client.send_goal_async(
                    planning_goal_for_destination(handle.request, destination == 'physical')
                    if kind is MoveGroup else handle.request,
                    feedback_callback=lambda fb: handle.publish_feedback(fb.feedback),
                )
            )
            if not child.accepted:
                raise RuntimeError("El destino rechazó la ejecución")
            result_future = child.get_result_async()
            started = time.monotonic()
            cancel_sent = False
            canceled_at = None
            while not result_future.done():
                with self.lock:
                    lost = (
                        not self.policy.available
                        or generation != self.policy.generation
                    )
                if (handle.is_cancel_requested or lost) and not cancel_sent:
                    child.cancel_goal_async()
                    cancel_sent = True
                    canceled_at = time.monotonic()
                if time.monotonic() - started > 180.0 or (
                    canceled_at is not None and time.monotonic() - canceled_at > 5.0
                ):
                    raise TimeoutError(
                        "Ejecución sin respuesta o cancelación sin confirmar"
                    )
                time.sleep(0.02)
            result = result_future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                with self.lock:
                    if (
                        not self.policy.available
                        or generation != self.policy.generation
                    ):
                        raise RuntimeError("La sesión cambió durante la ejecución")
                    if (
                        kind is MoveGroup
                        and handle.request.planning_options.plan_only
                        and result.result.error_code.val == 1
                    ):
                        self.policy.record_plan(
                            trajectory_signature(result.result.planned_trajectory),
                            generation,
                        )
                handle.succeed()
            elif (
                result.status == GoalStatus.STATUS_CANCELED
                and handle.is_cancel_requested
            ):
                handle.canceled()
            else:
                handle.abort()
            return result.result
        except Exception as exc:
            if destination == "physical":
                with self.lock:
                    if generation == self.policy.generation:
                        self.lose_physical(
                            "Falló la comunicación de ejecución: " + str(exc)
                        )
            if child is not None and child.accepted:
                child.cancel_goal_async()
            result = kind.Result()
            if kind is FollowJointTrajectory:
                result.error_code, result.error_string = -1, str(exc)
            else:
                result.error_code.val = -1
            handle.abort()
            self.get_logger().error(str(exc))
            return result
        finally:
            with self.lock:
                self.policy.end_job()

    def controller_names(self):
        live = ["mycobot280m5_live_arm_controller"]
        planned = ["mycobot280m5_group_controller"]
        if self.gripper:
            live.append("mycobot280m5_live_gripper_controller")
            planned.append("mycobot280m5_gripper_controller")
        return live, planned

    def list_controllers(self, req, res):
        try:
            if not self.policy.physical:
                if not self.sim_list.service_is_ready():
                    return res
                return wait_future(self.sim_list.call_async(req))
            with self.lock:
                live, planned = self.controller_names()
                operational = (
                    self.policy.available
                    and self.hardware.selected
                    and not self.hardware.fault
                )
                for name in live + planned:
                    active = operational and ((name in live) == self.hardware.live)
                    res.controller.append(
                        ControllerState(
                            name=name, state="active" if active else "inactive"
                        )
                    )
        except Exception as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=3.0)
        return res

    def switch_controllers(self, req, res):
        if not self.transition.acquire(blocking=False):
            return res
        try:
            with self.lock:
                if self.policy.jobs or self.changing or not self.policy.available:
                    return res
                live, planned = self.controller_names()
                activating = set(req.activate_controllers) == set(live)
                if (
                    set(req.deactivate_controllers)
                    != set(planned if activating else live)
                    or set(req.activate_controllers)
                    != set(live if activating else planned)
                    or req.strictness != req.STRICT
                ):
                    return res
            if self.policy.physical:
                self.physical_mode(True, activating)
                # Use confirmed service result immediately; status topic may arrive later.
                with self.lock:
                    self.hardware.live = activating
                    self.hardware.selected = True
            else:
                if not self.sim_switch.service_is_ready():
                    return res
                response = wait_future(self.sim_switch.call_async(req))
                if not response.ok:
                    return res
            with self.lock:
                self.policy.live = activating
                res.ok = True
        except Exception as exc:
            self.status = str(exc)
            self.publish_status()
        finally:
            self.transition.release()
        return res

    def arm_command(self, msg):
        with self.lock:
            if not self.policy.physical:
                self.sim_arm.publish(msg)
            # Physical commands carry explicit per-group intent on their own
            # atomic message; legacy simulation topics never authorize USB.
            return

    def physical_command(self, msg):
        with self.lock:
            if not self.policy.physical:
                return
            if not self.policy.live or not self.policy.available or not self.token:
                return
            if len(msg.positions) != len(self.names):
                return
            self.sequence += 1
            self.command_pub.publish(
                HardwareCommand(
                    stamp=self.get_clock().now().to_msg(),
                    session_id=self.token,
                    sequence=self.sequence,
                    positions=list(msg.positions),
                    arm_enabled=msg.arm_enabled,
                    gripper_enabled=msg.gripper_enabled,
                )
            )

    def gripper_command(self, msg):
        with self.lock:
            if not self.policy.physical:
                self.sim_grip.publish(msg)
            elif len(msg.data) == 1:
                self.grip_command = msg.data[0]

    def lose_physical(self, reason):
        # Caller holds lock. Keep the destination visible, close both command gates.
        if not self.policy.available:
            return
        now = time.monotonic()
        self.get_logger().warn(
            f"{reason} [panel_age={now-self.heartbeat:.3f}, "
            f"status_age={now-self.hardware_time:.3f}, "
            f"state_age={now-(self.sources['physical'].received or 0):.3f}]"
        )
        self.policy.lost()
        self.policy.live = False
        if self.stop_live_client.service_is_ready():
            self.stop_live_client.call_async(SetBool.Request(data=False))
        if self.mode_client.service_is_ready():
            self.mode_client.call_async(
                SetHardwareMode.Request(selected=False, session_id=self.token)
            )
        self.token = ""
        self.status = reason
        self.publish_status()

    def tick(self):
        with self.lock:
            if self.policy.physical and self.policy.available:
                now = time.monotonic()
                if now < self.physical_stopping_until:
                    # A bounded stop operation owns USB; no live forwarding is
                    # allowed. Its acknowledgement and fresh sample are checked
                    # before normal measurement watchdog checks resume.
                    if self.token:
                        self.heartbeat_pub.publish(String(data=self.token))
                    self.publish_status()
                    return
                if (
                    now - self.heartbeat > 0.75
                    or now - self.hardware_time > self.physical_state_timeout
                    or not self.hardware.selected
                    or self.hardware.fault
                    or not self.sources["physical"].fresh(now)
                ):
                    self.lose_physical(
                        "Brazo físico desconectado o sesión vencida; volvé a seleccionar. "
                        + self.hardware.message
                    )
                elif self.token:
                    self.heartbeat_pub.publish(String(data=self.token))
            self.publish_status()


def main(args=None):
    rclpy.init(args=args)
    node = ControlCoordinator()
    executor = MultiThreadedExecutor(num_threads=12)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.token and node.mode_client.service_is_ready():
            node.mode_client.call_async(
                SetHardwareMode.Request(selected=False, session_id=node.token)
            )
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
