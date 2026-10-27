"""Session-scoped physical transport sharing RobotController's sole USB handle."""

import time
import math
import secrets

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from mycobot_280_control_interfaces.msg import HardwareCommand, HardwareStatus
from mycobot_280_control_interfaces.srv import SetHardwareMode
from .hardware_session import HardwareSession
from .joint_conversion import joint_positions_to_degrees, validate_angles
from .gripper_conversion import radians_to_device_value
from .serial_transport import gripper_write_sent


class HardwareLiveController:
    def __init__(self, node):
        self.node = node
        self.driver_id = secrets.token_hex(16)
        self.status_sequence = 0
        lower, upper = list(node._lower), list(node._upper)
        if node._adaptive_gripper:
            lower.append(-0.5)
            upper.append(0.15)
        self.session = HardwareSession(
            lower, upper,
            command_period=float(node.get_parameter('live_command_period_s').value),
            goal_tolerance=min(.04, node._goal_tol),
            shared_step_budget=True,
            measurement_timeout=float(node.get_parameter('physical_state_timeout_s').value),
        )
        self.measured = None
        self.measured_time = None
        self.measurement_error = "Esperando mediciones"
        self.owner = None
        self.message = "Control físico desactivado"
        self.halt_reason = ""
        self._last_gripper_value = None
        self._last_arm_command = None
        self.group = ReentrantCallbackGroup()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = node.create_publisher(HardwareStatus, "control/status", qos)
        self.service = node.create_service(
            SetHardwareMode,
            "control/set_mode",
            self._service,
            callback_group=self.group,
        )
        self.commands = node.create_subscription(
            HardwareCommand,
            "control/command",
            self._command,
            1,
            callback_group=self.group,
        )
        self.heartbeats = node.create_subscription(
            String, "control/heartbeat", self._heartbeat, 1, callback_group=self.group
        )
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = node.create_timer(
            0.05, self.tick, clock=self.watchdog_clock, callback_group=self.group
        )

    def measured_state(self, positions):
        # Called under the driver's state lock. Never manufacture complete data.
        try:
            self.measured = self.session.positions(positions)
            self.measured_time = time.monotonic()
            self.measurement_error = ""
        except ValueError:
            if len(positions) != len(self.session.lower):
                self.measurement_error = "Faltan mediciones válidas de brazo o pinza"
                # A dropped USB reply is not a new measurement. Keep the last
                # complete sample with its original timestamp; the existing
                # 0.5-second watchdog still expires it. Never renew missing data.
            else:
                self.measured_time = None
                names = list(self.node._names) + (
                    ["pinza"] if self.node._adaptive_gripper else []
                )
                self.measurement_error = "; ".join(
                    f"{name}={q:.4f} fuera del rango [{lo}, {hi}]"
                    for name, q, lo, hi in zip(
                        names, positions, self.session.lower, self.session.upper
                    )
                    if not math.isfinite(q) or not lo <= q <= hi
                )

    def connection_lost(self):
        if self.session.selected:
            self.session.revoke(fault=True)
            self.node._motion_fault = True
            self.message = (
                "USB desconectado; no se pudo confirmar parada. Revisá el robot."
            )
        self.measured = self.measured_time = None

    def authorize_trajectory(self):
        s = self.session
        return (
            s.selected
            and not s.live
            and not s.fault
            and not s.watchdog(time.monotonic(), self.measured_time)
            and self.owner is self.node._arm
        )

    def _service(self, request, response):
        result = self.set_mode(request)
        response.success, response.session_id, response.message = (
            result.success,
            result.session_id,
            result.message,
        )
        response.driver_id = result.driver_id
        response.status_sequence = result.status_sequence
        return response

    def set_mode(self, request):
        n, s = self.node, self.session
        reply = SetHardwareMode.Response()
        with n._goal_lock, n._state_lock:
            try:
                if not request.selected:
                    if s.selected:
                        s.require(request.session_id)
                        self._halt("Control físico desactivado")
                    reply.success = not n._motion_fault
                else:
                    if n._arm is None:
                        raise ValueError(
                            "USB no disponible: revisá el puerto y su acceso en Docker"
                        )
                    if n._motion_fault:
                        raise ValueError(
                            "Fallo de movimiento enclavado: verificá la parada antes de reiniciar"
                        )
                    if not n._allow_motion:
                        raise ValueError(
                            "Brazo USB conectado en solo lectura (allow_motion=false)"
                        )
                    if n._adaptive_gripper and not n._gripper.stop_confirmed:
                        raise ValueError(
                            "Parada específica del gripper sin confirmar en el firmware real"
                        )
                    if n._active_goal:
                        raise ValueError("Hay una trayectoria en ejecución")
                    if (
                        self.measured_time is None
                        or time.monotonic() - self.measured_time > s.measurement_timeout
                    ):
                        raise ValueError(
                            self.measurement_error
                            or "Se necesitan mediciones físicas completas y recientes"
                        )
                    if not s.selected:
                        if request.live:
                            raise ValueError("Seleccioná primero el manipulador físico")
                        s.select(self.measured, time.monotonic())
                        self.owner = n._arm
                    else:
                        s.require(request.session_id)
                        if request.live and not s.live:
                            if n._adaptive_gripper and (
                                not n._gripper.stop_confirmed
                                or not n._arm.supports_gripper_stop
                            ):
                                raise ValueError(
                                    "Parada específica del gripper sin confirmar"
                                )
                            n._arm.set_fresh_mode(1)
                            if n._arm.get_fresh_mode() != 1:
                                raise ValueError(
                                    "No se pudo verificar modo de última consigna del firmware"
                                )
                            s.start_live(s.token, self.measured, time.monotonic())
                            self._last_arm_command = [
                                round(v, 2) for v in joint_positions_to_degrees(
                                    self.measured[:6], n._offsets, n._signs
                                )
                            ]
                            if n._adaptive_gripper:
                                self._last_gripper_value = radians_to_device_value(
                                    self.measured[6], n._gripper_calibration
                                )
                        elif not request.live and s.live:
                            if not self._stop():
                                raise RuntimeError("No se pudo verificar parada física")
                            refreshed_at = time.monotonic()
                            n.poll_once()
                            if (self.measured_time is None
                                    or self.measured_time < refreshed_at):
                                raise RuntimeError("Parada confirmada sin medición completa nueva")
                            s.stop_live()
                            # This authenticated mode request renews the lease
                            # after its blocking stop verification. Motion stays
                            # disabled; subsequent heartbeats use the new token.
                            s.heartbeat_time = time.monotonic()
                    self.message = (
                        "Brazo físico: seguimiento activo"
                        if s.live
                        else "Brazo físico preparado"
                    )
                    self.halt_reason = ""
                    reply.success = True
                reply.session_id = s.token
            except Exception as exc:
                if s.live:
                    self._halt(str(exc))
                elif isinstance(exc, RuntimeError):
                    s.revoke(fault=True)
                    n._motion_fault = True
                self.message = str(exc)
            reply.message = self.message
            self.publish_status()
            reply.driver_id = self.driver_id
            reply.status_sequence = self.status_sequence
        return reply

    def _heartbeat(self, msg):
        with self.node._state_lock:
            try:
                self.session.heartbeat(msg.data, time.monotonic())
            except ValueError:
                pass

    def _command(self, msg):
        with self.node._state_lock:
            try:
                self.session.command(
                    msg.session_id,
                    msg.sequence,
                    msg.positions,
                    msg.stamp.sec + msg.stamp.nanosec * 1e-9,
                    time.time(),
                    time.monotonic(),
                    arm_enabled=msg.arm_enabled,
                    gripper_enabled=msg.gripper_enabled,
                )
            except ValueError as exc:
                # An old packet must never stop or renew a newer session.
                self.node.get_logger().warn(str(exc), throttle_duration_sec=2.0)

    def _stop(self):
        n = self.node
        arm = n._arm
        if arm is None:
            return False
        verified = False
        try:
            for _ in range(3):
                arm.stop()
                n._sleep(0.05)
                if (
                    arm.is_moving() == 0
                    and validate_angles(arm.get_angles()) is not None
                ):
                    verified = True
                    break
        except Exception:
            verified = False
        if n._adaptive_gripper:
            verified_gripper = n._gripper.stop_confirmed and n._gripper.stop()
            verified = bool(verified and verified_gripper)
        return verified

    def stop_motion(self):
        with self.node._goal_lock, self.node._state_lock:
            stopped = self._stop()
            if not stopped:
                self.session.revoke(fault=True)
                self.node._motion_fault = True
                self.message = "Parada física NO verificada; movimiento bloqueado"
            return stopped

    def _halt(self, reason):
        # Close the gate before any serial stop attempt; never auto-rearm.
        self.session.revoke()
        stopped = self._stop()
        if not stopped:
            self.session.revoke(fault=True)
            self.node._motion_fault = True
        self.message = reason + (
            "; parada verificada"
            if stopped
            else "; parada NO verificada, movimiento bloqueado"
        )
        self.halt_reason = self.message
        if reason == "Control físico desactivado":
            self.node.get_logger().info(self.message)
        else:
            self.node.get_logger().warn(self.message)

    def tick(self):
        n, s = self.node, self.session
        # Keep the goal lock across dispatch so selection cannot change, but
        # release the state lock during serial I/O. Otherwise USB retries also
        # block fresh heartbeat/command/measurement callbacks and cause a false
        # watchdog trip. ControlledArm rechecks the session at the actual write.
        with n._goal_lock:
            target = None
            with n._state_lock:
                failure = s.watchdog(time.monotonic(), self.measured_time)
                if s.selected and self.owner is not n._arm:
                    failure = "Cambió la conexión USB"
                if failure:
                    self._halt("Se perdió " + failure)
                if s.live:
                    try:
                        target = s.next_command(self.measured, time.monotonic())
                    except Exception as exc:
                        n.get_logger().warn(
                            f"Live step rejected: {exc}; measured={self.measured}; "
                            f"last_sent={s.last_sent}; target={s.target}")
                        self._halt(str(exc))
                arm = n._arm
            if target is not None:
                try:
                    angles = [round(v, 2) for v in joint_positions_to_degrees(
                        target[:6], n._offsets, n._signs)]
                    if s.arm_enabled and angles != self._last_arm_command:
                        arm.send_angles(angles, min(10, n._max_speed))
                        self._last_arm_command = angles
                    if n._adaptive_gripper and s.gripper_enabled:
                        value = radians_to_device_value(target[6], n._gripper_calibration)
                        if value != self._last_gripper_value:
                            if not gripper_write_sent(
                                    arm.set_gripper_value(value, min(10, n._gripper.speed))):
                                raise RuntimeError("No se pudo despachar la orden del gripper")
                            self._last_gripper_value = value
                except Exception as exc:
                    with n._state_lock:
                        self._halt(str(exc))
            with n._state_lock:
                self.publish_status()

    def publish_status(self):
        self.status_sequence += 1
        s = self.session
        message = self.message
        if (
            self.node._arm is not None
            and not s.selected
            and not self.node._motion_fault
            and not self.halt_reason
        ):
            message = "Brazo USB conectado"
            if not self.node._allow_motion:
                message += " · solo lectura"
            if self.measurement_error:
                message += " · " + self.measurement_error
        self.pub.publish(
            HardwareStatus(
                driver_id=self.driver_id,
                status_sequence=self.status_sequence,
                connected=self.node._arm is not None,
                selected=s.selected,
                live=s.live,
                fault=bool(s.fault or self.node._motion_fault),
                session_id=s.token,
                message=message,
            )
        )

    def shutdown(self):
        with self.node._goal_lock, self.node._state_lock:
            if self.session.selected:
                self._halt("Cerrando controlador físico")
