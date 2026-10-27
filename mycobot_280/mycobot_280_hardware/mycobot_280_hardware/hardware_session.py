"""Revocable physical motion session; clocks and USB are supplied by the caller."""

import math
import secrets


class HardwareSession:
    def __init__(self, lower, upper, command_period=0.1,
                 goal_tolerance=0.015, shared_step_budget=False,
                 measurement_timeout=0.5):
        if not math.isfinite(command_period) or not .1 <= command_period <= .25:
            raise ValueError("Invalid physical command period")
        if not math.isfinite(goal_tolerance) or not 0 < goal_tolerance <= .05:
            raise ValueError("Invalid physical goal tolerance")
        if not math.isfinite(measurement_timeout) or not .5 <= measurement_timeout <= 1.:
            raise ValueError("Invalid physical measurement timeout")
        self.measurement_timeout = measurement_timeout
        self.command_period = command_period
        self.goal_tolerance = goal_tolerance
        self.shared_step_budget = shared_step_budget
        self.lower, self.upper = tuple(lower), tuple(upper)
        self.selected = self.live = self.fault = False
        self.token = ""
        self.sequence = -1
        self.target = None
        self.arm_enabled = self.gripper_enabled = False
        self.last_sent = None
        self.heartbeat_time = self.command_time = self.send_time = 0.0

    def positions(self, values):
        if len(values) != len(self.lower) or any(
            not math.isfinite(v) or v < lo or v > hi
            for v, lo, hi in zip(values, self.lower, self.upper)
        ):
            raise ValueError("Invalid measured/commanded position or joint limit")
        return tuple(values)

    def select(self, measured, now):
        if self.fault or self.selected:
            raise ValueError("Physical session unavailable or already selected")
        self.positions(measured)
        self.token = secrets.token_hex(16)
        self.selected = True
        self.heartbeat_time = now
        return self.token

    def require(self, token):
        if not self.selected or self.fault or token != self.token:
            raise ValueError("Physical session expired")

    def heartbeat(self, token, now):
        self.require(token)
        self.heartbeat_time = now

    def start_live(self, token, measured, now):
        self.require(token)
        self.last_sent = self.positions(measured)
        self.progress_anchor = self.last_sent
        self.progress_time = now
        self.target = None
        self.sequence = -1
        self.arm_enabled = self.gripper_enabled = False
        self.live = True
        self.command_time = self.send_time = now

    def stop_live(self):
        self.live = False
        self.target = None
        # Rotate the command generation even if the destination remains selected.
        if self.selected:
            self.token = secrets.token_hex(16)
        return self.token

    def command(self, token, sequence, positions, stamp, wall_now, now,
                arm_enabled=True, gripper_enabled=True):
        self.require(token)
        if (
            not self.live
            or sequence <= self.sequence
            or not math.isfinite(stamp)
            or not -0.05 <= wall_now - stamp <= 0.25
        ):
            raise ValueError("Stale, unordered or inactive live command")
        target = self.positions(positions)
        self.target = target
        self.arm_enabled = bool(arm_enabled)
        self.gripper_enabled = bool(gripper_enabled)
        self.sequence = sequence
        self.command_time = now

    def next_command(self, measured, now):
        if (not self.live or self.target is None
                or now - self.send_time < self.command_period - 1e-6):
            return None
        measured = self.positions(measured)
        enabled = (self.arm_enabled,) * 6 + (self.gripper_enabled,) * (len(measured) - 6)
        # Idle packets renew the communication watchdog, but cannot become
        # position commands. Rebase inactive channels on fresh measurements.
        self.last_sent = tuple(q if not active else prev for q, prev, active in
                               zip(measured, self.last_sent, enabled))
        self.target = tuple(q if not active else goal for q, goal, active in
                            zip(measured, self.target, enabled))
        if any(abs(v - q) > 0.06 for v, q in zip(self.last_sent, measured)):
            raise ValueError("Physical robot is not following the commanded position")
        if (
            max(abs(q - anchor) for q, anchor in zip(measured, self.progress_anchor))
            > 0.002
        ):
            self.progress_anchor, self.progress_time = measured, now
        elif max(abs(q - target) for q, target in zip(measured, self.target)) <= self.goal_tolerance:
            self.progress_time = now
        elif now - self.progress_time > 3.0:
            raise ValueError("No measured progress toward the live target")
        step = 0.1 * min(0.15, max(0.0, now - self.send_time))
        settled = tuple(
            abs(goal - prev) <= .0005 and abs(goal - actual) <= self.goal_tolerance
            for actual, prev, goal in zip(measured, self.last_sent, self.target)
        )
        target = tuple(
            prev if held else
            max(q - 0.03, min(q + 0.03, max(prev - step, min(prev + step, goal))))
            for q, prev, goal, held in zip(measured, self.last_sent, self.target, settled)
        )
        if self.shared_step_budget:
            # Preserve the IK direction. Clipping joints independently destroys
            # the coordinated shoulder/elbow/wrist cancellation: an upward TCP
            # target can initially go down. All advancing arm joints share one
            # interpolation fraction, including the measured-lead constraint.
            delta = [0. if held else goal - prev for goal, prev, held in
                     zip(self.target[:6], self.last_sent[:6], settled[:6])]
            budget = .06 * min(.2, max(0., now - self.send_time))
            scale = min(1., step / max(1e-12, max(abs(q) for q in delta)),
                        budget / max(1e-12, sum(abs(q) for q in delta)))
            for actual, prev, dq in zip(measured[:6], self.last_sent[:6], delta):
                if abs(dq) > 1e-12:
                    room = actual + math.copysign(.03, dq) - prev
                    scale = min(scale, max(0., room / dq))
            target = tuple(prev + scale * dq for prev, dq in
                           zip(self.last_sent[:6], delta)) + target[6:]
            # An unchanged joint already at its commanded goal may retain the
            # verified encoder residual. Chasing that residual would move its
            # command away from the goal and ratchet it on every arm packet.
            # Advancing commands still obey the original measured-lead bound.
            if any(not held and abs(q - actual) > .030001
                   for q, actual, held in zip(target, measured, settled)):
                raise ValueError("Measured jump exceeds physical command lead")
        self.last_sent = target
        self.send_time = now
        return target

    def watchdog(self, now, measurement_time):
        if not self.selected:
            return ""
        if now - self.heartbeat_time > 0.75:
            return "heartbeat"
        if measurement_time is None or now - measurement_time > self.measurement_timeout:
            return "measurements"
        if self.live and now - self.command_time > 0.35:
            return "commands"
        return ""

    def revoke(self, fault=False):
        self.fault |= fault
        self.selected = self.live = False
        self.token = ""
        self.target = None
