"""Motion execution abstraction and FollowJointTrajectory client implementation."""

from abc import ABC, abstractmethod
import math
import time
from typing import Sequence

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from mycobot_280_pick_place_demo.models_and_validators import (
    StepExecutionResult,
    WaypointSpec,
)


class MotionExecutor(ABC):
    """Abstract interface for executing a motion step (joint-space now, MoveIt 2 in future)."""

    @abstractmethod
    def wait_for_ready(self, timeout_sec: float) -> bool:
        """Return True if the underlying motion server is ready within timeout_sec."""

    @abstractmethod
    def execute_waypoint(
        self,
        waypoint: WaypointSpec,
        joint_names: Sequence[str],
        goal_response_timeout_sec: float,
        execution_timeout_margin_sec: float,
    ) -> StepExecutionResult:
        """Execute a validated waypoint and wait for its completion."""


def _float_seconds_to_duration_msg(duration_sec: float) -> Duration:
    whole_sec = int(math.floor(duration_sec))
    nanosec = int(round((duration_sec - whole_sec) * 1e9))
    if nanosec >= 1_000_000_000:
        whole_sec += 1
        nanosec -= 1_000_000_000
    return Duration(sec=whole_sec, nanosec=nanosec)


class FollowJointTrajectoryExecutor(MotionExecutor):
    """Executes joint-space waypoints over a FollowJointTrajectory action server."""

    def __init__(self, node: Node, action_name: str) -> None:
        self._node = node
        self._action_name = action_name
        self._client = ActionClient(node, FollowJointTrajectory, action_name)

    def wait_for_ready(self, timeout_sec: float) -> bool:
        """Wait for the FollowJointTrajectory action server without sending commands."""
        return bool(self._client.wait_for_server(timeout_sec=timeout_sec))

    def _spin_until_future_complete(self, future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            rclpy.spin_once(self._node, timeout_sec=min(0.05, remaining))
        return bool(future.done())

    def execute_waypoint(
        self,
        waypoint: WaypointSpec,
        joint_names: Sequence[str],
        goal_response_timeout_sec: float,
        execution_timeout_margin_sec: float,
    ) -> StepExecutionResult:
        """Send a single-point FollowJointTrajectory goal and wait for its result."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = list(joint_names)

        point = JointTrajectoryPoint()
        point.positions = list(waypoint.positions)
        point.velocities = [0.0] * len(joint_names)
        point.time_from_start = _float_seconds_to_duration_msg(waypoint.duration_sec)
        goal_msg.trajectory.points = [point]

        send_future = self._client.send_goal_async(goal_msg)
        if not self._spin_until_future_complete(send_future, goal_response_timeout_sec):
            return StepExecutionResult(
                success=False,
                stage_name=waypoint.name,
                error_code=-1,
                message=(
                    f"Timed out after {goal_response_timeout_sec:.1f}s waiting for goal "
                    f"acceptance on '{self._action_name}'."
                ),
            )

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return StepExecutionResult(
                success=False,
                stage_name=waypoint.name,
                error_code=-1,
                message=f"Goal for stage '{waypoint.name}' was rejected by '{self._action_name}'.",
            )

        total_exec_timeout = waypoint.duration_sec + execution_timeout_margin_sec
        result_future = goal_handle.get_result_async()
        if not self._spin_until_future_complete(result_future, total_exec_timeout):
            # Best-effort cancellation on execution timeout
            cancel_future = goal_handle.cancel_goal_async()
            self._spin_until_future_complete(cancel_future, 1.0)
            return StepExecutionResult(
                success=False,
                stage_name=waypoint.name,
                error_code=-1,
                message=(
                    f"Timed out after {total_exec_timeout:.1f}s waiting for trajectory "
                    f"completion at stage '{waypoint.name}'. Active goal canceled."
                ),
            )

        wrapped_result = result_future.result()
        if wrapped_result is None:
            return StepExecutionResult(
                success=False,
                stage_name=waypoint.name,
                error_code=-1,
                message=f"Received empty action result for stage '{waypoint.name}'.",
            )

        status = wrapped_result.status
        action_result = wrapped_result.result
        error_code = int(action_result.error_code)
        error_string = str(action_result.error_string or "").strip()

        if (
            status != GoalStatus.STATUS_SUCCEEDED
            or error_code != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            return StepExecutionResult(
                success=False,
                stage_name=waypoint.name,
                error_code=error_code,
                message=(
                    f"Stage '{waypoint.name}' failed with GoalStatus={status}, "
                    f"error_code={error_code}"
                    + (f" ({error_string})" if error_string else "")
                ),
            )

        return StepExecutionResult(
            success=True,
            stage_name=waypoint.name,
            error_code=error_code,
            message=f"Stage '{waypoint.name}' completed successfully.",
        )
