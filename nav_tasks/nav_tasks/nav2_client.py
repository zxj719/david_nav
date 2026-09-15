"""Shared one-attempt NavigateToPose backend with explicit action ownership.

No ROS imports occur until construction without injected test clients. Runtime
assembly owns one instance shared by skills; retry policy stays in the core.
Other Nav2 action types can share the same ActionSession rather than bypassing
its single motion slot.
"""

import math
from typing import Callable

from nav_tasks.adapters.ros.action_session import ActionClientLike, ActionSession, ActionState
from nav_tasks.core.ports import Clock, FeedbackCallback
from nav_tasks.core.retry import MonotonicClock
from nav_tasks.domain.errors import ConfigError
from nav_tasks.domain.tasks import NavGoal, NavOutcome, NavStatus

_STATUS_NAMES = {
    0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
    4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED',
}


class Nav2Client:
    """Convert domain POI goals into single-attempt ROS action requests."""

    def __init__(
        self, node: object, action_name: str, server_timeout_sec: float,
        *, client: ActionClientLike | None = None,
        goal_factory: Callable[[], object] | None = None,
        session: ActionSession | None = None, clock: Clock | None = None,
        cancel_timeout_sec: float = 5.0,
    ) -> None:
        """Inject a node and optionally fake action machinery for pure tests.

        Args:
            node: Existing ROS node; its owning runtime spins the executor.
            action_name: Configured action name, never a robot-specific constant.
            server_timeout_sec: Maximum nonblocking server readiness wait.
            client: Injected action client, or None to construct the real ROS client.
            goal_factory: Goal message factory; required with an injected client.
            session: Shared motion slot, or None to create one for this backend.
            clock: Monotonic attempt-duration and readiness clock.
            cancel_timeout_sec: Cleanup caller deadline when creating the session.

        Raises:
            ConfigError: Configuration or the injected client/factory pair is invalid.
        """
        if not action_name:
            raise ConfigError('action_name must be nonempty')
        if not math.isfinite(server_timeout_sec) or server_timeout_sec < 0:
            raise ConfigError('server_timeout_sec must be finite and nonnegative')
        if (client is None) != (goal_factory is None):
            raise ConfigError('client and goal_factory must be injected together')
        self._node = node
        self._action_name = action_name
        self._server_timeout = server_timeout_sec
        self._clock = clock if clock is not None else MonotonicClock()
        self._session = session if session is not None else ActionSession(
            cancel_timeout_sec=cancel_timeout_sec, clock=self._clock)
        self._owns_client = client is None
        self._destroyed = False
        if client is None:
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient

            client = ActionClient(node, NavigateToPose, action_name)
            goal_factory = NavigateToPose.Goal
        self._client = client
        self._goal_factory = goal_factory

    async def navigate(self, goal: NavGoal, on_feedback: FeedbackCallback) -> NavOutcome:
        """Execute one attempt and preserve legacy detailed failure text.

        Args:
            goal: Domain target and correlation fields.
            on_feedback: Loop-thread distance callback; the skill applies throttling.

        Returns:
            Success, external cancellation, or a readable one-attempt failure.

        Raises:
            asyncio.CancelledError: Cancellation propagates after arranging cleanup.
            Exception: Action ownership could not be safely resolved.
        """
        started = self._clock.now()
        remaining_distance: float | None = None

        def feedback(raw: object) -> None:
            nonlocal remaining_distance
            remaining_distance = float(getattr(raw.feedback, 'distance_remaining', 0.0))
            on_feedback(remaining_distance)

        result = await self._session.execute(
            self._client, lambda: self._build_goal(goal), feedback,
            server_timeout_sec=self._server_timeout)
        description = self._describe_goal(goal)
        if result.state == ActionState.UNAVAILABLE:
            return NavOutcome(NavStatus.FAILED, (
                f"Nav2 action server '{self._action_name}' 在 "
                f'{self._server_timeout:.1f}s 内不可用(未起来或 lifecycle 未 Activate), '
                f'{description}'))
        if result.state == ActionState.REJECTED:
            return NavOutcome(NavStatus.FAILED, (
                'Nav2 拒绝 goal(bt_navigator rejected;常见于 TF 未就绪/上一个 '
                f'goal 未清), {description}'))
        status = result.response.status
        if status == 4:
            return NavOutcome(NavStatus.SUCCEEDED)
        if status == 5:
            return NavOutcome(NavStatus.CANCELED)
        elapsed = self._clock.now() - started
        remaining = (
            f'{remaining_distance:.2f}m' if remaining_distance is not None
            else 'n/a(无 feedback,可能 planner 直接失败)')
        return NavOutcome(NavStatus.FAILED, (
            f'Nav2 终态 status={status}({_STATUS_NAMES.get(status, "?")}), '
            f'{description}, 剩余距离={remaining}, 本次耗时={elapsed:.1f}s'))

    async def cancel(self) -> None:
        """Wait for the shared action session to settle its current goal.

        Raises:
            Exception: Cleanup failed or timed out; the session retains ownership.
        """
        await self._session.cancel()

    async def close(self) -> None:
        """Settle owned work before destroying a client created by this backend.

        Injected clients are destroyed only by their external owner.

        Raises:
            Exception: Cleanup is incomplete; the client is deliberately not destroyed.
        """
        await self._session.close()
        if self._owns_client and not self._destroyed:
            self._client.destroy()
            self._destroyed = True

    @staticmethod
    def _describe_goal(goal: NavGoal) -> str:
        poi = goal.poi
        pose = poi.map_pose
        return (
            f'target={poi.poi_id}({poi.name}) '
            f'goal={pose.frame_id}(x={pose.x:.2f}, y={pose.y:.2f}, yaw={pose.yaw:.2f}rad)')

    def _build_goal(self, goal: NavGoal) -> object:
        message = self._goal_factory()
        pose = goal.poi.map_pose
        message.pose.header.frame_id = pose.frame_id
        message.pose.header.stamp = self._node.get_clock().now().to_msg()
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        message.pose.pose.position.z = pose.z
        quaternion = pose.orientation()
        message.pose.pose.orientation.x = quaternion['x']
        message.pose.pose.orientation.y = quaternion['y']
        message.pose.pose.orientation.z = quaternion['z']
        message.pose.pose.orientation.w = quaternion['w']
        return message
