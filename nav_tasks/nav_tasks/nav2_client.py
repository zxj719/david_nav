"""Shared one-attempt NavigateToPose/Spin backend with explicit action ownership.

No ROS imports occur until construction without injected test clients. Runtime
assembly owns one instance shared by skills; retry policy stays in the core.
Both action types use the same ActionSession and cannot bypass its motion slot.
"""

import math
from typing import Callable

from nav_tasks.adapters.ros.action_session import (
    ActionClientLike, ActionOutcome, ActionSession, ActionState)
from nav_tasks.core.ports import Clock, FeedbackCallback
from nav_tasks.core.retry import MonotonicClock
from nav_tasks.domain.errors import ConfigError, RuntimeAssemblyError
from nav_tasks.domain.tasks import NavGoal, NavOutcome, NavStatus, SpinGoal
from nav_tasks.domain.types import PoseRecord

_STATUS_NAMES = {
    0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
    4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED',
}


class Nav2Client:
    """Convert domain navigation and rotation goals into mutually exclusive attempts."""

    def __init__(
        self, node: object, action_name: str, server_timeout_sec: float,
        *, client: ActionClientLike | None = None,
        goal_factory: Callable[[], object] | None = None,
        session: ActionSession | None = None, clock: Clock | None = None,
        cancel_timeout_sec: float = 5.0,
        spin_action_name: str | None = None, spin_server_timeout_sec: float = 5.0,
        spin_client: ActionClientLike | None = None,
        spin_goal_factory: Callable[[], object] | None = None,
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
            spin_action_name: Explicit Spin endpoint, or None to leave Spin disabled.
            spin_server_timeout_sec: Spin readiness deadline, separate from navigation.
            spin_client: Optional injected rotation client; requires an explicit endpoint.
            spin_goal_factory: Goal factory paired with an injected rotation client.

        Raises:
            ConfigError: Configuration or the injected client/factory pair is invalid.
            RuntimeAssemblyError: Construction rollback failed; retain and retry cleanup.
            Exception: Action client construction failed after rollback succeeded.
        """
        if not action_name:
            raise ConfigError('action_name must be nonempty')
        if not math.isfinite(server_timeout_sec) or server_timeout_sec < 0:
            raise ConfigError('server_timeout_sec must be finite and nonnegative')
        if (client is None) != (goal_factory is None):
            raise ConfigError('client and goal_factory must be injected together')
        if (spin_client is None) != (spin_goal_factory is None):
            raise ConfigError('spin_client and spin_goal_factory must be injected together')
        if spin_action_name is not None and not spin_action_name:
            raise ConfigError('spin_action_name must be nonempty when configured')
        if spin_client is not None and spin_action_name is None:
            raise ConfigError('an injected spin client requires spin_action_name')
        if not math.isfinite(spin_server_timeout_sec) or spin_server_timeout_sec < 0:
            raise ConfigError('spin_server_timeout_sec must be finite and nonnegative')
        self._node = node
        self._action_name = action_name
        self._server_timeout = server_timeout_sec
        self._clock = clock if clock is not None else MonotonicClock()
        self._session = session if session is not None else ActionSession(
            cancel_timeout_sec=cancel_timeout_sec, clock=self._clock)
        self._owned_clients: list[object] = []
        self._spin_action_name = spin_action_name
        self._spin_server_timeout = spin_server_timeout_sec
        try:
            if client is None:
                from nav2_msgs.action import NavigateToPose
                from rclpy.action import ActionClient

                client = ActionClient(node, NavigateToPose, action_name)
                self._owned_clients.append(client)
                goal_factory = NavigateToPose.Goal
            if spin_action_name is not None and spin_client is None:
                from nav2_msgs.action import Spin
                from rclpy.action import ActionClient

                spin_client = ActionClient(node, Spin, spin_action_name)
                self._owned_clients.append(spin_client)
                spin_goal_factory = Spin.Goal
        except Exception as error:
            try:
                self._destroy_owned_clients()
            except Exception:
                raise RuntimeAssemblyError(error, self._rollback_clients) from error
            raise
        self._client = client
        self._goal_factory = goal_factory
        self._spin_client = spin_client
        self._spin_goal_factory = spin_goal_factory

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

    async def spin(self, goal: SpinGoal, on_feedback: FeedbackCallback) -> NavOutcome:
        """Run one relative rotation through the same session as ordinary navigation.

        This sends the server allowance but adds no implicit retry, settling or
        local result deadline. Dock owns those policies. Angle sign/magnitude is
        preserved; feedback is angular distance in radians, not remaining meters.

        Args:
            goal: Relative yaw and nonnegative ROS-representable time allowance.
            on_feedback: Loop-thread angular-distance callback.

        Returns:
            One terminal result or readable admission failure.

        Raises:
            ConfigError: Spin is disabled or goal fields are not representable.
            asyncio.CancelledError: Caller canceled; shared cleanup retains ownership.
            Exception: The shared session could not resolve action ownership safely.
        """
        def feedback(raw):
            on_feedback(float(raw.feedback.angular_distance_traveled))

        build = self._spin_factory(goal)
        result = await self._session.execute(
            self._spin_client, build, feedback,
            server_timeout_sec=self._spin_server_timeout)
        if result.state == ActionState.UNAVAILABLE:
            return NavOutcome(NavStatus.FAILED, (
                f"Spin action server '{self._spin_action_name}' 在 "
                f'{self._spin_server_timeout:.1f}s 内不可用'))
        if result.state == ActionState.REJECTED:
            return NavOutcome(NavStatus.FAILED, 'Spin 目标被拒绝')
        status = result.response.status
        if status == 4:
            return NavOutcome(NavStatus.SUCCEEDED)
        if status == 5:
            return NavOutcome(NavStatus.CANCELED)
        return NavOutcome(NavStatus.FAILED, (
            f'Spin 终态 status={status}({_STATUS_NAMES.get(status, "?")})'))

    async def navigate_pose(
        self, pose: PoseRecord, on_feedback: Callable[[object], None], *,
        server_timeout_sec: float, send_timeout_sec: float, result_timeout_sec: float,
        on_timeout: Callable[[ActionState], None] | None = None,
    ) -> ActionOutcome:
        """Execute a Dock pose through the same clients and slot, retaining raw feedback.

        Args:
            pose: Already-resolved target; no POI lookup or snapping is performed.
            on_feedback: Loop-thread callback receiving the complete ROS feedback wrapper.
            server_timeout_sec: Stage readiness budget.
            send_timeout_sec: Stage admission budget.
            result_timeout_sec: Stage execution budget, starting after admission.
            on_timeout: Synchronous deadline audit hook, before owned cleanup.

        Returns:
            Admission/terminal/settled-timeout result for the worker's frozen audit.

        Raises:
            Exception: Admission validation or motion cleanup failed; do not continue.
            asyncio.CancelledError: Explicit cancellation after attempting owned cleanup.
        """
        return await self._session.execute(
            self._client, lambda: self._build_pose(pose), on_feedback,
            server_timeout_sec=server_timeout_sec, send_timeout_sec=send_timeout_sec,
            result_timeout_sec=result_timeout_sec, on_timeout=on_timeout)

    async def spin_action(
        self, goal: SpinGoal, *, server_timeout_sec: float, send_timeout_sec: float,
        result_timeout_sec: float, on_timeout: Callable[[ActionState], None] | None = None,
    ) -> ActionOutcome:
        """Execute a Dock correction with explicit stage deadlines on the shared slot.

        Args:
            goal: Signed yaw and server-side allowance, unchanged from ordinary Spin.
            server_timeout_sec: Stage readiness budget.
            send_timeout_sec: Stage admission budget.
            result_timeout_sec: Stage result budget, separate from server allowance.
            on_timeout: Synchronous deadline audit hook, before owned cleanup.

        Returns:
            Raw settled outcome; angular feedback is not converted into POI distance.

        Raises:
            Exception: Invalid goal or unresolved action ownership.
            asyncio.CancelledError: Explicit cancellation after attempting owned cleanup.
        """
        build = self._spin_factory(goal)
        return await self._session.execute(
            self._spin_client, build, lambda raw: None,
            server_timeout_sec=server_timeout_sec, send_timeout_sec=send_timeout_sec,
            result_timeout_sec=result_timeout_sec, on_timeout=on_timeout)

    async def cancel(self) -> None:
        """Wait for the shared action session to settle its current goal.

        Raises:
            Exception: Cleanup failed or timed out; the session retains ownership.
        """
        await self._session.cancel()

    async def settle(self) -> None:
        """Seal and settle the action session while retaining its ROS clients.

        Keep the ROS executor alive until this succeeds. The outer owner may
        then quiesce callback threads before calling close to destroy clients.

        Raises:
            Exception: Cleanup is incomplete; session ownership is retained.
        """
        await self._session.close()

    async def close(self) -> None:
        """Settle shared work before destroying clients created by this backend.

        Injected clients are destroyed only by their external owner.

        Raises:
            Exception: Cleanup is incomplete; the client is deliberately not destroyed.
        """
        await self.settle()
        self._destroy_owned_clients()

    def _destroy_owned_clients(self) -> None:
        failure = None
        for client in tuple(reversed(self._owned_clients)):
            try:
                client.destroy()
            except Exception as error:
                failure = error
            else:
                self._owned_clients.remove(client)
        if failure is not None:
            raise failure

    async def _rollback_clients(self) -> None:
        # Construction never submitted work. Do not seal a caller-provided session.
        self._destroy_owned_clients()

    @staticmethod
    def _describe_goal(goal: NavGoal) -> str:
        poi = goal.poi
        pose = poi.map_pose
        return (
            f'target={poi.poi_id}({poi.name}) '
            f'goal={pose.frame_id}(x={pose.x:.2f}, y={pose.y:.2f}, yaw={pose.yaw:.2f}rad)')

    def _build_goal(self, goal: NavGoal) -> object:
        return self._build_pose(goal.poi.map_pose)

    def _build_pose(self, pose: PoseRecord) -> object:
        message = self._goal_factory()
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

    def _spin_factory(self, goal: SpinGoal) -> Callable[[], object]:
        if self._spin_client is None:
            raise ConfigError('Spin is not configured on this shared backend')
        if (type(goal.target_yaw) not in (int, float) or
                not math.isfinite(goal.target_yaw) or
                abs(goal.target_yaw) > 3.4028234663852886e38):
            raise ConfigError('target_yaw must be a finite float32 value')
        seconds = goal.time_allowance_sec
        if (type(seconds) not in (int, float) or not math.isfinite(seconds) or
                seconds < 0 or seconds >= 2**31):
            raise ConfigError('time_allowance_sec must be nonnegative and fit ROS Duration')
        nanoseconds = int(seconds * 1000000000)

        def build():
            message = self._spin_goal_factory()
            message.target_yaw = float(goal.target_yaw)
            message.time_allowance.sec = nanoseconds // 1000000000
            message.time_allowance.nanosec = nanoseconds % 1000000000
            return message

        return build
