"""Dock task decoding and POI routing with no snapping or displacement retries."""

from dataclasses import dataclass
import math
from typing import Callable, Mapping

from nav_tasks.core.handlers.navigate_to import NavigateToPoi, NavigateToPoiRequest
from nav_tasks.core.ports import Clock, DockBackend
from nav_tasks.core.registry import TaskContext
from nav_tasks.core.retry import MonotonicClock, RetryFailure, RetryPolicy
from nav_tasks.domain.errors import ConfigError
from nav_tasks.domain.tasks import DockGoal, NavOutcome, NavStatus
from nav_tasks.domain.types import PoseRecord, StoredPoi


@dataclass(frozen=True)
class DockRequest:
    """Detached target; an optional POI retains legacy WS diagnostics and status labels.

    Attributes:
        poi: 按标识解析的 Dock POI，直接位姿入站时可为 None。
        pose: 已校验的 Dock 目标位姿，直接目标与 POI 路由均提供。
    """

    pose: PoseRecord
    poi: StoredPoi | None = None


class Dock:
    """Run the managed docking port once and throttle only scalar task progress."""

    def __init__(
        self, backend: DockBackend, *, clock: Clock | None = None,
        progress_min_interval_sec: float = 1.0, retry_delay_sec: float = 1.0,
        on_failure: Callable[[TaskContext, RetryFailure], None] | None = None,
    ) -> None:
        """Inject execution and reporting without changing the worker's Dock policy.

        Args:
            backend: Managed Dock port using the one shared motion backend.
            clock: Monotonic reporting/diagnostic clock.
            progress_min_interval_sec: Legacy progress interval, not a raw-feedback filter.
            retry_delay_sec: Kept for legacy audit provenance; the retry count is always zero.
            on_failure: Optional frozen single-attempt POI failure audit.

        Raises:
            ConfigError: A reporting interval is invalid.
        """
        if not math.isfinite(progress_min_interval_sec) or progress_min_interval_sec < 0:
            raise ConfigError('progress_min_interval_sec must be finite and nonnegative')
        self._backend = backend
        self._clock = clock if clock is not None else MonotonicClock()
        self._interval = progress_min_interval_sec
        self._retry = RetryPolicy(0, retry_delay_sec)
        self._on_failure = on_failure

    def decode(self, payload: Mapping[str, object]) -> DockRequest:
        """Validate direct Dock pose requests before preempting any existing task.

        Args:
            payload: Domain payload containing an already-decoded PoseRecord under pose.

        Returns:
            Immutable planar target; frame interpretation stays at the ingress boundary.

        Raises:
            ConfigError: The pose type or coordinates are invalid.
        """
        pose = payload.get('pose')
        if not isinstance(pose, PoseRecord) or any(
                type(value) not in (int, float) or not math.isfinite(value)
                for value in (pose.x, pose.y, pose.yaw)):
            raise ConfigError('dock requires a PoseRecord with finite planar coordinates')
        return DockRequest(pose)

    async def run(self, context: TaskContext, payload: object) -> NavOutcome:
        """Run direct or POI docking exactly once, fencing canceled outcomes and feedback.

        Args:
            context: Sole task owner's loop-thread callbacks and identity.
            payload: Decoded immutable DockRequest.

        Returns:
            Final docking outcome; POI failures retain old one-attempt diagnostic wrapping.

        Raises:
            TypeError: A different payload was registered for this handler.
            asyncio.CancelledError: Cancellation fences arrival and later motion.
            Exception: Worker cleanup failed; do not treat it as ordinary Dock failure.
        """
        if not isinstance(payload, DockRequest):
            raise TypeError('Dock requires DockRequest')
        context.check_cancelled()
        context.describe(payload.poi.poi_id if payload.poi else 'dock_pose', 'to_anchor')
        context.start_segment()
        goal = DockGoal(payload.pose, context.task_id, context.sub_id, context.generation)
        started = self._clock.now()
        latest = None
        last_reported = 0.0
        accepting = True

        def feedback(distance):
            nonlocal latest, last_reported
            if not accepting or context.cancel_requested():
                return
            latest = float(distance)
            now = self._clock.now()
            if now - last_reported >= self._interval:
                last_reported = now
                context.feedback(latest)

        async def attempt():
            outcome = await self._backend.dock(goal, feedback)
            context.check_cancelled()
            if outcome.status != NavStatus.FAILED or payload.poi is None:
                return outcome
            poi, pose = payload.poi, payload.pose
            remaining = (f'{latest:.2f}m' if latest is not None
                         else 'n/a(无 feedback,可能 planner 直接失败)')
            elapsed = self._clock.now() - started
            return NavOutcome(NavStatus.FAILED, (
                f'Nav2 终态 status=6(ABORTED), target={poi.poi_id}({poi.name}) '
                f'goal={pose.frame_id}(x={pose.x:.2f}, y={pose.y:.2f}, yaw={pose.yaw:.2f}rad), '
                f'剩余距离={remaining}, 本次耗时={elapsed:.1f}s'))

        def audit(failure):
            if self._on_failure is not None:
                self._on_failure(context, failure)

        try:
            if payload.poi is None:
                return await attempt()
            return await self._retry.run(attempt, self._clock, audit)
        finally:
            accepting = False


class DockAwareNavigation:
    """Route configured POIs before ordinary snap or retry policy can touch them."""

    def __init__(self, navigation: NavigateToPoi, dock: Dock, dock_ids: tuple[str, ...]) -> None:
        """Share the normal decoder and dispatch only explicitly selected IDs to Dock.

        Args:
            navigation: Unchanged ordinary POI skill and repository decoder.
            dock: Managed one-attempt Dock handler.
            dock_ids: Explicit configured IDs; never inferred from a POI's name.
        """
        self._navigation = navigation
        self._dock = dock
        self._dock_ids = frozenset(dock_ids)

    def decode(self, payload: Mapping[str, object]) -> NavigateToPoiRequest:
        """Resolve a detached POI before motion admission using the ordinary decoder.

        Args:
            payload: Domain navigate_to payload.

        Returns:
            Immutable repository snapshot, preserving unknown-ID admission failure.
        """
        return self._navigation.decode(payload)

    async def run(self, context: TaskContext, payload: object) -> NavOutcome:
        """Select Dock or unchanged ordinary navigation under the same task owner.

        Args:
            context: Shared task-owner context.
            payload: Prepared POI target.

        Returns:
            Final outcome from exactly one selected skill.
        """
        if isinstance(payload, NavigateToPoiRequest) and payload.poi.poi_id in self._dock_ids:
            return await self._dock.run(context, DockRequest(payload.poi.map_pose, payload.poi))
        return await self._navigation.run(context, payload)
