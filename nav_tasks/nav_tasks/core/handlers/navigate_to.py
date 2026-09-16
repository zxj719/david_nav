"""Sequential POI navigation skill sharing the manager's one navigation backend."""

from dataclasses import dataclass
import math
from typing import Callable, Mapping

from nav_tasks.core.ports import Clock, GoalResolver, NavigationBackend, PoiRepository
from nav_tasks.core.registry import TaskContext
from nav_tasks.core.retry import MonotonicClock, RetryFailure, RetryPolicy
from nav_tasks.domain.errors import ConfigError, PoiNotFoundError
from nav_tasks.domain.tasks import NavGoal, NavOutcome, NavStatus
from nav_tasks.domain.types import StoredPoi


@dataclass(frozen=True)
class NavigateToPoiRequest:
    """Detached anchor captured by the decoder before motion admission."""

    poi: StoredPoi


class NavigateToPoi:
    """Resolve anchor/snap goals and retry each segment without nested callbacks."""

    def __init__(
        self, backend: NavigationBackend, pois: PoiRepository, resolver: GoalResolver,
        retry: RetryPolicy = RetryPolicy(), clock: Clock | None = None,
        progress_min_interval_sec: float = 1.0,
        on_failure: Callable[[TaskContext, RetryFailure], None] | None = None,
    ) -> None:
        """Inject ports and explicit interval/retry policy.

        Args:
            backend: Shared one-attempt motion backend, never a per-skill client.
            pois: Repository used to resolve IDs before admission.
            resolver: Anchor/snap policy; dock routing must use its separate skill.
            retry: Per-segment failure retry policy.
            clock: Monotonic interval clock; defaults to the production clock.
            progress_min_interval_sec: Minimum feedback-report interval per attempt.
            on_failure: Optional synchronous retry audit observer, outside motion policy.

        Raises:
            ConfigError: Feedback interval is negative or nonfinite.
        """
        if not math.isfinite(progress_min_interval_sec) or progress_min_interval_sec < 0:
            raise ConfigError('progress_min_interval_sec must be finite and nonnegative')
        self._backend = backend
        self._pois = pois
        self._resolver = resolver
        self._retry = retry
        self._clock = clock if clock is not None else MonotonicClock()
        self._progress_interval = progress_min_interval_sec
        self._on_failure = on_failure

    def decode(self, payload: Mapping[str, object]) -> NavigateToPoiRequest:
        """Resolve a POI ID before an existing motion task can be preempted.

        Args:
            payload: Domain values containing target_id; no wire message types.

        Returns:
            Immutable target snapshot for the handler.

        Raises:
            ConfigError: Target ID is missing, empty or not a string.
            PoiNotFoundError: The requested POI does not exist.
        """
        target = payload.get('target_id')
        if not isinstance(target, str) or not target.strip():
            raise ConfigError('navigate_to requires a nonempty target_id')
        poi = self._pois.get(target)
        if poi is None:
            raise PoiNotFoundError(f'目标点不存在: {target}')
        return NavigateToPoiRequest(poi)

    async def run(self, context: TaskContext, payload: object) -> NavOutcome:
        """Navigate to snap and, only when newly admissible, to the anchor.

        Args:
            context: Task identity and fenced loop-thread feedback callback.
            payload: NavigateToPoiRequest prepared by decode.

        Returns:
            Final segment outcome; an inaccessible anchor after snap is success.

        Raises:
            TypeError: Registration supplied the wrong domain payload.
            asyncio.CancelledError: Propagates, sealing retries and the next segment.
        """
        if not isinstance(payload, NavigateToPoiRequest):
            raise TypeError('NavigateToPoi requires NavigateToPoiRequest')
        context.check_cancelled()
        anchor = payload.poi
        resolved = self._resolver.resolve(anchor)
        if resolved is None:
            return NavOutcome(NavStatus.FAILED, '目标点不可达，且未找到可行扩圈点')
        phase = 'to_snap' if resolved.used_snap else 'to_anchor'
        outcome = await self._navigate(context, resolved.poi, phase)
        if outcome.succeeded and resolved.used_snap and self._resolver.is_admissible(anchor):
            outcome = await self._navigate(context, anchor, 'to_anchor')
        return outcome

    async def _navigate(self, context: TaskContext, poi: StoredPoi, phase: str) -> NavOutcome:
        context.check_cancelled()
        context.describe(poi.poi_id, phase)
        context.start_segment()
        goal = NavGoal(poi, context.task_id, context.sub_id)

        async def attempt() -> NavOutcome:
            context.check_cancelled()
            last_reported = 0.0
            accepting = True

            def feedback(remaining: float) -> None:
                nonlocal last_reported
                if not accepting:
                    return
                now = self._clock.now()
                if now - last_reported < self._progress_interval:
                    return
                last_reported = now
                context.feedback(float(remaining))

            try:
                outcome = await self._backend.navigate(goal, feedback)
                context.check_cancelled()
                return outcome
            finally:
                # A callback from a previous retry/segment shares the task ID,
                # so the manager generation fence alone cannot reject it.
                accepting = False

        def observe(failure: RetryFailure) -> None:
            if self._on_failure is not None:
                self._on_failure(context, failure)

        return await self._retry.run(attempt, self._clock, observe)
