"""Single-task retry policy separated from backend goal submission."""

import asyncio
from dataclasses import dataclass
import math
import time
from typing import Awaitable, Callable

from nav_tasks.core.ports import Clock
from nav_tasks.domain.errors import ConfigError
from nav_tasks.domain.tasks import NavOutcome, NavStatus


class MonotonicClock:
    """Default clock for intervals, never for source-pose timestamp comparisons."""

    def now(self) -> float:
        """Read monotonic seconds.

        Returns:
            Current monotonic time.
        """
        return time.monotonic()

    async def sleep(self, seconds: float) -> None:
        """Wait without blocking the event loop.

        Args:
            seconds: Nonnegative duration.
        """
        await asyncio.sleep(seconds)


@dataclass(frozen=True)
class RetryFailure:
    """Read-only failure observation emitted before retry delay or exhaustion."""

    attempt: int
    total_attempts: int
    delay_sec: float
    detail: str


@dataclass(frozen=True)
class RetryPolicy:
    """Retry FAILED outcomes only; never catch cancellation or backend crashes."""

    retry_count: int = 1
    retry_delay_sec: float = 1.0

    def __post_init__(self) -> None:
        """Validate the retry budget before an attempt can run.

        Raises:
            ConfigError: Count or delay is invalid.
        """
        if type(self.retry_count) is not int or self.retry_count < 0:
            raise ConfigError('retry_count must be a nonnegative integer')
        if not math.isfinite(self.retry_delay_sec) or self.retry_delay_sec < 0.0:
            raise ConfigError('retry_delay_sec must be finite and nonnegative')

    async def run(
        self, attempt: Callable[[], Awaitable[NavOutcome]], clock: Clock,
        on_failure: Callable[[RetryFailure], None] | None = None,
    ) -> NavOutcome:
        """Run attempts sequentially and preserve the frozen exhaustion text.

        Args:
            attempt: Factory for a fresh one-attempt coroutine.
            clock: Injectable interval clock.
            on_failure: Optional synchronous audit observer; exceptions propagate.

        Returns:
            First nonfailure outcome or a failure with the legacy retry count.

        Raises:
            asyncio.CancelledError: Cancellation stops the current attempt or delay.
        """
        total = self.retry_count + 1
        for index in range(total):
            outcome = await attempt()
            if outcome.status != NavStatus.FAILED:
                return outcome
            if on_failure is not None:
                on_failure(RetryFailure(index + 1, total, self.retry_delay_sec, outcome.detail))
            if index + 1 < total:
                await clock.sleep(self.retry_delay_sec)
        return NavOutcome(NavStatus.FAILED, f'导航失败(共尝试{total}次): {outcome.detail}')
