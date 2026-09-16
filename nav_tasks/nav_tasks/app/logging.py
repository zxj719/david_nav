"""Stable retry audit text, kept outside pure retry policy and transport code."""

from typing import Callable

from nav_tasks.core.registry import TaskContext
from nav_tasks.core.retry import RetryFailure


def _print_line(line: str) -> None:
    print(line, flush=True)


class RetryAudit:
    """Render legacy bridge retry lines through an injected line sink."""

    def __init__(self, emit: Callable[[str], None] = _print_line) -> None:
        """Inject a complete-line sink without starting background work.

        Args:
            emit: Synchronous line sink, defaulting to flushed stdout.
        """
        self._emit = emit

    def __call__(self, context: TaskContext, failure: RetryFailure) -> None:
        """Write one failure and its retry/exhaustion decision with frozen wording.

        Args:
            context: Correlated task identity.
            failure: Retry-policy observation; this observer does not choose policy.
        """
        attempt, total = failure.attempt, failure.total_attempts
        self._emit(
            f'[nav2_action_bridge] NAV FAILURE attempt {attempt}/{total} '
            f'request_id={context.task_id} sub_id={context.sub_id}: {failure.detail}')
        if attempt < total:
            self._emit(
                '[nav2_action_bridge] suppressing failure report, re-publishing goal in '
                f'{failure.delay_sec:.1f}s (attempt {attempt + 1}/{total})')
        else:
            self._emit(
                f'[nav2_action_bridge] retry budget exhausted ({total} attempts) — '
                f'reporting on_error to server: {failure.detail}')
