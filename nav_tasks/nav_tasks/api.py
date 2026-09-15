"""Public task and value contracts shared with in-process ingress adapters.

Transport code calls only TaskApi/PoiApi. The assembly root can construct TaskManager
and register skills through these exports without importing internal modules.
This API does not start ROS or a transport and does not change legacy entry points.
"""

import asyncio
from typing import Iterable, Protocol

from nav_tasks.core.handlers.poi_commands import MarkPoiResult
from nav_tasks.core.registry import CommandRegistry, TaskContext
from nav_tasks.core.state import TaskManager
from nav_tasks.domain.errors import BusyError, ClosedError, ConfigError, NavTaskError
from nav_tasks.domain.tasks import (
    EventKind, NavOutcome, NavStatus, PreemptPolicy, TaskEvent, TaskRequest,
    TaskSnapshot, TaskState,
)
from nav_tasks.domain.types import CapturedPose, PoseRecord, StoredPoi, Transform2D


class TaskApi(Protocol):
    """Loop-confined motion ownership API; administrative commands are separate."""

    async def submit(self, task: TaskRequest) -> TaskSnapshot:
        """Admit a task without waiting for its motion result.

        Args:
            task: Domain request and ingress preemption policy.

        Returns:
            Accepted ownership snapshot.
        """
        ...

    async def cancel(self, task_id: str | None = None) -> bool:
        """Wait for cancellation of a matching or current task.

        Args:
            task_id: Matching ID, or None for current ownership.

        Returns:
            Whether a matching task was canceled.
        """
        ...

    def status(self) -> TaskSnapshot:
        """Read current motion ownership.

        Returns:
            Immutable state snapshot.
        """
        ...

    def subscribe_events(self) -> asyncio.Queue[TaskEvent]:
        """Subscribe to subsequent task events on this event loop.

        Returns:
            Queue that the subscriber must drain and detach when finished.
        """
        ...

    def unsubscribe_events(self, queue: asyncio.Queue[TaskEvent]) -> None:
        """Detach an event consumer.

        Args:
            queue: Previously subscribed queue.
        """
        ...

    async def close(self) -> None:
        """Seal admission and wait for cleanup of owned motion."""
        ...


class PoiApi(Protocol):
    """Administrative POI operations, separate from exclusive motion ownership."""

    async def mark(self, poi_id: str, name: str) -> MarkPoiResult:
        """Capture and persist a named POI without preempting navigation.

        Args:
            poi_id: Decoded POI identifier.
            name: Decoded display name.

        Returns:
            Stored POI or a readable capture/freshness failure.
        """
        ...

    def update_directory(self, id_to_name: Iterable[tuple[str, str]]) -> None:
        """Apply directory names without creating unmeasured poses.

        Args:
            id_to_name: Decoded identifier/name pairs in input order.
        """
        ...


__all__ = [
    'BusyError', 'CapturedPose', 'ClosedError', 'CommandRegistry', 'ConfigError',
    'EventKind', 'MarkPoiResult', 'NavOutcome', 'NavStatus', 'NavTaskError', 'PoiApi',
    'PoseRecord', 'PreemptPolicy', 'StoredPoi', 'TaskApi', 'TaskContext', 'TaskEvent',
    'TaskManager', 'TaskRequest', 'TaskSnapshot', 'TaskState', 'Transform2D',
]
