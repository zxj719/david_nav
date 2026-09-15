"""Public task and value contracts shared with in-process ingress adapters.

Transport code calls only TaskApi. The assembly root can construct TaskManager
and register skills through these exports without importing internal modules.
This API does not start ROS or a transport and does not change legacy entry points.
"""

import asyncio
from typing import Protocol

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


__all__ = [
    'BusyError', 'CapturedPose', 'ClosedError', 'CommandRegistry', 'ConfigError',
    'EventKind', 'NavOutcome', 'NavStatus', 'NavTaskError', 'PoseRecord',
    'PreemptPolicy', 'StoredPoi', 'TaskApi', 'TaskContext', 'TaskEvent', 'TaskManager',
    'TaskRequest', 'TaskSnapshot', 'TaskState', 'Transform2D',
]
