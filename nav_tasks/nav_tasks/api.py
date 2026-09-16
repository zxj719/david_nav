"""Public task and value contracts shared with in-process ingress adapters.

Transport code calls only TaskApi/PoiApi. The assembly root can construct TaskManager
and register skills through these exports without importing internal modules.
This API does not start ROS or a transport and does not change legacy entry points.
"""

import asyncio
import time
from typing import Callable, Iterable, Mapping, Protocol

from nav_tasks.core.handlers.poi_commands import MarkPoiResult
from nav_tasks.core.ports import PoseSource
from nav_tasks.core.registry import CommandRegistry, TaskContext
from nav_tasks.core.state import TaskManager
from nav_tasks.domain.errors import (
    BusyError, ClosedError, ConfigError, NavTaskError, PoseUnavailableError, RuntimeAssemblyError,
)
from nav_tasks.domain.runtime_config import DockNodeConfig, DockRuntimeConfig, RuntimeConfig
from nav_tasks.domain.tasks import (
    DockGoal, EventKind, NavOutcome, NavStatus, PreemptPolicy, TaskCompletion, TaskEvent,
    TaskRequest, TaskSnapshot, TaskState,
)
from nav_tasks.domain.types import CapturedPose, PoseRecord, StoredPoi, Transform2D


class TaskApi(Protocol):
    """Loop-confined motion ownership API; administrative commands are separate."""

    async def submit(
        self, task: TaskRequest, *, on_settled: Callable[[TaskCompletion], None] | None = None,
    ) -> TaskSnapshot:
        """Admit a task without waiting for its motion result.

        Args:
            task: Domain request and ingress preemption policy.
            on_settled: Optional loop-thread receipt after task cleanup/ownership release.

        Returns:
            Accepted ownership snapshot.
        """
        ...

    async def cancel(self, task_id: str | None = None, *, generation: int | None = None) -> bool:
        """Wait for cancellation of a matching or current task.

        Args:
            task_id: Matching ID, or None for current ownership.
            generation: Optional admission generation, checked atomically with the ID.

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


class DockIngressApi(Protocol):
    """Empty ROS ingress owner; callbacks/cleanup require a live multithreaded host."""

    def open(self, node: object) -> None:
        """Create ingress resources; retain this owner if construction fails.

        Args:
            node: Caller-owned node, destroyed only after ingress teardown.
        """
        ...

    def stop(self) -> None:
        """Seal admission on the task loop without stopping ROS callbacks."""
        ...

    async def close(self) -> None:
        """Wait for owned task cleanup, keeping ROS alive and resources retained."""
        ...

    def destroy(self) -> None:
        """Release retained ROS resources after close and host executor quiescence."""
        ...

    def on_feedback(self, goal: DockGoal, raw: object, extra_distance: float) -> None:
        """Forward complete loop-thread Dock feedback for a matching live action only.

        Args:
            goal: Correlated Dock owner identity and generation.
            raw: Unmodified original ROS navigation feedback wrapper.
            extra_distance: Remaining later-stage distance.
        """
        ...


def create_dock_ingress(
    tasks: TaskApi, config: DockNodeConfig, *, cleanup_retry_sec: float = 1.0,
) -> DockIngressApi:
    """Create a no-I/O ingress owner on the task loop before opening its ROS resources.

    The caller must keep a MultiThreadedExecutor with at least two threads alive
    through close, then quiesce it before destroy. No new motion client is created.

    Args:
        tasks: Public shared task manager on the current running loop.
        config: Explicit Dock ingress names and runtime policy.
        cleanup_retry_sec: Delay between failed task cleanup attempts.

    Returns:
        Empty owner to retain across open failures and retryable teardown.

    Raises:
        ConfigError: Configuration or cleanup retry interval is invalid.
    """
    from nav_tasks.adapters.ros.dock_ingress import DockIngress

    return DockIngress(tasks, config, cleanup_retry_sec=cleanup_retry_sec)


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


class TaskRuntimeApi(Protocol):
    """In-process assembled task services; ROS node/spin and ingress remain external."""

    tasks: TaskApi
    pois: PoiApi
    poses: PoseSource

    def start_loop_closure(self) -> None:
        """Start or reuse the supervised connection-lifetime POI watcher."""
        ...

    async def stop_loop_closure(self) -> None:
        """Cancel/join the watcher without canceling motion."""
        ...

    async def close(self) -> None:
        """Settle motion then release owned resources; failure remains retryable."""
        ...


def _runtime_log(message: str) -> None:
    print(f'[navigation_executor] {message}', flush=True)


def dock_config_from_parameters(
    parameters: Mapping[str, object], *, spin_action_name: str = 'spin',
    pose_timeout_sec: float = 2.0,
) -> DockNodeConfig:
    """Translate complete old Dock ROS parameters without opening any runtime resource.

    Args:
        parameters: All sixteen legacy names and values; footprint remains a string.
        spin_action_name: Shared Spin client endpoint, retaining its old relative name.
        pose_timeout_sec: TF read budget, retaining the old two-second default.

    Returns:
        Validated immutable ingress and task configuration.

    Raises:
        ConfigError: Missing, unknown or invalid parameters.
    """
    from nav_tasks.app.dock_parameters import from_parameters

    return from_parameters(parameters, spin_action_name=spin_action_name,
                           pose_timeout_sec=pose_timeout_sec)


def declare_dock_parameters(
    node: object, defaults: Mapping[str, object], *, spin_action_name: str = 'spin',
    pose_timeout_sec: float = 2.0,
) -> DockNodeConfig:
    """Declare the unchanged Dock parameter surface and apply ROS overrides lazily.

    Args:
        node: Caller-owned ROS node, retained by the caller if declaration fails.
        defaults: Complete old parameter mapping, validated before any declaration.
        spin_action_name: Shared backend endpoint, not an added ROS parameter.
        pose_timeout_sec: TF read budget, not an added ROS parameter.

    Returns:
        Validated effective immutable settings after ROS parameter overrides.

    Raises:
        ConfigError: Defaults or effective values are invalid.
        Exception: ROS declaration/read failed; the caller must clean its node.
    """
    from nav_tasks.adapters.ros.dock_parameters import declare_parameters

    return declare_parameters(node, defaults, spin_action_name=spin_action_name,
                              pose_timeout_sec=pose_timeout_sec)


async def create_runtime(
    config: RuntimeConfig, node: object, tf_buffer: object, *,
    log: Callable[[str], None] = _runtime_log, now: Callable[[], float] = time.time,
    on_dock_feedback: Callable[[DockGoal, object, float], None] | None = None,
) -> TaskRuntimeApi:
    """Lazily assemble task services without importing implementation through ingress.

    Importing this public API remains free of ROS, YAML and transport imports.
    The caller owns node/listener/spin; keep them alive until runtime.close succeeds.
    Dock IDs require explicit Dock configuration, never ordinary-navigation fallback.

    Args:
        config: Explicit immutable robot/mode runtime values.
        node: Externally created ROS node.
        tf_buffer: Externally owned TF buffer.
        log: Unprefixed startup/POI log sink.
        now: Pose-compatible timestamp reader for freshness checks.
        on_dock_feedback: Optional loop-thread complete ROS feedback and distance offset.

    Returns:
        Public task/admin/observation and resource-lifetime handle.

    Raises:
        ConfigError: Configuration cannot be safely assembled.
        RuntimeAssemblyError: Retain this exception and await its cleanup before node teardown.
        Exception: Resource construction or rollback failed; no task has been admitted.
    """
    from nav_tasks.app.runtime import assemble_runtime

    return await assemble_runtime(config, node, tf_buffer, log=log, now=now,
                                  on_dock_feedback=on_dock_feedback)


__all__ = [
    'BusyError', 'CapturedPose', 'ClosedError', 'CommandRegistry', 'ConfigError',
    'DockGoal', 'DockIngressApi', 'DockNodeConfig', 'DockRuntimeConfig', 'EventKind',
    'MarkPoiResult',
    'NavOutcome', 'NavStatus', 'NavTaskError', 'PoiApi',
    'PoseRecord', 'PoseUnavailableError', 'PreemptPolicy', 'StoredPoi', 'TaskApi',
    'TaskCompletion', 'TaskContext', 'TaskEvent',
    'TaskManager', 'TaskRequest', 'TaskSnapshot', 'TaskState', 'Transform2D',
    'PoseSource', 'RuntimeAssemblyError', 'RuntimeConfig', 'TaskRuntimeApi', 'create_runtime',
    'declare_dock_parameters', 'dock_config_from_parameters',
    'create_dock_ingress',
]
