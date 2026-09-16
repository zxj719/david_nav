"""Task-service composition and resource ownership, without ROS spin or transport."""

import asyncio
from contextlib import suppress
import inspect
import logging
from pathlib import Path
from typing import Callable

from nav_tasks.adapters.ros.grid_source import RosGridSource
from nav_tasks.adapters.ros.tf_pose_source import TfPoseSource
from nav_tasks.app.logging import RetryAudit
from nav_tasks.app.loop_closure import watch_loop_closure
from nav_tasks.core.goal_resolution import GridGoalResolver, SnapSearch
from nav_tasks.core.handlers.navigate_to import NavigateToPoi
from nav_tasks.core.handlers.poi_commands import PoiCommands
from nav_tasks.core.registry import CommandRegistry
from nav_tasks.core.retry import RetryPolicy
from nav_tasks.core.state import TaskManager
from nav_tasks.core.supervisor import TaskSupervisor
from nav_tasks.domain.errors import ClosedError, ConfigError, RuntimeAssemblyError
from nav_tasks.domain.runtime_config import RuntimeConfig
from nav_tasks.nav2_client import Nav2Client
from nav_tasks.nav_site_data.named_poses_importer import import_named_poses
from nav_tasks.nav_site_data.yaml_poi_repository import YamlPoiRepository

_LOGGER = logging.getLogger(__name__)


class _Rollback:
    def __init__(self):
        self._callbacks: list[Callable[[], object]] = []
        self._lock = asyncio.Lock()

    async def __aenter__(self):
        return self

    async def __aexit__(self, error_type, error, traceback):
        if error is None:
            self._callbacks.clear()
            return
        if isinstance(error, RuntimeAssemblyError):
            self.track(error.cleanup)
        try:
            await self.cleanup()
        except BaseException:
            raise RuntimeAssemblyError(error, self.cleanup) from error

    def track(self, callback):
        self._callbacks.insert(0, callback)

    async def cleanup(self):
        async with self._lock:
            failure = None
            for callback in tuple(self._callbacks):
                try:
                    result = callback()
                    if inspect.isawaitable(result):
                        await result
                except BaseException as error:
                    failure = error
                else:
                    self._callbacks.remove(callback)
            if failure is not None:
                raise failure


class TaskRuntime:
    """Own assembled task resources while the caller owns ROS and ingress lifetimes.

    No motion or observer starts on construction. Public consumers should use
    the TaskRuntimeApi contract returned by nav_tasks.api.create_runtime.
    """

    def __init__(
        self, tasks: TaskManager, pois: PoiCommands, poses: TfPoseSource,
        grids: RosGridSource, backend: Nav2Client, repository: YamlPoiRepository,
        config: RuntimeConfig,
    ) -> None:
        """Take ownership of fully constructed resources on their asyncio loop.

        Args:
            tasks: Sole task manager sharing the backend.
            pois: Administrative POI API using the same pose/storage ports.
            poses: Owned navigation observation adapter.
            grids: Owned grid observation adapter.
            backend: Owned single-attempt action backend.
            repository: Shared persistence port.
            config: Immutable assembly values.
        """
        self.tasks = tasks
        self.pois = pois
        self.poses = poses
        self._grids = grids
        self._backend = backend
        self._repository = repository
        self._config = config
        self._loop = asyncio.get_running_loop()
        self._supervisor = TaskSupervisor(self._on_crash)
        self._watcher: asyncio.Task | None = None
        self._close_task: asyncio.Task | None = None
        self._closing = False
        self._closed = False

    def start_loop_closure(self) -> None:
        """Start one supervised watcher, for the caller's chosen connection lifetime.

        Repeated start while running is a no-op. The old WS lifecycle can call
        stop on disconnect and start on reconnect; no policy is changed here.

        Raises:
            ClosedError: Runtime shutdown has started.
        """
        self._check_loop()
        if self._closing:
            raise ClosedError('task runtime is closing or closed')
        if self._watcher is None or self._watcher.done():
            self._watcher = self._supervisor.spawn(watch_loop_closure(
                self.poses, self._repository, self._config.loop_closure_poll_sec), 'loop-closure')

    async def stop_loop_closure(self) -> None:
        """Cancel/join the watcher without canceling motion or discarding its crash log.

        The supervisor already retrieves and reports watcher errors. They must
        not prevent subsequent motion cleanup during final shutdown.
        """
        self._check_loop()
        task = self._watcher
        if task is not None:
            task.cancel()
            with suppress(asyncio.CancelledError, Exception):
                await task
            if self._watcher is task:
                self._watcher = None

    async def close(self) -> None:
        """Join retryable resource teardown without losing it on caller cancellation.

        Stop ingress before calling. Keep the externally owned ROS executor alive
        until this completes so action cancellation can receive its terminal result.
        On failure, retain this handle and call close again; do not destroy its
        ROS node or stop spinning merely because a cleanup deadline expired.

        Raises:
            Exception: Motion or subscription cleanup failed; resources remain owned.
        """
        self._check_loop()
        self._closing = True
        if self._close_task is None or (self._close_task.done() and not self._closed):
            self._close_task = self._supervisor.spawn(self._close(), 'close-task-runtime')
        await asyncio.shield(self._close_task)

    async def _close(self) -> None:
        await self.stop_loop_closure()
        await self.tasks.close()
        await self._backend.close()
        self._grids.close()
        self.poses.close()
        self._closed = True

    def _check_loop(self) -> None:
        if asyncio.get_running_loop() is not self._loop:
            raise RuntimeError('task runtime used from a different event loop')

    @staticmethod
    def _on_crash(name: str, error: BaseException) -> None:
        _LOGGER.error('Task runtime operation %s failed: %s', name, error)


async def assemble_runtime(
    config: RuntimeConfig, node: object, tf_buffer: object, *,
    log: Callable[[str], None], now: Callable[[], float],
) -> TaskRuntime:
    """Construct ordinary POI services with rollback before exposing task admission.

    No ROS initialization, node/listener/executor creation, spin, transport,
    observer or action goal is started here. During partial-construction failure,
    rollback attempts all completed resources in reverse order. If it fails, retain
    RuntimeAssemblyError and await its cleanup successfully before node destruction.
    No task was exposed or run, but action clients may still own ROS waitables.

    Args:
        config: Explicit robot/mode values; Dock integration remains pending.
        node: Caller-owned ROS node, kept alive through close success.
        tf_buffer: Caller-owned TF buffer/listener connection.
        log: Unprefixed runtime log sink for POI loading.
        now: Pose-compatible timestamp clock for administrative freshness checks.

    Returns:
        Owned resources exposing TaskApi, PoiApi and PoseSource.

    Raises:
        ConfigError: A not-yet-supported Dock mode or invalid retry policy is requested.
        RuntimeAssemblyError: Rollback failed; the exception retains retryable cleanup.
        Exception: Repository loading or resource construction/rollback failed.
    """
    if config.dock_poi_ids:
        raise ConfigError('Dock runtime assembly is not integrated yet; refusing ordinary routing')
    retry = RetryPolicy(config.nav_retry_count, config.nav_retry_delay_sec)
    search = SnapSearch(
        config.nav_goal_snap_radius, config.nav_goal_fallback_snap_radius,
        config.nav_goal_clearance_radius, config.nav_goal_max_snap_radius,
        config.nav_goal_snap_step)
    repository = YamlPoiRepository(
        config.poi_store_file, config.source_poi_file, map_frame=config.map_frame)
    repository.load()
    if config.pose_file is not None:
        path = Path(config.pose_file)
        if not path.exists():
            log(f'pose file not found: {config.pose_file}')
        else:
            count = import_named_poses(
                repository, path, map_frame=config.map_frame,
                odom_frame=config.odom_frame, log=log)
            log(f'loaded {count} POIs from {config.pose_file}')
    async with _Rollback() as rollback:
        poses = TfPoseSource(
            node, tf_buffer, robot_base_frame=config.robot_base_frame,
            odom_topic=config.odom_topic, map_frame=config.map_frame, odom_frame=config.odom_frame,
            session_id=lambda: config.slam_session_id)
        rollback.track(poses.close)
        grids = RosGridSource(
            node, map_topic=config.grid_map_topic, costmap_topic=config.global_costmap_topic)
        rollback.track(grids.close)
        backend = Nav2Client(
            node, config.action_name, config.server_timeout_sec,
            cancel_timeout_sec=config.cancel_timeout_sec)
        rollback.track(backend.close)
        resolver = GridGoalResolver(grids.snapshot, poses, search)
        skill = NavigateToPoi(
            backend, repository, resolver, retry=retry,
            progress_min_interval_sec=config.progress_min_interval_sec, on_failure=RetryAudit())
        registry = CommandRegistry()
        registry.register('navigate_to', skill.decode, skill.run)
        tasks = TaskManager(registry, backend.cancel)
        rollback.track(tasks.close)
        commands = PoiCommands(poses, repository, now, config.pose_stale_after_sec)
        runtime = TaskRuntime(tasks, commands, poses, grids, backend, repository, config)
        return runtime
