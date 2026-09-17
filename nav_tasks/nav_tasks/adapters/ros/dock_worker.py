"""Owned blocking Dock thread, joined before task cancellation or replacement completes."""

import asyncio
from concurrent.futures import Future
from dataclasses import dataclass
import logging
import math
import threading
from typing import Callable, Mapping

from nav_tasks.adapters.ros.dock_motion import _DockMotion
from nav_tasks.adapters.ros.futures import await_rclpy_future
from nav_tasks.core.supervisor import TaskSupervisor
from nav_tasks.domain.errors import BusyError, ClosedError, ConfigError
from nav_tasks.domain.footprint import FootprintCheck
from nav_tasks.domain.tasks import DockGoal, NavOutcome, NavStatus
from nav_tasks.nav2_client import Nav2Client
from nav_tasks.skills.docking.ports import DockLogger, PlanarPose
from nav_tasks.skills.docking.workflow import DockConfig, DockWorkflow

_LOGGER = logging.getLogger(__name__)


class _WorkerStopped(Exception):
    """Cooperative cancellation at a worker's pose, sleep or admission boundary."""


@dataclass
class _WorkerAttempt:
    stop: threading.Event
    motion: _DockMotion
    thread: threading.Thread
    worker: asyncio.Task | None = None
    cleanup: asyncio.Task | None = None
    settled: bool = False


class DockWorker:
    """Run synchronous Dock policy without granting its thread motion ownership.

    Construct and call lifecycle methods on the TaskManager event loop. The
    shared Nav2Client stays on that loop; no second ROS executor or client is
    constructed here. Pose/map/logger callbacks must be worker-safe. Canceling
    an asyncio caller requests cooperative thread cancellation and joins owned
    cleanup, including repeated cancellation. A stuck callback cannot be forcibly
    stopped: ownership stays held until it returns and the thread really exits.
    """

    def __init__(
        self, backend: Nav2Client, global_frame: str, config: DockConfig,
        pose_fn: Callable[[], PlanarPose | None],
        approach_check: Callable[[PlanarPose], FootprintCheck], logger: DockLogger, *,
        server_timeout_sec: float = 5.0, send_timeout_sec: float = 10.0,
        now_fn: Callable[[], float] | None = None,
        sleep_fn: Callable[[float], None] | None = None,
        on_raw_feedback: Callable[[DockGoal, object, float], None] | None = None,
        on_event: Callable[[DockGoal, str, Mapping[str, object]], None] | None = None,
    ) -> None:
        """Bind explicit Dock policy and worker-safe ports without starting any thread.

        Args:
            backend: Sole Nav2 client shared with ordinary navigation and Spin.
            global_frame: Configured target frame; no robot literal is supplied here.
            config: Explicit workflow policy, including stage and settle budgets.
            pose_fn: Worker-safe navigation-base pose source.
            approach_check: Worker-safe full-footprint static-map admission check.
            logger: Worker-safe frozen log sink.
            server_timeout_sec: Legacy readiness budget, independently injectable in tests.
            send_timeout_sec: Legacy admission budget, independently injectable in tests.
            now_fn: Optional deterministic monotonic clock for finish tests.
            sleep_fn: Optional test sleeper; production uses interruptible Event.wait.
            on_raw_feedback: Optional complete feedback observer with task/generation identity.
            on_event: Optional worker-thread observation sink with immutable task identity.

        Raises:
            ConfigError: A frame or admission budget is invalid.
        """
        if not global_frame:
            raise ConfigError('Dock global_frame must be nonempty')
        for value in (server_timeout_sec, send_timeout_sec):
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                raise ConfigError('Dock admission budgets must be finite and nonnegative')
        self._loop = asyncio.get_running_loop()
        self._backend = backend
        self._frame = global_frame
        self._config = config
        self._pose = pose_fn
        self._check = approach_check
        self._logger = logger
        self._server_timeout = server_timeout_sec
        self._send_timeout = send_timeout_sec
        self._now = now_fn
        self._sleep = sleep_fn
        self._raw_feedback = on_raw_feedback
        self._on_event = on_event
        self._supervisor = TaskSupervisor(self._on_crash)
        self._active: _WorkerAttempt | None = None
        self._closed = False

    @property
    def busy(self) -> bool:
        """Whether a worker or failed/unresolved cleanup still owns this facade."""
        return self._active is not None

    async def dock(
        self, goal: DockGoal, on_feedback: Callable[[float], None],
    ) -> NavOutcome:
        """Implement the pure Dock port while retaining full feedback for ROS ingress.

        Args:
            goal: Immutable pose and owner identity; coordinates use configured Dock frame.
            on_feedback: Loop-thread distance callback, including the stage-one offset.

        Returns:
            Complete measured Dock outcome, never intermediate approach arrival.

        Raises:
            Exception: Worker or motion cleanup failed and remains owned.
            asyncio.CancelledError: Explicit cancellation after owned cleanup.
        """
        def feedback(raw, extra):
            on_feedback(float(raw.feedback.distance_remaining) + float(extra))
            if self._raw_feedback is not None:
                self._raw_feedback(goal, raw, extra)

        pose = goal.pose
        observe = None if self._on_event is None else lambda name, data: self._on_event(
            goal, name, data)
        return await self.execute((pose.x, pose.y, pose.yaw), feedback, on_event=observe)

    async def execute(
        self, goal: PlanarPose, on_feedback: Callable[[object, float], None] | None = None,
        *, on_event: Callable[[str, Mapping[str, object]], None] | None = None,
    ) -> NavOutcome:
        """Run one blocking workflow and fence any verdict after cancellation starts.

        Args:
            goal: Already-decoded planar target; Dock deliberately bypasses snapping.
            on_feedback: Loop-only complete raw feedback plus approach-distance offset.
            on_event: Optional synchronous worker-thread observation sink, fenced by this attempt.

        Returns:
            Workflow verdict, never success after explicit cancellation has begun.

        Raises:
            BusyError: A prior worker or unresolved cleanup still owns admission.
            ClosedError: Worker lifecycle admission is sealed.
            asyncio.CancelledError: Caller canceled after waiting for owned cleanup.
            Exception: Worker or cleanup failed; retain the owner and retry cancel/close.
        """
        self._check_loop()
        if self._closed:
            raise ClosedError('Dock worker is closed')
        if self._active is not None:
            raise BusyError('Dock worker or its cleanup is still active')
        stop = threading.Event()
        motion = _DockMotion(self._backend, self._frame, stop, self._supervisor,
                             self._logger, on_feedback, self._server_timeout, self._send_timeout)
        completion = Future()

        def observe(name, data):
            if on_event is not None and self._active is attempt and not stop.is_set():
                on_event(name, data)

        thread = threading.Thread(
            target=self._run, args=(completion, stop, motion, goal, observe),
            name='nav-dock-worker', daemon=False)
        attempt = _WorkerAttempt(stop, motion, thread)
        self._active = attempt
        attempt.worker = self._supervisor.spawn(
            self._wait_thread(thread, completion), 'dock-worker-join')
        try:
            thread.start()
        except Exception as error:
            completion.set_exception(error)
            raise
        try:
            result = await asyncio.shield(attempt.worker)
            if stop.is_set():
                await asyncio.shield(self._cleanup_task(attempt))
                return NavOutcome(NavStatus.CANCELED)
            attempt.settled = True
            if self._active is attempt:
                self._active = None
            return NavOutcome(NavStatus(result))
        except asyncio.CancelledError:
            cleanup = self._cleanup_task(attempt)
            while True:
                try:
                    await asyncio.shield(cleanup)
                    break
                except asyncio.CancelledError:
                    if cleanup.cancelled():
                        raise
                    # Repeated caller cancellation cannot abandon an owned thread.
            raise

    async def cancel(self) -> None:
        """Fence actions and join the actual worker plus backend cleanup.

        Raises:
            Exception: Cleanup failed; ownership remains retained for retry.
        """
        self._check_loop()
        if self._active is not None:
            await asyncio.shield(self._cleanup_task(self._active))

    async def close(self) -> None:
        """Seal worker admission and settle it without destroying the shared backend.

        Raises:
            Exception: Cleanup failed; the runtime must retain resources and retry.
        """
        self._check_loop()
        self._closed = True
        await self.cancel()

    def _check_loop(self):
        if asyncio.get_running_loop() is not self._loop:
            raise RuntimeError('Dock worker used from a different event loop')

    def _cleanup_task(self, attempt):
        if attempt.settled:
            return attempt.cleanup
        attempt.motion.request_cancel()
        if attempt.cleanup is None or attempt.cleanup.done():
            attempt.cleanup = self._supervisor.spawn(self._cleanup(attempt), 'dock-worker-cleanup')
        return attempt.cleanup

    async def _cleanup(self, attempt):
        await self._backend.cancel()
        # The cancellation fence prevents another stage while we join. Exceptions
        # already belong to execute/supervisor; confirmed cleanup can release them.
        await asyncio.gather(asyncio.shield(attempt.worker), return_exceptions=True)
        attempt.settled = True
        if self._active is attempt:
            self._active = None

    async def _wait_thread(self, thread, completion):
        try:
            return await await_rclpy_future(completion)
        finally:
            # Future completion precedes the thread's final return by a few
            # instructions. Do not confuse it with actual thread termination.
            while thread.is_alive():
                await asyncio.sleep(0.001)
            if thread.ident is not None:
                thread.join(timeout=0)

    def _run(self, completion, stop, motion, goal, observe):
        """在线程执行同步 Dock 流程，在位姿、地图检查和等待前后检查停止标志.

        此处完成 Future 不等于线程已退出；外层 _wait_thread 仍负责确认并 join。
        流程异常写入 completion，不从线程悄悄丢失。

        Args:
            completion: 工作结果或异常的跨线程 Future，由外层持有。
            stop: 取消与关闭共享的线程事件。
            motion: 把动作调用交给唯一 asyncio 后端的阻塞端口。
            goal: 全局平面目标位姿（米、弧度）。
            observe: 在工作线程调用的观察回调；外层负责其线程安全。
        """
        def check_stop():
            if stop.is_set():
                raise _WorkerStopped

        def pose():
            check_stop()
            value = self._pose()
            check_stop()
            return value

        def approach(target):
            check_stop()
            value = self._check(target)
            check_stop()
            return value

        def sleep(seconds):
            check_stop()
            if self._sleep is None:
                stop.wait(seconds)
            else:
                self._sleep(seconds)
            check_stop()

        try:
            check_stop()
            workflow = DockWorkflow(self._config, motion, pose, approach, self._logger,
                                    now_fn=self._now, sleep_fn=sleep, on_event=observe)
            result = workflow.execute(goal)
            completion.set_result('canceled' if stop.is_set() else result)
        except _WorkerStopped:
            completion.set_result('canceled')
        except BaseException as error:
            completion.set_exception(error)

    @staticmethod
    def _on_crash(name: str, error: BaseException) -> None:
        _LOGGER.error('Owned %s failed; Dock cleanup must be resolved: %s', name, error)
