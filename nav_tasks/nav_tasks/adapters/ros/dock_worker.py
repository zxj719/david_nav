"""持有同步 Dock 工作线程，取消或替换完成前等待其实际退出."""

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
    """表示工作线程在位姿读取、等待或阶段边界观察到协作停止请求."""


@dataclass
class _WorkerAttempt:
    """保存一次 Dock 工作线程及异步收尾的可变所有权记录.

    Attributes:
        stop: 后续阶段与工作线程边界共享的停止事件。
        motion: 本线程的阻塞端口，动作仍交由循环内共享后端执行。
        thread: 非守护工作线程，可能尚未启动或启动失败。
        worker: 等待结果并确认线程退出的受监督任务。
        cleanup: 可共享、失败后可重试的后台清理任务。
        settled: 已确认软件清理结束，不是额外的物理停稳证明。
    """

    stop: threading.Event
    motion: _DockMotion
    thread: threading.Thread
    worker: asyncio.Task | None = None
    cleanup: asyncio.Task | None = None
    settled: bool = False


class DockWorker:
    """在受管线程运行同步 Dock 策略，不把后端动作所有权交给该线程.

    构造和生命周期调用属于 TaskManager 的 asyncio 循环；共享 Nav2Client
    留在同一循环，不另建 ROS executor 或客户端。位姿、地图、日志及观察回调
    必须适于工作线程。execute 等待者反复取消不能放弃线程清理；阻塞回调
    无法被强制停止，必须保留所有权和所需资源直到真实线程退出。
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
        """绑定显式 Dock 策略与工作线程端口，构造时不启动线程.

        这里只校验帧真值及两个接收预算，不替代完整 workflow_config 校验；
        也不验证回调的线程安全或目标几何有效性。

        Args:
            backend: 与普通导航及旋转共享的唯一 Nav2 客户端。
            global_frame: 解释平面目标的配置帧，不进行 TF 转换。
            config: 显式工作流策略，包含阶段、补偿与停稳预算。
            pose_fn: 可在工作线程读取导航旋转中心位姿的回调。
            approach_check: 可在工作线程执行完整轮廓静态图检查的回调。
            logger: 可在工作线程使用的冻结日志接收器。
            server_timeout_sec: 有限非负就绪预算，拒绝布尔值。
            send_timeout_sec: 有限非负接收预算，拒绝布尔值。
            now_fn: 可选单调时钟，用于确定性的入位测量测试。
            sleep_fn: 可选测试等待器；默认以停止事件执行可中断等待。
            on_raw_feedback: 可选完整反馈观察器，附带任务和代次身份。
            on_event: 可选工作线程同步观察器，附带任务身份。

        Raises:
            ConfigError: 空帧或已检查的接收预算无效。
            RuntimeError: 没有运行中的 asyncio 循环。
            Exception: 未归一化的转换异常，例如超大整数转浮点溢出。
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
        """读取是否仍持有工作线程或未解决清理，不等同于实体运动状态."""
        return self._active is not None

    async def dock(
        self, goal: DockGoal, on_feedback: Callable[[float], None],
    ) -> NavOutcome:
        """适配领域 Dock 端口，并为 ROS 入口保留完整动作反馈.

        只从目标取 x/y/yaw，按本实例的配置帧解释，不转换 goal.pose.frame_id。
        普通反馈先加阶段一的后续距离，再调用可选完整反馈观察器。

        Args:
            goal: 带位姿和任务、子任务、代次身份的目标。
            on_feedback: 循环线程中的剩余米数回调，含后续阶段距离。

        Returns:
            整个工作流的测量结论，不把预备点到达当作 Dock 成功。

        Raises:
            asyncio.CancelledError: 显式取消并等待持有的清理后传播。
            Exception: 工作线程或动作清理失败，未解决的所有权继续保留。
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
        """运行一次阻塞工作流，停止屏障一旦设置就不再返回成功判定.

        以非守护线程执行，结果 Future 完成后还需确认线程退出。调用方取消时
        反复屏蔽等待者取消以加入同一清理；清理自身取消或失败仍向外传播。
        线程启动、工作流或清理异常不能作为安全释放所有权的证据。

        Args:
            goal: 已解码的平面目标，刻意不执行普通 POI 吸附。
            on_feedback: 仅循环线程调用的完整反馈与后续距离回调。
            on_event: 可选同步工作线程观察器，受当前尝试身份与停止屏障限制。

        Returns:
            完整工作流结论；显式停止开始后不返回成功。

        Raises:
            BusyError: 旧工作线程或未完成清理仍占有准入。
            ClosedError: 生命周期准入已经封闭。
            asyncio.CancelledError: 调用方取消并等待清理，或清理任务自身取消。
            Exception: 启动、工作流或清理失败；保留所有者并重试 cancel/close。
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
        """封闭当前尝试的后续动作，并等待后端清理和真实工作线程退出.

        不封闭未来尝试准入。等待者取消不取消后台清理，但此方法自身不反复
        屏蔽等待者取消；execute 的取消路径另有持续加入清理的屏障。

        Raises:
            asyncio.CancelledError: 等待者或后台清理取消，资源不能据此销毁。
            Exception: 跨循环调用或清理失败，未完成所有权保留供重试。
        """
        self._check_loop()
        if self._active is not None:
            await asyncio.shield(self._cleanup_task(self._active))

    async def close(self) -> None:
        """永久封闭工作线程准入并等待清理，不销毁共享 Nav2 后端.

        失败后仍保持封闭；调用方须保留循环、ROS 回调及工作线程端口资源。

        Raises:
            asyncio.CancelledError: 等待者或后台清理取消。
            Exception: 跨循环调用或清理失败，需保留所有者重试。
        """
        self._check_loop()
        self._closed = True
        await self.cancel()

    def _check_loop(self):
        if asyncio.get_running_loop() is not self._loop:
            raise RuntimeError('Dock worker used from a different event loop')

    def _cleanup_task(self, attempt):
        """先封闭新运动阶段，再创建或复用本次工作线程的清理任务.

        已收尾时不再发送取消请求；失败或已结束的清理任务可重新创建。
        返回任务由 supervisor 持有，调用方应通过 shield 等待。

        Args:
            attempt: 持有运动端口、工作线程和清理状态的尝试。

        Returns:
            清理任务；已收尾且从未创建清理任务时可为 None。
        """
        if attempt.settled:
            return attempt.cleanup
        attempt.motion.request_cancel()
        if attempt.cleanup is None or attempt.cleanup.done():
            attempt.cleanup = self._supervisor.spawn(self._cleanup(attempt), 'dock-worker-cleanup')
        return attempt.cleanup

    async def _cleanup(self, attempt):
        """确认后端取消并等工作任务结束，随后才释放本次 Dock 所有权.

        后端取消失败时不继续释放；工作任务的异常已由 execute 或 supervisor
        接管，gather 只收集结果。外层取消屏障阻止等待期间进入新的运动阶段。

        Args:
            attempt: 已设置运动取消屏障、仍由本对象持有的尝试。

        Raises:
            asyncio.CancelledError: 清理协程被取消，收尾标记尚未写入。
            Exception: 后端取消失败，保留所有权供后续重试。
        """
        await self._backend.cancel()
        # The cancellation fence prevents another stage while we join. Exceptions
        # already belong to execute/supervisor; confirmed cleanup can release them.
        await asyncio.gather(asyncio.shield(attempt.worker), return_exceptions=True)
        attempt.settled = True
        if self._active is attempt:
            self._active = None

    async def _wait_thread(self, thread, completion):
        """等待跨线程结果，并在退出等待时确认工作线程实际结束.

        Future 完成可能早于线程函数返回；finally 轮询存活状态后才做非阻塞 join。
        本辅助函数自身不屏蔽重复取消，持续等待由外层所有权流程保证。

        Args:
            thread: 本次尝试拥有的线程，可能因启动失败而从未启动。
            completion: 线程写入结果或异常的跨线程 Future。

        Returns:
            工作线程写入 completion 的结果。

        Raises:
            asyncio.CancelledError: 结果等待或线程退出轮询被取消。
            Exception: 工作线程报告失败，在 finally 尝试等待线程退出后传播。
        """
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
