"""把阻塞 Dock 阶段请求交给唯一 Nav2 动作所有者循环的跨线程适配器."""

import asyncio
from concurrent.futures import Future
import logging
import threading
from typing import Callable

from nav_tasks.adapters.ros.action_session import ActionState
from nav_tasks.core.supervisor import TaskSupervisor
from nav_tasks.domain.errors import BusyError
from nav_tasks.domain.tasks import SpinGoal
from nav_tasks.domain.types import PoseRecord
from nav_tasks.nav2_client import Nav2Client
from nav_tasks.skills.docking.ports import DockActionResult, DockLogger, PlanarPose

_LOGGER = logging.getLogger(__name__)


class _DockMotion:
    """为单个 Dock 工作线程提供阻塞端口，后端动作始终在所有者循环执行."""

    def __init__(
        self, backend: Nav2Client, frame: str, stop: threading.Event,
        supervisor: TaskSupervisor, logger: DockLogger,
        feedback: Callable[[object, float], None] | None,
        server_timeout: float, send_timeout: float,
    ) -> None:
        """在所有者循环记录共享后端与线程桥接依赖，不启动动作或线程.

        保存参数而不重复验证；预算及端口适用性由外层工作线程所有者负责。

        Args:
            backend: 仅在此 asyncio 循环调用的共享 Nav2 后端。
            frame: 解释导航平面位姿的目标帧。
            stop: 工作线程与循环共享的协作停止事件。
            supervisor: 持有阶段任务强引用的循环监督器。
            logger: 冻结日志接收器。
            feedback: 可选循环线程原始反馈与后续距离回调。
            server_timeout: 服务就绪预算，单位为秒。
            send_timeout: 目标接收预算，单位为秒。

        Raises:
            RuntimeError: 当前没有运行中的 asyncio 循环。
        """
        self._loop = asyncio.get_running_loop()
        self._loop_thread = threading.get_ident()
        self._backend = backend
        self._frame = frame
        self._stop = stop
        self._supervisor = supervisor
        self._logger = logger
        self._feedback = feedback
        self._server_timeout = server_timeout
        self._send_timeout = send_timeout
        self._stage: asyncio.Task | None = None
        self._label: str | None = None
        self._cancel_logged = False

    def navigate(
        self, pose: PlanarPose, label: str, timeout_sec: float, extra_distance: float,
    ) -> DockActionResult:
        """阻塞工作线程等待共享导航阶段的软件收尾结果.

        此端口不查 POI 或吸附，也不驱动 ROS；跨线程等待没有独立期限，
        外层必须保持所有者循环与 ROS 回调存活。

        Args:
            pose: 配置全局帧下的平面目标，单位为米与弧度。
            label: 冻结的阶段日志标签。
            timeout_sec: 接收目标后的结果预算，不含独立清理预算。
            extra_distance: 导航反馈附加的后续阶段距离，单位为米。

        Returns:
            单次阶段结果，不等于整个 Dock 成功或实体停稳。

        Raises:
            RuntimeError: 从所有者循环线程调用会死锁，或循环已关闭。
            BaseException: 后端或清理异常经跨线程 Future 传播，不转换成普通失败。
        """
        return self._call('navigate', pose, label, timeout_sec, extra_distance)

    def spin(
        self, yaw: float, label: str, time_allowance_sec: float, timeout_sec: float,
    ) -> DockActionResult:
        """通过同一后端与动作槽位阻塞等待一次有符号旋转.

        Args:
            yaw: 相对旋转角，单位为弧度，不归一化或裁剪。
            label: 冻结的补偿阶段日志标签。
            time_allowance_sec: Nav2 服务端时间额度。
            timeout_sec: 独立的本地结果预算，不是跨线程等待总期限。

        Returns:
            阶段结果；不把旋转反馈误作剩余米数，不自行检查停稳。

        Raises:
            RuntimeError: 从所有者循环线程调用，或循环已关闭。
            BaseException: 目标验证、动作或清理异常经跨线程 Future 传播。
        """
        return self._call('spin', SpinGoal(yaw, time_allowance_sec), label, timeout_sec, 0)

    def request_cancel(self) -> None:
        """在所有者循环设置停止屏障，并请求取消尚未结束的本地阶段.

        即使取消审计日志抛出普通异常，finally 仍取消阶段。只发起请求，
        不等待后端终态或工作线程退出；线程约束由调用方遵守，不在此检查。
        """
        self._stop.set()
        stage = self._stage
        if stage is not None and not stage.done() and not self._cancel_logged:
            try:
                self._cancel_logged = True
                self._logger.warn('[dock] %s: 收到取消请求，取消当前目标' % self._label)
            except Exception:
                # Logging cannot be allowed to disable motion cancellation.
                _LOGGER.exception('Dock cancellation audit failed')
            finally:
                stage.cancel()

    def _call(self, kind, goal, label, timeout, extra):
        """从阻塞工作线程把动作请求交给唯一所有者循环，并等待跨线程结果.

        不 spin ROS，也不直接操作后端；answer 是本地跨线程 Future。
        此等待没有独立超时，外层必须保持所有者循环运行直到工作线程收尾。

        Args:
            kind: navigate 或 spin，选择后端阶段。
            goal: 平面位姿或 SpinGoal，交由循环内分发。
            label: 冻结的阶段日志标签。
            timeout: 传入后端的结果等待预算（秒），不是本地 Future 等待期限。
            extra: 导航反馈附加距离（米），旋转阶段不使用。

        Returns:
            循环内阶段转换出的 DockActionResult。

        Raises:
            RuntimeError: 在所有者循环线程调用会死锁，或循环已关闭。
            BaseException: 阶段异常经跨线程 Future 传回工作线程。
        """
        if threading.get_ident() == self._loop_thread:
            raise RuntimeError('blocking Dock motion called from the owner loop')
        answer = Future()
        self._loop.call_soon_threadsafe(self._dispatch, answer, kind, goal, label, timeout, extra)
        # This is our own cross-thread future, not a ROS future. No executor spins
        # or state mutation occur on the worker while it waits for the loop owner.
        return answer.result()

    def _dispatch(self, answer, kind, goal, label, timeout, extra):
        """在所有者循环接收工作线程请求，同一时刻最多持有一个动作阶段.

        BusyError 和阶段创建中的 Exception 在这里捕获后写入 answer，
        由工作线程读取结果时接收，不直接作为本回调的拒绝异常抛出。

        Args:
            answer: 尚未完成或取消的跨线程结果 Future；拒绝或创建异常通过它交还。
            kind: navigate 或 spin，选择共享后端动作。
            goal: 平面位姿（米、弧度）或 SpinGoal（弧度、秒）。
            label: 保持不变的阶段日志标签。
            timeout: 接收目标后的结果等待预算（秒）。
            extra: 导航反馈附加距离（米），旋转阶段不使用。

        Raises:
            concurrent.futures.InvalidStateError: 应答 Future 已完成或取消，
                连异常写回也失败；本函数不吞掉这个桥接状态错误。
        """
        try:
            if self._stop.is_set():
                self._logger.warn('[dock] %s: 已收到取消请求，不再下发' % label)
                answer.set_result(DockActionResult(False, True))
                return
            if self._stage is not None and not self._stage.done():
                raise BusyError('Dock worker already has a stage in flight')
            self._label = label
            self._cancel_logged = False
            self._stage = self._supervisor.spawn(
                self._run_stage(kind, goal, label, timeout, extra), 'dock-action-stage')
            self._stage.add_done_callback(lambda task: self._finish_stage(task, answer))
        except Exception as error:
            answer.set_exception(error)

    def _finish_stage(self, task, answer):
        """仅清除仍对应本阶段的标识，并把阶段结果桥接给工作线程.

        取消转换为明确的取消结果，其它异常写回 Future；本函数不额外取消后端，
        也不把阶段标识已清除当作后端运动所有权已释放的证明。

        Args:
            task: 已结束的本地动作阶段任务。
            answer: 工作线程等待中的跨线程 Future，必须尚未完成。

        Raises:
            Exception: 写入已经完成或取消的应答 Future 等桥接错误。
        """
        if self._stage is task:
            self._stage = None
            self._label = None
        try:
            answer.set_result(task.result())
        except asyncio.CancelledError:
            answer.set_result(DockActionResult(False, True))
        except BaseException as error:
            answer.set_exception(error)

    async def _run_stage(self, kind, goal, label, timeout, extra):
        """通过唯一后端执行导航或旋转，确认终态后转换为 Dock 阶段结果.

        Args:
            kind: navigate 使用位姿动作，否则使用已验证的旋转动作。
            goal: 平面位姿（米、弧度）或含角度及时间预算的 SpinGoal。
            label: 原有日志阶段标签。
            timeout: 接收目标后的本地结果等待预算（秒）。
            extra: 转发导航反馈时附加的剩余距离（米）。

        Returns:
            成功、失败或取消的阶段结果；停止标志优先于动作成功。

        Raises:
            Exception: 后端或清理失败，不转换为可继续执行的普通失败结果。
            asyncio.CancelledError: 当前阶段被取消，由后端完成其清理约定。
        """
        def deadline(state):
            if state == ActionState.SEND_TIMEOUT:
                self._logger.error('[dock] %s: 发目标后 10s 没有应答' % label)
            else:
                self._logger.error('[dock] %s: 超时 %.0fs，取消' % (label, timeout))

        budgets = dict(server_timeout_sec=self._server_timeout,
                       send_timeout_sec=self._send_timeout,
                       result_timeout_sec=timeout, on_timeout=deadline)
        if kind == 'navigate':
            x, y, yaw = goal
            pose = PoseRecord(self._frame, float(x), float(y), 0.0, float(yaw), 0.0)

            def feedback(raw):
                if not self._stop.is_set() and self._feedback is not None:
                    self._feedback(raw, extra)

            result = await self._backend.navigate_pose(pose, feedback, **budgets)
        else:
            result = await self._backend.spin_action(goal, **budgets)
        if self._stop.is_set():
            return DockActionResult(False, True)
        if result.state == ActionState.UNAVAILABLE:
            self._logger.error('[dock] %s: action server 不可用' % label)
        elif result.state == ActionState.REJECTED:
            self._logger.error('[dock] %s: 目标被拒绝' % label)
        elif result.state == ActionState.TERMINAL:
            status = result.response.status
            ok = status == 4
            self._logger.info('[dock] %s: %s' % (
                label, 'SUCCEEDED' if ok else 'status=%d' % status))
            return DockActionResult(ok)
        return DockActionResult(False)
