"""以单个共享槽位持有 ROS 动作，包括迟到句柄的取消清理."""

import asyncio
from dataclasses import dataclass
from enum import Enum
import logging
import math
from typing import Callable, Protocol

from nav_tasks.adapters.ros.futures import await_rclpy_future, FutureLike
from nav_tasks.core.ports import Clock
from nav_tasks.core.retry import MonotonicClock
from nav_tasks.core.supervisor import TaskSupervisor
from nav_tasks.domain.errors import BusyError, ClosedError, ConfigError

_LOGGER = logging.getLogger(__name__)
_TERMINAL_STATUSES = frozenset((4, 5, 6))


class _PhaseExpired(Exception):
    """表示本地阶段等待到期，与源 Future 返回的 TimeoutError 区分."""


class ActionClientLike(Protocol):
    """描述动作客户端所需接口，模块导入时不依赖 rclpy."""

    def server_is_ready(self) -> bool:
        """探测服务就绪，约定实现不能阻塞事件循环.

        Returns:
            当前服务是否可用；协议本身不强制非阻塞或等待期限。
        """
        ...

    def send_goal_async(
        self, goal: object, feedback_callback: Callable[[object], None],
    ) -> FutureLike:
        """异步提交动作目标并返回可追踪接收状态的 Future.

        Args:
            goal: 对应动作类型的目标消息。
            feedback_callback: 原始反馈回调，可能由 ROS executor 线程调用。

        Returns:
            最终给出目标句柄或拒绝状态的 Future。
        """
        ...


class ActionState(str, Enum):
    """描述单次尝试的会话分类，尚未转换成具体技能结果.

    Attributes:
        REJECTED: 服务拒绝目标或未给出句柄。
        RESULT_TIMEOUT: 结果等待到期，且已确认软件清理。
        SEND_TIMEOUT: 接收等待到期，且已确认软件清理。
        TERMINAL: 收到状态 4、5 或 6；不是物理停稳证明。
        UNAVAILABLE: 就绪预算内未发现服务。
    """

    UNAVAILABLE = 'unavailable'
    REJECTED = 'rejected'
    TERMINAL = 'terminal'
    SEND_TIMEOUT = 'send_timeout'
    RESULT_TIMEOUT = 'result_timeout'


@dataclass(frozen=True)
class ActionOutcome:
    """保存单次尝试分类与可选 ROS 响应，不深度冻结响应对象.

    构造本身不校验状态与响应的匹配关系。

    Attributes:
        response: 可选的原始终态响应引用，默认 None。
        state: 会话结果分类。
    """

    state: ActionState
    response: object | None = None


@dataclass
class _Attempt:
    """记录单次动作的可变所有权状态，不把软件标志当作物理停稳证据.

    Attributes:
        on_feedback: 本次尝试的循环线程反馈接收者。
        submission_started: 已进入提交调用，即使调用尚未返回可追踪 Future。
        send: 接收 Future；提交抛错时可能仍为 None，不能据此断言未发出。
        handle: 可选目标句柄，供迟到接收与取消使用。
        result: 可选终态 Future，失败时可在清理路径重新获取。
        cancel_requested: 取消请求标志，同时封闭旧反馈。
        settled: 已确认软件所有权结束，不是实体静止标志。
        cleanup: 受监督并保留强引用的后台清理任务。
    """

    on_feedback: Callable[[object], None]
    submission_started: bool = False
    send: FutureLike | None = None
    handle: object | None = None
    result: FutureLike | None = None
    cancel_requested: bool = False
    settled: bool = False
    cleanup: asyncio.Task | None = None


class ActionSession:
    """让所有共享客户端一次仅持有一个动作，含未完成取消清理.

    取消需处理迟到接收、取消确认与终态；已确认终态或拒绝无需额外取消。
    调用方等待超时或取消不会丢弃后台清理，也不因此释放槽位。
    这些屏障描述软件所有权，不保证实车已经停止。
    """

    def __init__(
        self, *, cancel_timeout_sec: float = 5.0, server_poll_sec: float = 0.05,
        clock: Clock | None = None,
    ) -> None:
        """在持有动作状态的 asyncio 循环内创建会话，不启动动作.

        Args:
            cancel_timeout_sec: 每次调用方清理等待预算，有限非负；超时保留清理。
            server_poll_sec: 就绪探测间隔，有限且严格为正。
            clock: 可注入的就绪等待单调时钟；阶段定时仍使用 asyncio。

        Raises:
            ConfigError: 已检查的间隔非有限或范围错误。
            RuntimeError: 当前没有运行中的 asyncio 循环。
            Exception: 类型转换等异常；此处未统一拒绝布尔值或归一化类型错误。
        """
        if not math.isfinite(cancel_timeout_sec) or cancel_timeout_sec < 0:
            raise ConfigError('cancel_timeout_sec must be finite and nonnegative')
        if not math.isfinite(server_poll_sec) or server_poll_sec <= 0:
            raise ConfigError('server_poll_sec must be finite and positive')
        self._loop = asyncio.get_running_loop()
        self._clock = clock if clock is not None else MonotonicClock()
        self._cancel_timeout = cancel_timeout_sec
        self._server_poll = server_poll_sec
        self._active: _Attempt | None = None
        self._closed = False
        self._supervisor = TaskSupervisor(self._on_crash)

    @property
    def busy(self) -> bool:
        """读取是否仍有尝试持有槽位；这不是跨线程同步或物理运动状态."""
        return self._active is not None

    async def execute(
        self, client: ActionClientLike, goal_factory: Callable[[], object],
        on_feedback: Callable[[object], None],
        *, server_timeout_sec: float, send_timeout_sec: float | None = None,
        result_timeout_sec: float | None = None,
        on_timeout: Callable[[ActionState], None] | None = None,
    ) -> ActionOutcome:
        """执行单次目标，按接收、终态或清理后的超时分类返回.

        先占有槽位再等待就绪，服务可用后才构造目标。仅状态 4/5/6 确认终态。
        本地阶段到期会先调用审计钩子，再尝试清理；钩子异常也不跳过清理。
        源异常或非法终态不证明所有权结束，可能保留忙碌槽位，需外层清理。
        调用方取消时尝试清理，清理异常可能替代原取消异常。

        Args:
            client: 由运行时持有、使用既有 ROS executor 的动作客户端。
            goal_factory: 服务就绪后生成新消息的工厂。
            on_feedback: 循环线程回调；取消开始后过滤排队的旧反馈。
            server_timeout_sec: 就绪等待预算，使用注入时钟。
            send_timeout_sec: 可选接收预算，与清理预算分开。
            result_timeout_sec: 可选接收后结果预算；已完成 Future 走直接读取路径。
            on_timeout: 可选同步审计钩子，在本地超时清理前运行。

        Returns:
            不可用、拒绝、终态，或仅在确认清理后返回的阶段超时。

        Raises:
            BusyError: 其他动作或未完成清理占据槽位。
            ClosedError: 接收已封闭。
            ConfigError: 已检查的等待预算无效。
            asyncio.CancelledError: 调用方或源动作取消，已尝试清理。
            RuntimeError: 跨事件循环调用或结果不是认可的终态。
            TimeoutError: 清理等待到期，或源 Future 自身抛出此异常。
            Exception: 工厂、客户端、时钟、审计或清理异常。
        """
        self._check_loop()
        if self._closed:
            raise ClosedError('action session is closed')
        if self._active is not None:
            raise BusyError('another action or its cleanup still owns the backend')
        if not math.isfinite(server_timeout_sec) or server_timeout_sec < 0:
            raise ConfigError('server_timeout_sec must be finite and nonnegative')
        for name, value in (('send_timeout_sec', send_timeout_sec),
                            ('result_timeout_sec', result_timeout_sec)):
            if value is not None and (type(value) not in (int, float)
                                      or not math.isfinite(value) or value < 0):
                raise ConfigError(f'{name} must be finite and nonnegative or None')
        attempt = _Attempt(on_feedback)
        self._active = attempt
        deadline = self._clock.now() + server_timeout_sec
        try:
            while not client.server_is_ready():
                if attempt.cancel_requested:
                    raise asyncio.CancelledError
                remaining = deadline - self._clock.now()
                if remaining <= 0:
                    attempt.settled = True
                    return ActionOutcome(ActionState.UNAVAILABLE)
                await self._clock.sleep(min(self._server_poll, remaining))
            if attempt.cancel_requested:
                raise asyncio.CancelledError
            goal = goal_factory()
            attempt.submission_started = True
            attempt.send = client.send_goal_async(
                goal, feedback_callback=lambda message: self._queue_feedback(attempt, message))
            phase = ActionState.SEND_TIMEOUT
            attempt.handle = await self._wait_phase(attempt.send, send_timeout_sec)
            if attempt.handle is None or not attempt.handle.accepted:
                attempt.settled = True
                return ActionOutcome(ActionState.REJECTED)
            if attempt.cancel_requested:
                raise asyncio.CancelledError
            if attempt.result is None:
                attempt.result = attempt.handle.get_result_async()
            phase = ActionState.RESULT_TIMEOUT
            response = await self._wait_phase(attempt.result, result_timeout_sec)
            if getattr(response, 'status', None) not in _TERMINAL_STATUSES:
                raise RuntimeError('action result did not confirm a terminal status')
            if attempt.cancel_requested:
                # The result may precede the cancellation-service response.
                # Do not let close/new admission bypass that still-owned cleanup.
                await self._cancel_attempt(attempt)
            else:
                attempt.settled = True
            return ActionOutcome(ActionState.TERMINAL, response)
        except _PhaseExpired:
            # A deadline is a policy failure, not evidence that motion stopped.
            # Do not hand the skill a failure it can continue from until cleanup
            # proves settlement. Cleanup failure propagates and retains ownership.
            try:
                if on_timeout is not None:
                    on_timeout(phase)
            finally:
                # A broken audit sink must not prevent cancellation cleanup.
                await self._cancel_attempt(attempt)
            return ActionOutcome(phase)
        except asyncio.CancelledError:
            await self._cancel_attempt(attempt)
            raise
        finally:
            if attempt.settled and self._active is attempt:
                self._active = None

    async def cancel(self) -> None:
        """取消并清理当前动作，空闲时不做操作，也不封闭后续接收.

        等待超时或等待者取消仅结束本次等待，受监督的清理继续持有槽位。
        结束的失败清理可重试，但不保证底层动作错误能够恢复。

        Raises:
            TimeoutError: 清理等待预算用尽，所有权不释放。
            asyncio.CancelledError: 等待者或底层清理取消。
            Exception: 清理失败或跨循环调用。
        """
        self._check_loop()
        if self._active is not None:
            await self._cancel_attempt(self._active)

    async def close(self) -> None:
        """先永久封闭新动作，再等待已持有工作清理完成.

        失败后仍保持封闭，后续 close 可重试清理；成功前不得销毁 ROS 客户端。
        这里只确认动作软件所有权结束，不执行实体停稳采样。

        Raises:
            Exception: 清理未完成或跨循环调用，相关资源仍需保留。
            asyncio.CancelledError: 等待者或底层清理取消。
        """
        self._check_loop()
        self._closed = True
        await self.cancel()

    def _check_loop(self) -> None:
        if asyncio.get_running_loop() is not self._loop:
            raise RuntimeError('action session used from a different event loop')

    async def _wait_phase(self, future: FutureLike, timeout: float | None) -> object:
        """等待一个动作阶段，超时仅取消本地桥接任务，不取消原 ROS Future.

        Args:
            future: 仍由动作尝试持有、用于迟到接收和终态清理的 Future。
            timeout: 等待预算（秒）；None 或已完成 Future 不走定时分支。

        Returns:
            ROS Future 的结果。

        Raises:
            _PhaseExpired: 预算内桥接任务未结束，原 Future 仍由尝试持有。
            asyncio.CancelledError: 等待者被取消，本地桥接任务先收尾。
            Exception: ROS Future 或桥接任务失败。
        """
        if timeout is None or future.done():
            return await await_rclpy_future(future)
        waiter = self._supervisor.spawn(await_rclpy_future(future), 'action-phase-wait')
        try:
            completed, _ = await asyncio.wait((waiter,), timeout=timeout)
            if completed:
                return waiter.result()
            raise _PhaseExpired
        finally:
            # Cancel only our bridge, never the source ROS future. The attempt
            # retains that future for late admission and terminal cleanup.
            if not waiter.done():
                waiter.cancel()
            await asyncio.gather(waiter, return_exceptions=True)

    async def _cancel_attempt(self, attempt: _Attempt) -> None:
        """封闭本次动作反馈，并在调用方预算内等待仍由本会话持有的清理.

        已经收尾的尝试直接返回；未运行或已结束的清理可重新创建。
        shield 使等待超时或调用方取消不取消后台清理，不据此释放动作所有权。

        Args:
            attempt: 需要取消的动作尝试，包含清理任务及收尾状态。

        Raises:
            TimeoutError: 本次等待预算耗尽，后台清理仍可能运行。
            asyncio.CancelledError: 等待者或清理任务被取消。
            Exception: 清理失败，所有权保留供后续重试。
        """
        attempt.cancel_requested = True
        if attempt.settled:
            return
        if attempt.cleanup is None or attempt.cleanup.done():
            attempt.cleanup = self._supervisor.spawn(self._cleanup(attempt), 'action-cleanup')
        await asyncio.wait_for(asyncio.shield(attempt.cleanup), timeout=self._cancel_timeout)

    async def _cleanup(self, attempt: _Attempt) -> None:
        """等待迟到的目标接收并确认动作终态后，才释放尝试的所有权.

        取消服务应答不等于终态；失败结果可重新请求，不能仅凭取消请求成功
        就允许下一段动作。失败时保持 active，供上层重试清理。

        Args:
            attempt: 持有提交、结果和目标句柄的动作尝试。

        Raises:
            RuntimeError: 提交不可追踪，或结果未确认动作处于终态。
            Exception: ROS 提交、取消或结果 Future 失败，所有权不释放。
            asyncio.CancelledError: 清理被取消，所有权不释放。
        """
        if attempt.submission_started and attempt.send is None:
            raise RuntimeError('goal submission failed without a trackable future')
        if attempt.send is not None:
            handle = await await_rclpy_future(attempt.send)
            attempt.handle = handle
            if handle is not None and handle.accepted:
                if attempt.result is None:
                    attempt.result = handle.get_result_async()
                confirmed_terminal = False
                refresh_result = False
                if attempt.result.done():
                    try:
                        confirmed_terminal = getattr(
                            attempt.result.result(), 'status', None) in _TERMINAL_STATUSES
                        refresh_result = not confirmed_terminal
                    except Exception:
                        refresh_result = True
                if not confirmed_terminal:
                    await await_rclpy_future(handle.cancel_goal_async())
                    if refresh_result:
                        attempt.result = handle.get_result_async()
                response = await await_rclpy_future(attempt.result)
                if getattr(response, 'status', None) not in _TERMINAL_STATUSES:
                    raise RuntimeError('cancel did not confirm a terminal action status')
        attempt.settled = True
        if self._active is attempt:
            self._active = None

    def _queue_feedback(self, attempt: _Attempt, message: object) -> None:
        # ROS may call from its executor thread. No task state is touched there.
        """把 ROS 线程反馈排入所有者事件循环，不在回调线程修改任务状态.

        循环已关闭时仅记录并忽略反馈；是否仍属于活动尝试由循环内回调判断。

        Args:
            attempt: 收到反馈的动作尝试。
            message: ROS 原始反馈对象，不在此转换。
        """
        try:
            self._loop.call_soon_threadsafe(self._feedback, attempt, message)
        except RuntimeError:
            # Runtime shutdown owns cleanup; a closed loop cannot consume feedback.
            _LOGGER.debug('Ignoring action feedback after the event loop closed')

    def _feedback(self, attempt: _Attempt, message: object) -> None:
        """仅向仍活动且未取消、未收尾的尝试交付反馈.

        必须在所有者事件循环调用；被替换尝试或迟到反馈不会重新激活任务。

        Args:
            attempt: 接收反馈时关联的动作尝试。
            message: 交给尝试回调的 ROS 原始反馈。

        Raises:
            Exception: 活动尝试的反馈回调失败，由外层执行器处理。
        """
        if self._active is attempt and not attempt.cancel_requested and not attempt.settled:
            attempt.on_feedback(message)

    @staticmethod
    def _on_crash(name: str, error: BaseException) -> None:
        _LOGGER.error('Owned %s failed; action remains busy: %s', name, error)
