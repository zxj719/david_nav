"""ROS Dock ingress for one loop-owned manager, without another motion client.

The host must use a MultiThreadedExecutor with at least two callback threads.
Synchronous action execution waits on an internal concurrent future, never spins
ROS and never runs the blocking Dock workflow itself. The runtime's DockWorker
owns that workflow. Stop/close retain ROS resources; destroy follows successful
host executor quiescence, including rclpy's result-publication callback tail.
"""

import asyncio
from concurrent.futures import Future
from dataclasses import dataclass, field
import logging
import math
import threading
import traceback
from typing import Callable
from uuid import uuid4

from nav_tasks.core.supervisor import TaskSupervisor
from nav_tasks.domain.errors import BusyError, ClosedError, ConfigError
from nav_tasks.domain.runtime_config import DockNodeConfig
from nav_tasks.domain.tasks import (
    DockGoal, EventKind, NavStatus, TaskRequest, TaskSnapshot, TaskState,
)
from nav_tasks.domain.types import PoseRecord
from nav_tasks.skills.docking.config import workflow_config

_LOGGER = logging.getLogger(__name__)


@dataclass
class _Entry:
    task_id: str
    handle: object
    settled: asyncio.Future
    inbox: asyncio.Queue = field(default_factory=asyncio.Queue)
    snapshot: TaskSnapshot | None = None
    cancel_requested: bool = False
    failure: bool = False
    task: asyncio.Task | None = None


class DockIngress:
    """Own topic/action adapters, marshaling all task state to their asyncio loop."""

    def __init__(
        self, tasks: object, config: DockNodeConfig, *, cleanup_retry_sec: float = 1.0,
    ) -> None:
        """Create an empty adapter owner before opening caller-owned ROS resources.

        Args:
            tasks: Public TaskApi implementation on the current running event loop.
            config: Explicit validated ingress names and Dock policy.
            cleanup_retry_sec: Delay between failed owned-task cancellation attempts.

        Raises:
            ConfigError: Configuration or cleanup retry interval is invalid.
        """
        if not isinstance(config, DockNodeConfig):
            raise ConfigError('Dock ingress requires DockNodeConfig')
        workflow_config(config.runtime)
        for name in ('goal_topic', 'action_name'):
            value = getattr(config, name)
            if not isinstance(value, str) or not value.strip():
                raise ConfigError(f'dock.{name} must be a nonempty string')
        if (type(cleanup_retry_sec) not in (int, float)
                or not math.isfinite(cleanup_retry_sec) or cleanup_retry_sec <= 0):
            raise ConfigError('cleanup_retry_sec must be finite and positive')
        self._tasks, self._config = tasks, config
        self._loop = asyncio.get_running_loop()
        self._thread = threading.get_ident()
        self._retry = cleanup_retry_sec
        self._supervisor = TaskSupervisor(self._on_crash)
        self._entries: dict[str, _Entry] = {}
        self._pending_cancels: dict[int, object] = {}
        self._node = self._subscription = self._server = self._action = None
        self._logger = None
        self._closing = self._closed = False
        self._opened = False
        self._close_task = None

    def open(self, node: object) -> None:
        """Create the old depth-one goal subscription and reentrant action service.

        The caller retains this owner if either construction stage fails. No node,
        executor, TF listener or action client is created by this adapter.

        Args:
            node: Caller-owned ROS node kept alive until destroy succeeds.

        Raises:
            RuntimeError: Open was already attempted, or the owner is closing.
            Exception: Construction failed; close/quiesce/destroy completed stages.
        """
        self._check_loop()
        if self._node is not None or self._closing:
            raise RuntimeError('Dock ingress was already opened or closed')
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionServer, CancelResponse, GoalResponse
        from rclpy.callback_groups import ReentrantCallbackGroup

        self._node, self._logger, self._action = node, node.get_logger(), NavigateToPose
        self._goal_response, self._cancel_response = GoalResponse, CancelResponse
        group = ReentrantCallbackGroup()
        self._subscription = node.create_subscription(
            PoseStamped, self._config.goal_topic, self._topic_callback, 1, callback_group=group)
        self._server = ActionServer(
            node, NavigateToPose, self._config.action_name, self._execute_action,
            callback_group=group, goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback)
        self._opened = True

    def stop(self) -> None:
        """Seal admission immediately and wake owned requests without destroying ROS resources."""
        self._check_loop()
        self._closing = True
        for entry in self._entries.values():
            entry.inbox.put_nowait(None)

    async def close(self) -> None:
        """Seal admission and settle only this ingress's owned tasks, retaining ROS callbacks.

        Canceling this waiter does not cancel owned cleanup. Retryable task cleanup
        errors keep the request and its action callback pending, never report a
        terminal result as if motion stopped. The host must keep spinning.
        After this completes, quiesce the executor before calling destroy.
        """
        self._check_loop()
        self.stop()
        if self._close_task is None:
            self._close_task = self._supervisor.spawn(self._close(), 'close-dock-ingress')
        await asyncio.shield(self._close_task)

    def destroy(self) -> None:
        """Release ROS resources only after close AND successful host executor quiescence.

        The host guarantees no rclpy callback tails or queued goal callbacks remain.
        This method cannot prove that host invariant and must not replace executor
        shutdown. A failed stage is retained for retry; the external node is never
        destroyed here.

        Raises:
            RuntimeError: Close has not completed or subscription destruction failed.
            Exception: ROS destruction failed; retain this owner and retry.
        """
        self._check_loop()
        if not self._closed:
            raise RuntimeError('close Dock ingress before destroying its ROS resources')
        if self._server is not None:
            self._server.destroy()
            self._server = None
        if self._subscription is not None:
            if self._node.destroy_subscription(self._subscription) is False:
                raise RuntimeError('Dock goal subscription destruction failed')
            self._subscription = None
        self._pending_cancels.clear()

    def on_feedback(self, goal: DockGoal, raw: object, extra_distance: float) -> None:
        """Forward complete legacy feedback only for the matching live action generation.

        Args:
            goal: Loop-thread worker identity, including admission generation.
            raw: Original NavigateToPose feedback wrapper, not modified here.
            extra_distance: Remaining later-stage distance added to this stage's value.
        """
        self._check_loop()
        entry = self._entries.get(goal.request_id)
        if (entry is None or entry.handle is None or self._closing or entry.cancel_requested
                or entry.settled.done() or entry.failure):
            return
        state = self._tasks.status()
        if ((state.task_id, state.sub_id, state.generation) !=
                (goal.request_id, goal.sub_id, goal.generation)
                or state.state not in (TaskState.STARTING, TaskState.NAVIGATING)):
            return
        try:
            if entry.handle.is_cancel_requested or not entry.handle.is_active:
                return
            inner, feedback = raw.feedback, self._action.Feedback()
            feedback.current_pose = inner.current_pose
            feedback.navigation_time = inner.navigation_time
            feedback.estimated_time_remaining = inner.estimated_time_remaining
            feedback.number_of_recoveries = inner.number_of_recoveries
            feedback.distance_remaining = float(inner.distance_remaining) + float(extra_distance)
            entry.handle.publish_feedback(feedback)
        except Exception:
            entry.failure = True
            entry.inbox.put_nowait(None)
            self._log_exception(entry.handle)

    def _check_loop(self):
        if asyncio.get_running_loop() is not self._loop:
            raise RuntimeError('Dock ingress used from a different event loop')

    def _call(self, function: Callable, *args):
        """把同步操作投递到入口所有者循环，并在 ROS 回调线程等待结果.

        Args:
            function: 不应阻塞所有者循环的同步函数；同线程时直接调用。
            args: 传给函数的位置参数。

        Returns:
            函数结果；跨线程等待没有超时，所有者循环必须保持存活。

        Raises:
            BaseException: 函数异常由 Future 原样转回调用线程。
            RuntimeError: 无法向已关闭的事件循环投递。
        """
        if threading.get_ident() == self._thread:
            return function(*args)
        answer = Future()

        def invoke():
            try:
                answer.set_result(function(*args))
            except BaseException as error:
                answer.set_exception(error)

        self._loop.call_soon_threadsafe(invoke)
        return answer.result()

    def _pose(self, message):
        """沿用旧 Dock 接口提取平面位姿，按配置帧解释数值而不转换 header.

        保留 x/y 和四元数计算的 yaw，忽略输入 z、header 帧和时间；
        输出 z 与时间戳均为零，不在此验证四元数归一化。

        Args:
            message: 含 pose.position 和 pose.orientation 的入口消息。

        Returns:
            使用配置全局帧的 PoseRecord，距离为米、角度为弧度。

        Raises:
            AttributeError: 消息缺少所需位姿字段。
            TypeError: 四元数字段不支持所需数值运算。
        """
        position, q = message.pose.position, message.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # Preserve the old interface: header is not transformed, z is ignored.
        return PoseRecord(self._config.runtime.global_frame,
                          position.x, position.y, 0.0, yaw, 0.0)

    def _topic_callback(self, message):
        """解析话题位姿后投递到所有者循环，回调线程不直接提交任务.

        解析失败只记录并忽略本次输入；循环内 _start 仍需检查入口关闭状态。

        Args:
            message: Dock 话题入口的位姿消息，沿用 _pose 的帧约定。

        Raises:
            RuntimeError: 向已关闭的事件循环投递失败。
            Exception: 解析失败后的日志记录本身失败。
        """
        try:
            pose = self._pose(message)
        except Exception:
            self._log_exception(None)
            return
        self._loop.call_soon_threadsafe(self._start, pose, None, None)

    def _goal_callback(self, request):
        """在所有者循环检查 Dock 动作准入，只接受当前空闲且开放的入口.

        此回调不保留运动槽位，也不校验目标位姿；最终提交仍由任务内核裁决。

        Args:
            request: ROS 动作请求，此处不读取其内容。

        Returns:
            ROS GoalResponse 的 ACCEPT 或 REJECT。

        Raises:
            Exception: 跨线程调用、状态读取或拒绝日志失败。
        """
        def admit():
            if (not self._opened or self._closing
                    or self._tasks.status().state != TaskState.IDLE):
                self._logger.warn('[dock] 上一个目标还在执行，拒绝动作目标')
                return self._goal_response.REJECT
            return self._goal_response.ACCEPT

        return self._call(admit)

    def _cancel_callback(self, handle):
        """在所有者循环记录动作取消请求，唤醒已登记任务或保留提前取消.

        Args:
            handle: ROS 动作目标句柄，以对象身份匹配入口任务。

        Returns:
            ROS 取消接收响应；仅确认请求已记录，不代表运动已停稳。

        Raises:
            Exception: 跨线程投递或循环中的取消记录操作失败。
        """
        def cancel():
            self._logger.warn('[dock] 收到动作取消请求')
            for entry in self._entries.values():
                if entry.handle is handle:
                    entry.cancel_requested = True
                    entry.inbox.put_nowait(None)
                    break
            else:
                self._pending_cancels[id(handle)] = handle
            return self._cancel_response.ACCEPT

        return self._call(cancel)

    def _execute_action(self, handle):
        """在 ROS 工作线程等待入口任务收尾，再设置动作句柄的终态.

        Args:
            handle: ROS 动作目标句柄；其请求位姿沿用既有不转换 header 的约定。

        Returns:
            空动作结果；成功、取消或失败由句柄状态表达。

        Raises:
            RuntimeError: 在 asyncio 所有者线程调用会死锁，因此直接拒绝。
            Exception: ROS 终态设置或清理提前取消记录失败；任务执行异常
                本身会记录并转换为失败结果。
        """
        if threading.get_ident() == self._thread:
            raise RuntimeError('Dock action execution requires a multithreaded ROS host')
        answer = Future()
        try:
            pose = self._pose(handle.request.pose)
            self._loop.call_soon_threadsafe(self._start, pose, handle, answer)
            outcome = answer.result()
        except Exception:
            self._log_exception(handle)
            outcome = NavStatus.FAILED
        if handle.is_active:
            if handle.is_cancel_requested:
                handle.canceled()
            elif outcome == NavStatus.SUCCEEDED:
                handle.succeed()
            else:
                handle.abort()
        self._call(self._pending_cancels.pop, id(handle), None)
        return self._action.Result()

    def _start(self, pose, handle, answer):
        """在所有者循环登记入口请求，并把执行结果桥接给等待中的 ROS 线程.

        Args:
            pose: 请求位姿记录，位置为米、yaw 为弧度。
            handle: 动作句柄；话题入口为 None。
            answer: ROS 线程持有的结果 Future；话题入口为 None。

        Raises:
            Exception: 创建后台任务或写入应答失败；已登记入口仍由本对象持有。
        """
        if self._closing or not self._opened:
            if answer is not None:
                answer.set_result(NavStatus.CANCELED)
            return
        entry = _Entry(f'dock:{uuid4()}', handle, self._loop.create_future())
        if handle is not None:
            entry.cancel_requested = (self._pending_cancels.pop(id(handle), None) is not None
                                      or handle.is_cancel_requested)
        self._entries[entry.task_id] = entry
        entry.task = self._supervisor.spawn(self._run(entry, pose), 'dock-ingress-request')
        if answer is not None:
            def completed(task):
                try:
                    answer.set_result(task.result())
                except BaseException as error:
                    answer.set_exception(error)
            entry.task.add_done_callback(completed)

    async def _run(self, entry, pose):
        """提交 Dock 请求并等待所有权释放凭据，不把进度事件误当作完成.

        关闭、取消或观察失败时等待自有任务清理；finally 等待 ROS 取消状态
        可见后解除事件订阅并移除入口，避免取消应答与执行结果竞态。

        Args:
            entry: 本次入口记录，持有任务身份、事件队列和收尾 Future。
            pose: 提交给任务内核的 Dock 位姿记录（米、弧度）。

        Returns:
            收尾结果的导航状态；忙碌、配置错误和已处理异常转换为失败。

        Raises:
            asyncio.CancelledError: 外部取消协程；finally 仍回收事件观察资源。
            Exception: 异常处理中的清理或最终回收本身失败。
        """
        events = None
        observer = None
        try:
            if entry.cancel_requested or self._closing:
                return NavStatus.CANCELED
            events = self._tasks.subscribe_events()
            observer = self._supervisor.spawn(self._observe(entry, events), 'dock-ingress-events')

            def settled(receipt):
                if not entry.settled.done():
                    entry.settled.set_result(receipt)
                    entry.inbox.put_nowait(None)

            entry.snapshot = await self._tasks.submit(
                TaskRequest(entry.task_id, 'dock', {'pose': pose}, source='ros_dock'),
                on_settled=settled)
            while not entry.settled.done():
                if self._closing or entry.cancel_requested or entry.failure:
                    await self._cancel_owned(entry)
                else:
                    await entry.inbox.get()
            if entry.failure:
                return NavStatus.FAILED
            if self._closing or entry.cancel_requested:
                return NavStatus.CANCELED
            return entry.settled.result().outcome.status
        except BusyError:
            self._logger.warn('[dock] 上一个目标还在执行，动作目标放弃' if entry.handle else
                              '[dock] 上一个目标还在执行，忽略本次')
            return NavStatus.FAILED
        except (ClosedError, ConfigError):
            self._log_exception(entry.handle)
            return NavStatus.FAILED
        except Exception:
            self._log_exception(entry.handle)
            if entry.snapshot is not None:
                await self._cancel_owned(entry)
            return NavStatus.FAILED
        finally:
            # The cancel service marks CANCELING only after our callback returns.
            # Include cancellation before admission in this acknowledgement gap.
            if entry.cancel_requested and entry.handle is not None:
                while entry.handle.is_active and not entry.handle.is_cancel_requested:
                    await asyncio.sleep(0.01)
            if observer is not None:
                observer.cancel()
                await asyncio.gather(observer, return_exceptions=True)
            if events is not None:
                self._tasks.unsubscribe_events(events)
            self._entries.pop(entry.task_id, None)

    async def _observe(self, entry, events):
        """观察本入口对应任务的失败或暂停事件，通知入口流程启动收尾.

        有提交快照后同时核对代次，忽略旧代次及其它任务。
        成功事件不作为完成凭据；正常完成仍由独立 settled 回调确认。

        Args:
            entry: 当前入口记录，含身份、可选提交快照及唤醒队列。
            events: 任务内核提供的异步事件队列。

        Raises:
            asyncio.CancelledError: 入口退出时取消观察任务。
        """
        while True:
            event = await events.get()
            if event.task_id != entry.task_id:
                continue
            if entry.snapshot is not None and event.generation != entry.snapshot.generation:
                continue
            if event.kind == EventKind.FAILED or event.status == 'paused':
                entry.failure = True
                entry.inbox.put_nowait(None)

    async def _cancel_owned(self, entry):
        """按任务身份与代次重试取消，只等待自身入口对应的任务.

        收尾凭据已到则无需取消；取消调用正常返回即结束本辅助函数。
        真正的入口完成仍由外层检查 entry.settled，不把取消返回值当作完成凭据。

        Args:
            entry: 已有提交快照的入口记录，持有任务 ID、代次及收尾 Future。

        Raises:
            asyncio.CancelledError: 本协程被取消，普通异常重试不吞掉取消。
        """
        while not entry.settled.done():
            try:
                await self._tasks.cancel(entry.task_id, generation=entry.snapshot.generation)
                return
            except Exception as error:
                _LOGGER.warning('Dock cleanup pending: %s; retrying', error)
                await asyncio.sleep(self._retry)

    async def _close(self):
        """等待所有入口任务退出后标记关闭，不销毁 ROS 入口对象.

        外层已封闭入口；每轮固定当前任务快照，shield 保留仍在运行的入口任务。
        任一等待失败时不设置 closed，允许外层保留资源并重试。

        Raises:
            asyncio.CancelledError: 等待者或入口任务被取消。
            Exception: 某个入口任务失败，其余受屏蔽任务仍归入口持有。
        """
        while self._entries:
            await asyncio.gather(*(asyncio.shield(entry.task)
                                   for entry in tuple(self._entries.values())))
        self._closed = True

    def _log_exception(self, handle):
        """按入口种类输出冻结的 Dock 异常文本，附上当前异常的堆栈.

        应在处理异常期间调用；本函数不改变任务状态或执行取消。

        Args:
            handle: 动作入口句柄；None 表示话题编排入口。

        Raises:
            Exception: 日志接收器本身失败。
        """
        prefix = '[dock] 动作目标执行异常，本次目标放弃：\n%s' if handle is not None else (
            '[dock] 编排线程异常，本次目标放弃：\n%s')
        self._logger.error(prefix % traceback.format_exc())

    @staticmethod
    def _on_crash(name, error):
        _LOGGER.error('Dock ingress operation %s failed: %s', name, error)
