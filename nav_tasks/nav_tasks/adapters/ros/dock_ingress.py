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
        position, q = message.pose.position, message.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # Preserve the old interface: header is not transformed, z is ignored.
        return PoseRecord(self._config.runtime.global_frame,
                          position.x, position.y, 0.0, yaw, 0.0)

    def _topic_callback(self, message):
        try:
            pose = self._pose(message)
        except Exception:
            self._log_exception(None)
            return
        self._loop.call_soon_threadsafe(self._start, pose, None, None)

    def _goal_callback(self, request):
        def admit():
            if (not self._opened or self._closing
                    or self._tasks.status().state != TaskState.IDLE):
                self._logger.warn('[dock] 上一个目标还在执行，拒绝动作目标')
                return self._goal_response.REJECT
            return self._goal_response.ACCEPT

        return self._call(admit)

    def _cancel_callback(self, handle):
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
        while not entry.settled.done():
            try:
                await self._tasks.cancel(entry.task_id, generation=entry.snapshot.generation)
                return
            except Exception as error:
                _LOGGER.warning('Dock cleanup pending: %s; retrying', error)
                await asyncio.sleep(self._retry)

    async def _close(self):
        while self._entries:
            await asyncio.gather(*(asyncio.shield(entry.task)
                                   for entry in tuple(self._entries.values())))
        self._closed = True

    def _log_exception(self, handle):
        prefix = '[dock] 动作目标执行异常，本次目标放弃：\n%s' if handle is not None else (
            '[dock] 编排线程异常，本次目标放弃：\n%s')
        self._logger.error(prefix % traceback.format_exc())

    @staticmethod
    def _on_crash(name, error):
        _LOGGER.error('Dock ingress operation %s failed: %s', name, error)
