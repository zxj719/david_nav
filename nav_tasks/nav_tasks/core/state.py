"""Loop-confined task manager with serialized admission and cancellation cleanup."""

import asyncio
from dataclasses import dataclass
import logging
from typing import Awaitable, Callable, Mapping

from nav_tasks.core.registry import CommandRegistry, Handler, TaskContext
from nav_tasks.core.supervisor import TaskSupervisor
from nav_tasks.domain.errors import BusyError, ClosedError, ConfigError, NavTaskError
from nav_tasks.domain.tasks import (
    EventKind, NavOutcome, NavStatus, PreemptPolicy, TaskCompletion, TaskEvent, TaskRequest,
    TaskSnapshot, TaskState,
)

_LOGGER = logging.getLogger(__name__)


@dataclass
class _Active:
    request: TaskRequest
    generation: int
    on_settled: Callable[[TaskCompletion], None] | None = None
    worker: asyncio.Task | None = None
    remaining: float = 0.0
    cancel_requested: bool = False
    accept_feedback: bool = True
    target_id: str | None = None
    phase: str | None = None
    crash: BaseException | None = None
    crash_outcome: NavOutcome | None = None


class TaskManager:
    """Own at most one motion task on one asyncio loop.

    Admission and cancellation are short owned background operations. Canceling
    an API await does not undo an accepted request: use cancel explicitly. A
    loop-local admission lock serializes transitions across cleanup awaits; it
    is not a thread lock and never wraps normal skill execution.

    External CANCELED retains the legacy paused active task (G12). Unexpected
    skill exceptions are reported and trigger supervised backend cleanup. Ownership
    is released only after cleanup succeeds; failure retains ERROR for retry.
    """

    def __init__(
        self, registry: CommandRegistry, cancel_backend: Callable[[], Awaitable[None]],
        *, on_event: Callable[[str, Mapping[str, object]], None] | None = None,
    ) -> None:
        """Construct on the event loop that will own all task state.

        Args:
            registry: Skill decoders and handlers configured by the assembly root.
            cancel_backend: Cleanup operation for the single shared motion backend.
            on_event: Optional synchronous observation sink, separate from legacy WS events.
        """
        self._loop = asyncio.get_running_loop()
        self._registry = registry
        self._cancel_backend = cancel_backend
        self._on_event = on_event
        self._supervisor = TaskSupervisor(self._on_crash)
        self._transition_lock = asyncio.Lock()
        self._state = TaskState.IDLE
        self._active: _Active | None = None
        self._generation = 0
        self._closing = False
        self._close_task: asyncio.Task | None = None
        self._subscribers: set[asyncio.Queue[TaskEvent]] = set()

    async def submit(
        self, task: TaskRequest, *, on_settled: Callable[[TaskCompletion], None] | None = None,
    ) -> TaskSnapshot:
        """Decode and admit a motion task without awaiting its navigation result.

        Args:
            task: Domain request with explicit ingress preemption policy.
            on_settled: Optional loop-thread receipt after ownership release, including preemption.

        Returns:
            Snapshot of the accepted task before its skill starts running.

        Raises:
            ConfigError: Task identity or preemption policy is invalid.
            ClosedError: Closing has started.
            BusyError: Another task owns motion and preemption is rejected.
            UnknownActionError: No skill is registered for the action.
        """
        self._check_loop()
        self._check_open()
        if not isinstance(task.task_id, str) or not task.task_id.strip():
            raise ConfigError('task_id must be a nonempty string')
        if not isinstance(task.preempt, PreemptPolicy):
            raise ConfigError('preempt must be a PreemptPolicy')
        if not isinstance(task.source, str):
            raise ConfigError('task source must be a string')
        if on_settled is not None and not callable(on_settled):
            raise ConfigError('on_settled must be callable')
        handler, payload = self._registry.prepare(task.action, task.payload)
        operation = self._supervisor.spawn(
            self._submit(task, handler, payload, on_settled), f'admit:{task.task_id}')
        return await asyncio.shield(operation)

    async def cancel(self, task_id: str | None = None, *, generation: int | None = None) -> bool:
        """Cancel current ownership, waiting for backend and skill cleanup.

        Args:
            task_id: Matching task ID, or None to cancel the current owner.
            generation: Optional admission generation, checked inside the transition lock.

        Returns:
            True when a matching task was canceled; False for no match.

        Raises:
            Exception: Backend cleanup failed; ownership remains in ERROR.
        """
        self._check_loop()
        operation = self._supervisor.spawn(self._cancel(task_id, generation), 'cancel-task')
        return await asyncio.shield(operation)

    def status(self) -> TaskSnapshot:
        """Read a detached immutable observation on the manager event loop.

        Returns:
            Current ownership state and generation.
        """
        self._check_loop()
        active = self._active
        if active is None:
            return TaskSnapshot(self._state)
        return TaskSnapshot(
            self._state, active.request.task_id, active.request.sub_id,
            active.generation, active.remaining, active.target_id, active.phase)

    def subscribe_events(self) -> asyncio.Queue[TaskEvent]:
        """Subscribe to subsequent events without replaying earlier events.

        Consumers must drain their queue and unsubscribe on teardown. Queues do
        not drop terminal events and allocate nothing until explicitly created.
        Durable delivery and feedback throttling belong to adapter/skill policy.

        Returns:
            Subscriber-owned queue used on the manager event loop.

        Raises:
            ClosedError: Closing has started.
        """
        self._check_loop()
        self._check_open()
        queue: asyncio.Queue[TaskEvent] = asyncio.Queue()
        self._subscribers.add(queue)
        return queue

    def unsubscribe_events(self, queue: asyncio.Queue[TaskEvent]) -> None:
        """Detach a subscriber without discarding its already queued events.

        Args:
            queue: Queue returned by subscribe_events; unknown queues are ignored.
        """
        self._check_loop()
        self._subscribers.discard(queue)

    async def close(self) -> None:
        """Seal admission immediately and wait for owned motion cleanup.

        Repeated callers join the same close operation. A failed cleanup can be
        retried by calling close again; admission never reopens in the meantime.

        Raises:
            Exception: Cleanup failed; the task remains owned in ERROR.
        """
        self._check_loop()
        self._closing = True
        if self._close_task is None or (
                self._close_task.done() and self._state != TaskState.CLOSED):
            self._close_task = self._supervisor.spawn(self._close(), 'close-task-manager')
        await asyncio.shield(self._close_task)

    def _check_loop(self) -> None:
        if asyncio.get_running_loop() is not self._loop:
            raise RuntimeError('task manager used from a different event loop')

    def _check_open(self) -> None:
        if self._closing:
            raise ClosedError('task manager is closing or closed')

    async def _submit(
        self, request: TaskRequest, handler: Handler, payload: object,
        on_settled: Callable[[TaskCompletion], None] | None,
    ) -> TaskSnapshot:
        async with self._transition_lock:
            self._check_open()
            if self._active is not None:
                if request.preempt != PreemptPolicy.ALLOW:
                    raise BusyError(f'motion is owned by {self._active.request.task_id}')
                await self._cancel_active()
                # close() can seal admission while cancellation is awaiting I/O.
                self._check_open()
            self._generation += 1
            active = _Active(request, self._generation, on_settled)
            self._active = active
            self._state = TaskState.STARTING
            active.worker = self._supervisor.spawn(
                self._run(active, handler, payload),
                f'skill:{request.task_id}:{active.generation}')
            self._observe(active, 'task_admitted')
            return self.status()

    async def _run(self, active: _Active, handler: Handler, payload: object) -> None:
        context = TaskContext(
            active.request.task_id, active.request.sub_id, active.generation,
            lambda remaining: self._feedback(active, remaining),
            lambda: active.cancel_requested, lambda: self._starting(active),
            lambda target, phase: self._describe(active, target, phase))
        try:
            outcome = await handler(context, payload)
            if not isinstance(outcome, NavOutcome) or not isinstance(outcome.status, NavStatus):
                raise TypeError('skill must return a typed NavOutcome')
        except asyncio.CancelledError:
            active.accept_feedback = False
            if self._is_current(active):
                self._state = TaskState.NAVIGATING
                self._emit(active, EventKind.PROGRESS, status='paused')
            raise
        finally:
            active.accept_feedback = False
        if not self._is_current(active):
            return
        if outcome.status == NavStatus.CANCELED:
            # The legacy backend reports zero for external cancellation, unlike
            # our own abort acknowledgement, which retains the last feedback.
            active.remaining = 0.0
            self._state = TaskState.NAVIGATING
            self._emit(active, EventKind.PROGRESS, status='paused')
            return
        kind = EventKind.SUCCEEDED if outcome.succeeded else EventKind.FAILED
        self._emit(active, kind, detail=outcome.detail)
        self._active = None
        self._state = TaskState.IDLE
        self._settled(active, outcome)

    def _settled(self, active: _Active, outcome: NavOutcome) -> None:
        self._observe(active, 'task_' + outcome.status.value,
                      ownership_released=True, detail=outcome.detail)
        callback, active.on_settled = active.on_settled, None
        if callback is not None:
            receipt = TaskCompletion(active.request.task_id, active.request.sub_id,
                                     active.generation, outcome)
            try:
                callback(receipt)
            except Exception as error:
                self._on_crash('task-completion-observer', error)

    def _is_current(self, active: _Active) -> bool:
        return self._active is active and not active.cancel_requested

    def _starting(self, active: _Active) -> None:
        self._check_loop()
        if self._is_current(active) and active.accept_feedback:
            self._state = TaskState.STARTING

    def _describe(self, active: _Active, target_id: str, phase: str) -> None:
        self._check_loop()
        if self._is_current(active) and active.accept_feedback:
            active.target_id = target_id
            active.phase = phase
            self._observe(active, 'task_phase')

    def _feedback(self, active: _Active, remaining: float) -> None:
        self._check_loop()
        if self._is_current(active) and active.accept_feedback:
            active.remaining = float(remaining)
            self._state = TaskState.NAVIGATING
            self._emit(active, EventKind.PROGRESS)

    def _emit(
        self, active: _Active, kind: EventKind, *, status: str = 'running', detail: str = '',
    ) -> None:
        event = TaskEvent(
            kind, active.request.task_id, active.request.sub_id, active.generation,
            active.remaining, status, detail, active.request.source)
        for queue in self._subscribers:
            queue.put_nowait(event)
        if kind == EventKind.PROGRESS and self._active is active:
            self._observe(active, 'task_paused' if status == 'paused' else 'task_progress')

    def _observe(self, active, name, *, ownership_released=False, detail='', error_type=''):
        if self._on_event is None:
            return
        request = active.request
        data = dict(task_id=request.task_id, sub_id=request.sub_id,
                    generation=active.generation, action=request.action,
                    ingress_source=request.source, state=self._state.value,
                    target_id=active.target_id, phase=active.phase,
                    remaining_distance=active.remaining, ownership_released=ownership_released,
                    detail=detail, error_type=error_type)
        try:
            self._on_event(name, data)
        except Exception as error:
            _LOGGER.warning('Task observation failed (%s)', type(error).__name__)

    async def _cancel_active(self) -> _Active:
        active = self._active
        assert active is not None
        # Only a crash known before this cleanup attempt is an expected worker
        # result. A fresh exception raised during cancellation is still failure.
        reported_crash = active.crash
        active.cancel_requested = True
        self._state = TaskState.ABORTING
        self._observe(active, 'task_cleanup_started')
        if active.worker is not None and not active.worker.done():
            active.worker.cancel()
        cleanup_error = None
        try:
            await self._cancel_backend()
        except asyncio.CancelledError as error:
            cleanup_error = error
        except Exception as error:
            cleanup_error = error
        if active.worker is not None:
            try:
                await active.worker
            except asyncio.CancelledError:
                pass
            except Exception as error:
                if cleanup_error is None and error is not reported_crash:
                    cleanup_error = error
        if cleanup_error is not None:
            self._state = TaskState.ERROR
            self._observe(active, 'task_cleanup_failed', error_type=type(cleanup_error).__name__)
            raise cleanup_error
        self._active = None
        self._state = TaskState.IDLE
        self._settled(active, active.crash_outcome or NavOutcome(NavStatus.CANCELED))
        return active

    async def _cancel(self, task_id: str | None, generation: int | None) -> bool:
        async with self._transition_lock:
            active = self._active
            if (active is None or (task_id is not None and task_id != active.request.task_id)
                    or (generation is not None and generation != active.generation)):
                return False
            canceled = await self._cancel_active()
            if canceled.crash_outcome is None:
                self._emit(canceled, EventKind.PROGRESS, status='paused')
            return True

    async def _close(self) -> None:
        async with self._transition_lock:
            if self._active is not None:
                await self._cancel_active()
            self._state = TaskState.CLOSED
            self._subscribers.clear()

    async def _recover_crash(self, active: _Active) -> None:
        async with self._transition_lock:
            # A queued cancel/preempt/close may already have released this owner.
            if self._active is active:
                await self._cancel_active()

    def _on_crash(self, name: str, error: BaseException) -> None:
        if not isinstance(error, NavTaskError):
            _LOGGER.error('Task operation %s failed: %s', name, error)
        active = self._active
        if (active is None or active.worker is None or active.crash is not None
                or active.worker.get_name() != name or not active.worker.done()
                or active.worker.cancelled() or active.worker.exception() is not error):
            return
        active.crash = error
        active.accept_feedback = False
        if active.cancel_requested:
            # Explicit cancellation owns this attempt. Retain the exception so a
            # failed attempt can be retried, without inventing another terminal.
            return
        detail = f'执行器内部错误: {type(error).__name__}'
        active.crash_outcome = NavOutcome(NavStatus.FAILED, detail)
        self._state = TaskState.ERROR
        self._emit(active, EventKind.FAILED, detail=detail)
        self._observe(active, 'task_fault', error_type=type(error).__name__)
        self._supervisor.spawn(
            self._recover_crash(active),
            f'crash-cleanup:{active.request.task_id}:{active.generation}')
