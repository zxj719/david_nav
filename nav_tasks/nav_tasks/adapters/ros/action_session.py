"""Single-slot ROS action ownership, including late-handle cancellation cleanup."""

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
    """Local wait expiration, distinct from errors returned by the ROS future."""


class ActionClientLike(Protocol):
    """Action client operations needed without importing rclpy at module load."""

    def server_is_ready(self) -> bool:
        """Probe availability without blocking the event loop.

        Returns:
            Whether the action server is currently ready.
        """
        ...

    def send_goal_async(
        self, goal: object, feedback_callback: Callable[[object], None],
    ) -> FutureLike:
        """Send a goal asynchronously.

        Args:
            goal: Action-specific goal message.
            feedback_callback: Raw feedback callback on the ROS executor thread.

        Returns:
            Future resolving to a goal handle or rejection.
        """
        ...


class ActionState(str, Enum):
    """Single-attempt result before action-specific domain translation.

    Attributes:
        REJECTED: 服务明确拒绝目标。
        RESULT_TIMEOUT: 等待执行结果超过期限。
        SEND_TIMEOUT: 发送/接受目标阶段超过期限。
        TERMINAL: 收到 ROS 动作终态响应，具体状态见 response。
        UNAVAILABLE: 期限内未发现动作服务。
    """

    UNAVAILABLE = 'unavailable'
    REJECTED = 'rejected'
    TERMINAL = 'terminal'
    SEND_TIMEOUT = 'send_timeout'
    RESULT_TIMEOUT = 'result_timeout'


@dataclass(frozen=True)
class ActionOutcome:
    """Single-attempt result and optional ROS terminal response.

    Attributes:
        response: 可选的 ROS 终态响应；无响应时为 None。
        state: 动作会话单次尝试的状态分类。
    """

    state: ActionState
    response: object | None = None


@dataclass
class _Attempt:
    on_feedback: Callable[[object], None]
    submission_started: bool = False
    send: FutureLike | None = None
    handle: object | None = None
    result: FutureLike | None = None
    cancel_requested: bool = False
    settled: bool = False
    cleanup: asyncio.Task | None = None


class ActionSession:
    """Own one action at a time across all clients sharing this session.

    Cancellation waits for admission, cancel acknowledgement and a terminal
    result. A caller timeout does not discard cleanup: the supervisor retains
    it so a late accepted handle is still canceled. The slot remains busy until
    ownership is proven settled, never merely because a waiter was canceled.
    """

    def __init__(
        self, *, cancel_timeout_sec: float = 5.0, server_poll_sec: float = 0.05,
        clock: Clock | None = None,
    ) -> None:
        """Construct on the asyncio loop that owns action state.

        Args:
            cancel_timeout_sec: Per-caller cleanup wait; timeout retains owned cleanup.
            server_poll_sec: Nonblocking readiness probe interval while submitting.
            clock: Injectable monotonic clock for readiness waits.

        Raises:
            ConfigError: Wait intervals are nonfinite or invalid.
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
        """Whether an attempt still owns the shared action slot."""
        return self._active is not None

    async def execute(
        self, client: ActionClientLike, goal_factory: Callable[[], object],
        on_feedback: Callable[[object], None],
        *, server_timeout_sec: float, send_timeout_sec: float | None = None,
        result_timeout_sec: float | None = None,
        on_timeout: Callable[[ActionState], None] | None = None,
    ) -> ActionOutcome:
        """Execute one goal, returning only a confirmed terminal/rejection outcome.

        Args:
            client: Action client owned by the runtime, using its existing executor.
            goal_factory: Build a fresh stamped message after the server is ready.
            on_feedback: Loop-thread callback, never called after cancellation starts.
            server_timeout_sec: Maximum nonblocking wait for server availability.
            send_timeout_sec: Optional admission wait budget, separate from cleanup.
            result_timeout_sec: Optional result wait budget starting after admission.
            on_timeout: Optional synchronous audit hook before deadline cleanup begins.

        Returns:
            Unavailable, rejected, terminal, or a timeout after confirmed cleanup.

        Raises:
            BusyError: Another action or unresolved cleanup owns the slot.
            ClosedError: Session admission has been sealed.
            ConfigError: A wait duration is invalid.
            asyncio.CancelledError: The caller canceled; cleanup is attempted first.
            RuntimeError: The server returned a nonterminal result response.
            TimeoutError: Cancellation wait expired; owned cleanup continues.
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
        """Cancel and settle the current action, or do nothing when idle.

        Raises:
            TimeoutError: Cleanup is still owned and running after the caller deadline.
            Exception: Cleanup failed; ownership remains busy and can be retried.
        """
        self._check_loop()
        if self._active is not None:
            await self._cancel_attempt(self._active)

    async def close(self) -> None:
        """Seal new actions and settle owned work before clients may be destroyed.

        Raises:
            Exception: Cleanup did not complete; do not destroy the ROS clients yet.
        """
        self._check_loop()
        self._closed = True
        await self.cancel()

    def _check_loop(self) -> None:
        if asyncio.get_running_loop() is not self._loop:
            raise RuntimeError('action session used from a different event loop')

    async def _wait_phase(self, future: FutureLike, timeout: float | None) -> object:
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
        attempt.cancel_requested = True
        if attempt.settled:
            return
        if attempt.cleanup is None or attempt.cleanup.done():
            attempt.cleanup = self._supervisor.spawn(self._cleanup(attempt), 'action-cleanup')
        await asyncio.wait_for(asyncio.shield(attempt.cleanup), timeout=self._cancel_timeout)

    async def _cleanup(self, attempt: _Attempt) -> None:
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
        try:
            self._loop.call_soon_threadsafe(self._feedback, attempt, message)
        except RuntimeError:
            # Runtime shutdown owns cleanup; a closed loop cannot consume feedback.
            _LOGGER.debug('Ignoring action feedback after the event loop closed')

    def _feedback(self, attempt: _Attempt, message: object) -> None:
        if self._active is attempt and not attempt.cancel_requested and not attempt.settled:
            attempt.on_feedback(message)

    @staticmethod
    def _on_crash(name: str, error: BaseException) -> None:
        _LOGGER.error('Owned %s failed; action remains busy: %s', name, error)
