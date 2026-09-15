"""Deterministic fake-only tests for registry, retries and background ownership."""

import asyncio
import gc
import inspect
import weakref

import pytest

from nav_tasks.core.registry import CommandRegistry
from nav_tasks.core.retry import MonotonicClock, RetryPolicy
from nav_tasks.core.supervisor import TaskSupervisor
from nav_tasks.domain.errors import ClosedError, ConfigError, UnknownActionError
from nav_tasks.domain.tasks import NavOutcome, NavStatus


class _Clock:
    def __init__(self):
        self.delays = []
        self.entered = asyncio.Event()
        self.release = None

    async def sleep(self, seconds):
        self.delays.append(seconds)
        self.entered.set()
        if self.release is not None:
            await self.release.wait()


@pytest.mark.parametrize('statuses,attempts,delays,detail', [
    ([NavStatus.SUCCEEDED], 1, [], ''),
    ([NavStatus.CANCELED], 1, [], ''),
    ([NavStatus.FAILED, NavStatus.SUCCEEDED], 2, [1.0], ''),
    ([NavStatus.FAILED, NavStatus.CANCELED], 2, [1.0], ''),
    ([NavStatus.FAILED, NavStatus.FAILED], 2, [1.0], '导航失败(共尝试2次): blocked'),
])
def test_retry_outcomes(statuses, attempts, delays, detail):
    """Only FAILED consumes the retry budget and exhaustion text stays exact."""
    async def scenario():
        clock = _Clock()
        calls = []

        async def attempt():
            status = statuses[len(calls)]
            calls.append(status)
            return NavOutcome(status, 'blocked' if status == NavStatus.FAILED else '')

        result = await RetryPolicy().run(attempt, clock)
        assert len(calls) == attempts
        assert clock.delays == delays
        assert result.status == statuses[-1]
        assert result.detail == detail

    asyncio.run(scenario())


@pytest.mark.parametrize('count,delay', [
    (-1, 1), (True, 1), (1.5, 1), (1, -1), (1, float('nan')), (1, float('inf')),
])
def test_invalid_retry_configuration(count, delay):
    """Reject invalid budgets before a backend can be called."""
    with pytest.raises(ConfigError):
        RetryPolicy(count, delay)


def test_cancel_during_retry_delay_never_resubmits():
    """Cancellation seals the failure-to-retry window without wall-clock sleeps."""
    async def scenario():
        clock = _Clock()
        clock.release = asyncio.Event()
        calls = []

        async def attempt():
            calls.append('attempt')
            return NavOutcome(NavStatus.FAILED, 'failure')

        task = asyncio.create_task(RetryPolicy().run(attempt, clock))
        await clock.entered.wait()
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        clock.release.set()
        await asyncio.sleep(0)
        assert calls == ['attempt']

    asyncio.run(scenario())


def test_zero_retry_and_unexpected_exception():
    """Even zero retries wraps a failure; exceptions are not navigation failures."""
    async def scenario():
        clock = _Clock()

        async def fail():
            return NavOutcome(NavStatus.FAILED, 'detail')

        async def crash():
            raise RuntimeError('programming error')

        assert (await RetryPolicy(0).run(fail, clock)).detail == '导航失败(共尝试1次): detail'
        with pytest.raises(RuntimeError, match='programming error'):
            await RetryPolicy().run(crash, clock)
        assert clock.delays == []
        actual_clock = MonotonicClock()
        before = actual_clock.now()
        await actual_clock.sleep(0)
        assert actual_clock.now() >= before

    asyncio.run(scenario())


def test_registry_extension_duplicate_unknown_and_decoder_error():
    """An extra action requires registration only and cannot replace another one."""
    registry = CommandRegistry()

    async def handler(context, value):
        return NavOutcome(NavStatus.SUCCEEDED, str(value))

    registry.register('fixture', lambda values: int(values['number']), handler)
    assert registry.prepare('fixture', {'number': '3'}) == (handler, 3)
    with pytest.raises(ValueError):
        registry.register('fixture', lambda values: values, handler)
    with pytest.raises(ValueError):
        registry.register('', lambda values: values, handler)
    with pytest.raises(ValueError):
        registry.register('invalid', None, handler)
    with pytest.raises(UnknownActionError):
        registry.prepare('missing', {})
    with pytest.raises(ValueError):
        registry.prepare('fixture', {'number': 'invalid'})


def test_supervisor_keeps_tasks_alive_and_retrieves_failures_once():
    """GC cannot discard a pending task; failures reach the owner exactly once."""
    async def scenario():
        crashes = []
        supervisor = TaskSupervisor(lambda name, error: crashes.append((name, str(error))))
        ready = asyncio.Event()
        release = asyncio.Event()

        async def work():
            ready.set()
            await release.wait()
            raise ValueError('boom')

        task = supervisor.spawn(work(), 'owned')
        reference = weakref.ref(task)
        await ready.wait()
        del task
        gc.collect()
        assert reference() is not None
        assert supervisor.active_count == 1
        release.set()
        with pytest.raises(ValueError):
            await reference()
        await asyncio.sleep(0)
        assert supervisor.active_count == 0
        assert crashes == [('owned', 'boom')]
        await supervisor.cancel_and_wait()

    asyncio.run(scenario())


def test_supervisor_close_waits_for_finally_and_closes_rejected_coroutine():
    """Drain cleanup and avoid a never-awaited coroutine warning after close."""
    async def scenario():
        crashes = []
        supervisor = TaskSupervisor(lambda *values: crashes.append(values))
        started = asyncio.Event()
        cleaned = []

        async def work():
            try:
                started.set()
                await asyncio.Event().wait()
            finally:
                await asyncio.sleep(0)
                cleaned.append(True)

        supervisor.spawn(work(), 'waiter')
        await started.wait()
        await supervisor.cancel_and_wait()
        assert cleaned == [True]
        assert crashes == []
        assert supervisor.active_count == 0
        rejected = work()
        with pytest.raises(ClosedError):
            supervisor.spawn(rejected, 'too-late')
        assert inspect.getcoroutinestate(rejected) == inspect.CORO_CLOSED
        await supervisor.cancel_and_wait()

    asyncio.run(scenario())


def test_supervisor_broken_crash_callback_reaches_loop_handler():
    """Diagnostic callback failures do not leave the original task unobserved."""
    async def scenario():
        observed = []
        loop = asyncio.get_running_loop()
        loop.set_exception_handler(lambda loop, context: observed.append(context))

        def broken(name, error):
            raise RuntimeError('callback')

        async def crash():
            raise ValueError('worker')

        supervisor = TaskSupervisor(broken)
        with pytest.raises(ValueError):
            await supervisor.spawn(crash(), 'crasher')
        await asyncio.sleep(0)
        assert len(observed) == 1
        assert str(observed[0]['exception']) == 'callback'
        assert str(observed[0]['original_exception']) == 'worker'
        await supervisor.cancel_and_wait()

    asyncio.run(scenario())


def test_retry_cancellation_in_flight_is_not_a_retryable_failure():
    """Cancellation of an attempt propagates without sleeping or trying again."""
    async def scenario():
        clock = _Clock()
        entered = asyncio.Event()
        calls = []

        async def attempt():
            calls.append(True)
            entered.set()
            await asyncio.Event().wait()

        task = asyncio.create_task(RetryPolicy().run(attempt, clock))
        await entered.wait()
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert calls == [True]
        assert clock.delays == []

    asyncio.run(scenario())


def test_supervisor_rejects_self_join_without_canceling_siblings():
    """A programming error cannot deadlock shutdown by awaiting its own task."""
    async def scenario():
        crashes = []
        supervisor = TaskSupervisor(lambda *values: crashes.append(values))

        async def self_join():
            with pytest.raises(RuntimeError, match='own supervisor'):
                await supervisor.cancel_and_wait()

        await supervisor.spawn(self_join(), 'self-join')
        await supervisor.cancel_and_wait()
        assert crashes == []

    asyncio.run(scenario())
