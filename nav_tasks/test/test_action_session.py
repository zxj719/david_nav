"""Manually sequenced action-future races without ROS or physical goals."""

import asyncio
from concurrent.futures import Future
import threading
from types import SimpleNamespace

import pytest

from nav_tasks.adapters.ros.action_session import ActionSession, ActionState
from nav_tasks.domain.errors import BusyError, ClosedError


def _done(value):
    future = Future()
    future.set_result(value)
    return future


class _Handle:
    def __init__(self, *, accepted=True, status=None, cancel_status=5):
        self.accepted = accepted
        self.result_future = Future() if status is None else _done(SimpleNamespace(status=status))
        self.cancel_future = _done(None)
        self.cancel_status = cancel_status
        self.cancel_calls = 0
        self.result_calls = 0
        self.cancel_started = asyncio.Event()

    def get_result_async(self):
        self.result_calls += 1
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        self.cancel_started.set()
        if self.cancel_status is not None and not self.result_future.done():
            self.result_future.set_result(SimpleNamespace(status=self.cancel_status))
        return self.cancel_future


class _Client:
    def __init__(self, send=None, ready=True):
        self.send = send if send is not None else _done(_Handle(status=4))
        self.ready = ready
        self.sent = asyncio.Event()
        self.goals = []
        self.feedback = None

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal, feedback_callback):
        self.goals.append(goal)
        self.feedback = feedback_callback
        self.sent.set()
        return self.send


async def _settle():
    for _ in range(8):
        await asyncio.sleep(0)


@pytest.mark.parametrize('status', [4, 5, 6])
def test_terminal_outcomes_release_ownership_without_extra_cancel(status):
    """A confirmed terminal result permits the next attempt."""
    async def scenario():
        session = ActionSession()
        handle = _Handle(status=status)
        client = _Client(_done(handle))
        result = await session.execute(client, object, lambda raw: None, server_timeout_sec=0)
        assert result.state == ActionState.TERMINAL
        assert result.response.status == status
        assert not session.busy
        await session.cancel()
        assert handle.cancel_calls == 0
        await session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('handle', [None, SimpleNamespace(accepted=False)])
def test_rejection_and_unavailability_are_not_owned_goals(handle):
    """No-goal outcomes free the slot without attempting cancellation."""
    async def scenario():
        session = ActionSession()
        client = _Client(_done(handle))
        outcome = await session.execute(client, object, lambda raw: None, server_timeout_sec=0)
        assert outcome.state == ActionState.REJECTED
        assert not session.busy
        unavailable = _Client(ready=False)
        outcome = await session.execute(
            unavailable, object, lambda raw: None, server_timeout_sec=0)
        assert outcome.state == ActionState.UNAVAILABLE
        assert unavailable.goals == []
        await session.close()

    asyncio.run(scenario())


def test_cancel_waits_for_late_handle_ack_and_terminal_result():
    """Both cancel callers share cleanup and cannot release any barrier early."""
    async def scenario():
        pending = Future()
        client = _Client(pending)
        session = ActionSession()
        navigation = asyncio.create_task(session.execute(
            client, object, lambda raw: None, server_timeout_sec=0))
        await client.sent.wait()
        navigation.cancel()
        cancel = asyncio.create_task(session.cancel())
        await _settle()
        assert session.busy
        assert not navigation.done()
        assert not cancel.done()
        with pytest.raises(BusyError):
            await session.execute(_Client(), object, lambda raw: None, server_timeout_sec=0)
        handle = _Handle(cancel_status=None)
        handle.cancel_future = Future()
        pending.set_result(handle)
        await handle.cancel_started.wait()
        assert handle.cancel_calls == 1
        assert handle.result_calls == 1
        assert not navigation.done()
        handle.cancel_future.set_result(None)
        await _settle()
        assert not cancel.done()
        assert session.busy
        handle.result_future.set_result(SimpleNamespace(status=5))
        with pytest.raises(asyncio.CancelledError):
            await navigation
        await cancel
        assert not session.busy
        assert handle.cancel_calls == 1
        await session.close()

    asyncio.run(scenario())


def test_cancel_timeout_retains_cleanup_for_a_late_goal():
    """A timed-out cancel caller cannot discard the future or admit replacement motion."""
    async def scenario():
        pending = Future()
        client = _Client(pending)
        session = ActionSession(cancel_timeout_sec=0.01)
        navigation = asyncio.create_task(session.execute(
            client, object, lambda raw: None, server_timeout_sec=0))
        await client.sent.wait()
        with pytest.raises(asyncio.TimeoutError):
            await session.cancel()
        assert session.busy
        assert session._supervisor.active_count == 1
        with pytest.raises(BusyError):
            await session.execute(_Client(), object, lambda raw: None, server_timeout_sec=0)
        handle = _Handle()
        pending.set_result(handle)
        await handle.cancel_started.wait()
        with pytest.raises(asyncio.CancelledError):
            await navigation
        await _settle()
        assert not session.busy
        assert handle.cancel_calls == 1
        assert session._supervisor.active_count == 0
        await session.close()

    asyncio.run(scenario())


def test_canceled_cancel_waiter_does_not_abandon_cleanup():
    """Disconnecting a cancellation waiter leaves cleanup strongly owned."""
    async def scenario():
        handle = _Handle(cancel_status=None)
        client = _Client(_done(handle))
        session = ActionSession()
        navigation = asyncio.create_task(session.execute(
            client, object, lambda raw: None, server_timeout_sec=0))
        await client.sent.wait()
        cancel = asyncio.create_task(session.cancel())
        await handle.cancel_started.wait()
        cancel.cancel()
        with pytest.raises(asyncio.CancelledError):
            await cancel
        assert session.busy
        handle.result_future.set_result(SimpleNamespace(status=5))
        try:
            await navigation
        except asyncio.CancelledError:
            pass
        await session.cancel()
        assert not session.busy
        await session.close()

    asyncio.run(scenario())


def test_feedback_runs_on_loop_and_is_fenced_after_cancel():
    """Thread callbacks become loop callbacks, and stale feedback never leaks."""
    async def scenario():
        handle = _Handle()
        client = _Client(_done(handle))
        session = ActionSession()
        events = asyncio.Queue()
        loop_thread = threading.get_ident()
        navigation = asyncio.create_task(session.execute(
            client, object, lambda raw: events.put_nowait((raw, threading.get_ident())),
            server_timeout_sec=0))
        await client.sent.wait()
        thread = threading.Thread(target=lambda: client.feedback('feedback'))
        thread.start()
        assert await events.get() == ('feedback', loop_thread)
        thread.join()
        await session.cancel()
        try:
            await navigation
        except asyncio.CancelledError:
            pass
        client.feedback('late')
        await _settle()
        assert events.empty()
        await session.close()

    asyncio.run(scenario())


def test_nonblocking_server_wait_can_be_canceled_before_submission():
    """Readiness waiting does not freeze cancellation or send a late goal."""
    async def scenario():
        entered = asyncio.Event()
        release = asyncio.Event()

        class Clock:
            def now(self):
                return 10

            async def sleep(self, seconds):
                entered.set()
                await release.wait()

        client = _Client(ready=False)
        session = ActionSession(clock=Clock())
        navigation = asyncio.create_task(session.execute(
            client, object, lambda raw: None, server_timeout_sec=5))
        await entered.wait()
        await session.cancel()
        client.ready = True
        release.set()
        with pytest.raises(asyncio.CancelledError):
            await navigation
        assert client.goals == []
        assert not session.busy
        await session.close()

    asyncio.run(scenario())


def test_failed_cancel_remains_busy_and_can_retry():
    """A cancellation-service failure cannot falsely release action ownership."""
    async def scenario():
        handle = _Handle(cancel_status=None)
        handle.cancel_future = Future()
        handle.cancel_future.set_exception(RuntimeError('cancel service failed'))
        client = _Client(_done(handle))
        session = ActionSession()
        navigation = asyncio.create_task(session.execute(
            client, object, lambda raw: None, server_timeout_sec=0))
        await client.sent.wait()
        # Let admission complete before injecting cancellation service failure.
        await _settle()
        with pytest.raises(RuntimeError, match='cancel service failed'):
            await session.cancel()
        assert session.busy
        handle.cancel_future = _done(None)
        handle.cancel_status = 5
        await session.cancel()
        assert (await navigation).response.status == 5
        assert handle.cancel_calls == 2
        assert not session.busy
        await session.close()

    asyncio.run(scenario())


def test_close_seals_admission_and_synchronous_send_failure_stays_owned():
    """Untrackable submission failure cannot be mistaken for a proven no-goal state."""
    async def scenario():
        session = ActionSession()

        class Client:
            def server_is_ready(self):
                return True

            def send_goal_async(self, goal, feedback_callback):
                raise RuntimeError('send uncertain')

        with pytest.raises(RuntimeError, match='send uncertain'):
            await session.execute(Client(), object, lambda raw: None, server_timeout_sec=0)
        assert session.busy
        with pytest.raises(RuntimeError, match='trackable future'):
            await session.close()
        assert session.busy
        with pytest.raises(ClosedError):
            await session.execute(_Client(), object, lambda raw: None, server_timeout_sec=0)

    asyncio.run(scenario())


@pytest.mark.parametrize('failure', [False, True])
def test_bad_result_keeps_ownership_and_cleanup_requests_a_fresh_terminal_result(failure):
    """An invalid/failed result future cannot stand in for a confirmed terminal state."""
    async def scenario():
        handle = _Handle(status=2)
        if failure:
            handle.result_future = Future()
            handle.result_future.set_exception(RuntimeError('result service failed'))
        client = _Client(_done(handle))
        session = ActionSession()
        with pytest.raises(RuntimeError):
            await session.execute(client, object, lambda raw: None, server_timeout_sec=0)
        assert session.busy
        handle.result_future = _done(SimpleNamespace(status=5))
        await session.cancel()
        assert handle.cancel_calls == 1
        assert handle.result_calls == 2
        assert not session.busy
        await session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('accepted,status', [(False, None), (True, 4)])
def test_late_rejection_or_already_terminal_handle_needs_no_cancel(accepted, status):
    """Cleanup proves absence of motion without canceling a rejected or finished goal."""
    async def scenario():
        pending = Future()
        client = _Client(pending)
        session = ActionSession()
        navigation = asyncio.create_task(session.execute(
            client, object, lambda raw: None, server_timeout_sec=0))
        await client.sent.wait()
        navigation.cancel()
        await _settle()
        handle = _Handle(accepted=accepted, status=status)
        pending.set_result(handle)
        with pytest.raises(asyncio.CancelledError):
            await navigation
        assert handle.cancel_calls == 0
        assert not session.busy
        await session.close()

    asyncio.run(scenario())


def test_terminal_before_cancel_ack_keeps_slot_and_close_waiting():
    """Reversed response order cannot release ownership or bypass pending cleanup."""
    async def scenario():
        handle = _Handle(cancel_status=None)
        handle.cancel_future = Future()
        client = _Client(_done(handle))
        session = ActionSession()
        navigation = asyncio.create_task(session.execute(
            client, object, lambda raw: None, server_timeout_sec=0))
        await client.sent.wait()
        await _settle()
        cancel = asyncio.create_task(session.cancel())
        await handle.cancel_started.wait()
        handle.result_future.set_result(SimpleNamespace(status=5))
        await _settle()
        assert session.busy
        assert not navigation.done()
        assert not cancel.done()
        close = asyncio.create_task(session.close())
        await _settle()
        assert not close.done()
        handle.cancel_future.set_result(None)
        assert (await navigation).response.status == 5
        await cancel
        await close
        assert not session.busy
        assert handle.cancel_calls == 1

    asyncio.run(scenario())
