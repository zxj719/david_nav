"""Local action deadlines never discard late or still-running motion ownership."""

import asyncio
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from nav_tasks.adapters.ros.action_session import ActionSession, ActionState
from nav_tasks.domain.errors import BusyError, ConfigError
from test_action_session import _Client, _done, _Handle, _settle


@pytest.mark.parametrize('field', ['send_timeout_sec', 'result_timeout_sec'])
@pytest.mark.parametrize('value', [-1, float('nan'), float('inf'), True, '10'])
def test_invalid_phase_deadline_rejected_before_submission(field, value):
    """Invalid opt-in deadlines leave the session idle and never submit."""
    async def scenario():
        session = ActionSession()
        client = _Client()
        with pytest.raises(ConfigError):
            await session.execute(client, object, lambda raw: None,
                                  server_timeout_sec=0, **{field: value})
        assert not session.busy
        assert client.goals == []
        await session.close()

    asyncio.run(scenario())


def test_send_deadline_retains_late_handle_ack_and_terminal_barriers():
    """Admission timeout is not returned as a settled failure while motion is owned."""
    async def scenario():
        send = Future()
        client = _Client(send)
        session = ActionSession(cancel_timeout_sec=1)
        operation = asyncio.create_task(session.execute(
            client, object, lambda raw: None, server_timeout_sec=0, send_timeout_sec=0))
        await client.sent.wait()
        await _settle()
        assert session.busy and not operation.done()
        assert not send.cancelled()
        with pytest.raises(BusyError):
            await session.execute(_Client(), object, lambda raw: None, server_timeout_sec=0)
        handle = _Handle(cancel_status=None)
        handle.cancel_future = Future()
        send.set_result(handle)
        await handle.cancel_started.wait()
        handle.result_future.set_result(SimpleNamespace(status=5))
        await _settle()
        assert session.busy and not operation.done()
        handle.cancel_future.set_result(None)
        result = await operation
        assert result.state == ActionState.SEND_TIMEOUT
        assert not session.busy
        assert handle.cancel_calls == 1
        await session.close()

    asyncio.run(scenario())


def test_result_deadline_cancels_and_requires_terminal_after_ack():
    """The result budget begins after admission and cannot release on cancel ack alone."""
    async def scenario():
        handle = _Handle(cancel_status=None)
        session = ActionSession(cancel_timeout_sec=1)
        operation = asyncio.create_task(session.execute(
            _Client(_done(handle)), object, lambda raw: None,
            server_timeout_sec=0, result_timeout_sec=0))
        await handle.cancel_started.wait()
        await _settle()
        assert session.busy and not operation.done()
        assert not handle.result_future.cancelled()
        handle.result_future.set_result(SimpleNamespace(status=5))
        assert (await operation).state == ActionState.RESULT_TIMEOUT
        assert not session.busy
        await session.close()

    asyncio.run(scenario())


def test_cleanup_timeout_is_exception_not_settled_action_failure():
    """A worker cannot continue to its next segment when cleanup times out."""
    async def scenario():
        handle = _Handle(cancel_status=None)
        session = ActionSession(cancel_timeout_sec=0.01)
        with pytest.raises(asyncio.TimeoutError):
            await session.execute(_Client(_done(handle)), object, lambda raw: None,
                                  server_timeout_sec=0, result_timeout_sec=0)
        assert session.busy
        assert session._supervisor.active_count == 1
        handle.result_future.set_result(SimpleNamespace(status=5))
        await session.cancel()
        assert not session.busy
        await session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('phase', ['send', 'result'])
@pytest.mark.parametrize('pending', [False, True])
def test_source_timeout_error_is_not_a_local_deadline(phase, pending):
    """A transport TimeoutError retains ownership and its original identity."""
    async def scenario():
        failed = Future()
        error = TimeoutError('transport failed')
        if not pending:
            failed.set_exception(error)
        handle = _Handle()
        client = _Client(failed if phase == 'send' else _done(handle))
        if phase == 'result':
            handle.result_future = failed
        session = ActionSession()
        operation = asyncio.create_task(session.execute(
            client, object, lambda raw: None, server_timeout_sec=0,
            send_timeout_sec=1, result_timeout_sec=1))
        if pending:
            await client.sent.wait()
            await _settle()
            failed.set_exception(error)
        with pytest.raises(TimeoutError) as caught:
            await operation
        assert caught.value is error
        assert session.busy
        # The fake transport recovers; close still has to prove a terminal result.
        if phase == 'send':
            session._active.send = _done(handle)
        else:
            handle.result_future = _done(SimpleNamespace(status=5))
        await session.close()

    asyncio.run(scenario())


def test_positive_budgets_allow_separate_admission_and_result_completion():
    """Normal pending futures still produce their terminal response without cancellation."""
    async def scenario():
        send = Future()
        client = _Client(send)
        session = ActionSession()
        operation = asyncio.create_task(session.execute(
            client, object, lambda raw: None, server_timeout_sec=0,
            send_timeout_sec=30, result_timeout_sec=30))
        await client.sent.wait()
        await _settle()
        handle = _Handle(cancel_status=None)
        send.set_result(handle)
        await _settle()
        assert handle.result_calls == 1
        handle.result_future.set_result(SimpleNamespace(status=4))
        assert (await operation).response.status == 4
        assert handle.cancel_calls == 0
        assert not session.busy
        await session.close()
        await _settle()
        assert session._supervisor.active_count == 0

    asyncio.run(scenario())


def test_deadline_fences_feedback_until_cleanup_settles():
    """A timed-out action cannot keep reporting progress while cancel is pending."""
    async def scenario():
        handle = _Handle(cancel_status=None)
        client = _Client(_done(handle))
        feedback = []
        session = ActionSession()
        operation = asyncio.create_task(session.execute(
            client, object, feedback.append, server_timeout_sec=0, result_timeout_sec=0))
        await handle.cancel_started.wait()
        client.feedback('late')
        await _settle()
        assert feedback == []
        handle.result_future.set_result(SimpleNamespace(status=5))
        assert (await operation).state == ActionState.RESULT_TIMEOUT
        await session.close()

    asyncio.run(scenario())


@pytest.mark.parametrize('status', [4, 5, 6])
def test_already_completed_futures_win_even_with_zero_deadlines(status):
    """Immediate responses preserve the old wait_future done-before-timeout behavior."""
    async def scenario():
        session = ActionSession()
        handle = _Handle(status=status)
        result = await session.execute(_Client(_done(handle)), object, lambda raw: None,
                                       server_timeout_sec=0, send_timeout_sec=0,
                                       result_timeout_sec=0)
        assert result.state == ActionState.TERMINAL
        assert result.response.status == status
        assert handle.cancel_calls == 0
        assert not session.busy
        await session.close()

    asyncio.run(scenario())


def test_explicit_cancel_with_deadlines_still_waits_for_cleanup():
    """Canceling a timed wait never cancels the underlying callback future."""
    async def scenario():
        send = Future()
        client = _Client(send)
        session = ActionSession()
        operation = asyncio.create_task(session.execute(
            client, object, lambda raw: None, server_timeout_sec=0, send_timeout_sec=30))
        await client.sent.wait()
        operation.cancel()
        await _settle()
        assert not send.cancelled()
        assert session.busy
        handle = _Handle()
        send.set_result(handle)
        with pytest.raises(asyncio.CancelledError):
            await operation
        assert handle.cancel_calls == 1
        assert not session.busy
        await session.close()

    asyncio.run(scenario())
