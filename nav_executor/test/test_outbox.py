"""Reliable delivery ownership without sockets or external servers."""

import asyncio
import json

import pytest

from nav_executor.adapters.transport.outbox import ReliableOutbox


async def _cancel(task):
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task


def test_put_freezes_payload_and_uses_legacy_unicode_json_order():
    """Caller mutation after enqueue cannot rewrite an in-flight terminal frame."""
    async def scenario():
        outbox = ReliableOutbox()
        frame = {'event_type': 'on_mark_poi_success', 'poi': {'id': 'p', 'name': '大厅'}}
        outbox.put(frame)
        frame['poi']['name'] = 'changed'
        received = asyncio.Queue()

        async def send(encoded):
            received.put_nowait(encoded)

        task = asyncio.create_task(outbox.drain(send))
        assert await received.get() == (
            '{"event_type": "on_mark_poi_success", "poi": {"id": "p", "name": "大厅"}}')
        assert outbox.pending_count == 0
        await _cancel(task)
        with pytest.raises(TypeError):
            outbox.put({'bad': object()})
        assert outbox.pending_count == 0

    asyncio.run(scenario())


def test_cancel_during_send_retains_pending_ahead_of_new_frames():
    """Cancellation has the same retained-pending contract as transport failure."""
    async def scenario():
        audit = []
        outbox = ReliableOutbox(on_sent=lambda encoded, replay: audit.append(replay))
        outbox.put({'id': 1})
        entered = asyncio.Event()

        async def interrupted_send(encoded):
            entered.set()
            await asyncio.Event().wait()

        first = asyncio.create_task(outbox.drain(interrupted_send))
        await entered.wait()
        assert outbox.pending_count == 1
        await _cancel(first)
        outbox.put({'id': 2})
        received = asyncio.Queue()

        async def send(encoded):
            received.put_nowait(json.loads(encoded))

        second = asyncio.create_task(outbox.drain(send))
        assert await received.get() == {'id': 1}
        assert await received.get() == {'id': 2}
        assert audit == [True, False]
        assert outbox.pending_count == 0
        await _cancel(second)

    asyncio.run(scenario())


def test_competing_drainers_are_serialized_across_cancellation():
    """A second sender cannot bypass a still-owned first send."""
    async def scenario():
        outbox = ReliableOutbox()
        outbox.put({'id': 1})
        entered = asyncio.Event()
        received = asyncio.Queue()

        async def blocked_send(encoded):
            entered.set()
            await asyncio.Event().wait()

        async def new_send(encoded):
            received.put_nowait(encoded)

        first = asyncio.create_task(outbox.drain(blocked_send))
        await entered.wait()
        second = asyncio.create_task(outbox.drain(new_send))
        outbox.put({'id': 2})
        await asyncio.sleep(0)
        assert received.empty()
        await _cancel(first)
        assert await received.get() == '{"id": 1}'
        assert await received.get() == '{"id": 2}'
        await _cancel(second)

    asyncio.run(scenario())


def test_successful_send_is_not_repeated_when_post_send_logging_fails():
    """Audit callback failure cannot put an already-sent frame back into pending."""
    async def scenario():
        def broken_audit(encoded, replay):
            raise RuntimeError('logging failed')

        outbox = ReliableOutbox(on_sent=broken_audit)
        outbox.put({'id': 1})
        sent = []

        async def send(encoded):
            sent.append(encoded)

        with pytest.raises(RuntimeError, match='logging failed'):
            await outbox.drain(send)
        assert outbox.pending_count == 0
        assert sent == ['{"id": 1}']

    asyncio.run(scenario())


def test_pre_send_logging_failure_retains_pending_and_marks_replay():
    """Failure before send leaves a retained pending frame, matching the legacy holder."""
    async def scenario():
        attempts = []
        audit = []

        def log_attempt(encoded):
            attempts.append(encoded)
            if len(attempts) == 1:
                raise RuntimeError('before send')

        outbox = ReliableOutbox(
            on_sending=log_attempt, on_sent=lambda encoded, replay: audit.append(replay))
        outbox.put({'id': 1})
        received = asyncio.Queue()

        async def send(encoded):
            received.put_nowait(encoded)

        with pytest.raises(RuntimeError, match='before send'):
            await outbox.drain(send)
        assert received.empty()
        assert outbox.pending_count == 1
        task = asyncio.create_task(outbox.drain(send))
        assert await received.get() == '{"id": 1}'
        assert audit == [True]
        await _cancel(task)

    asyncio.run(scenario())


def test_outbox_refuses_cross_loop_use():
    """The same outbox cannot be moved to an unrelated asyncio runtime."""
    async def construct():
        return ReliableOutbox()

    outbox = asyncio.run(construct())

    async def wrong_loop():
        with pytest.raises(RuntimeError, match='different event loop'):
            outbox.put({'id': 1})

    asyncio.run(wrong_loop())
