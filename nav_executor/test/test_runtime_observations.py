"""Frozen logging and status cadence without ROS, sockets or wall-clock waits."""

import asyncio
from dataclasses import replace
import json
from types import SimpleNamespace

import pytest

from nav_executor.adapters.transport.outbox import ReliableOutbox
from nav_executor.app.logging import log_line, OutboundAudit
from nav_executor.app.status import format_status_line, print_status_loop
from nav_tasks.api import PoseRecord, TaskSnapshot, TaskState


def _poses():
    return SimpleNamespace(current_map_pose=lambda: PoseRecord('world', 1, 2, 0, 0.5, 0))


def test_frozen_outbound_error_line_uses_legacy_prefix(capsys):
    """The default sink matches the existing error audit including its newline."""
    OutboundAudit().on_sending(json.dumps({
        'event_type': 'on_error', 'request_id': 'req_1', 'sub_id': 1,
        'error_message': '目标点不存在: X',
    }))
    assert capsys.readouterr().out == (
        '[navigation_executor] NAV ERROR reported to server: request_id=req_1 sub_id=1 '
        'error_message=目标点不存在: X\n')
    log_line('test')
    assert capsys.readouterr().out == '[navigation_executor] test\n'


@pytest.mark.parametrize('event,resend,expected', [
    ('on_progress', False, []),
    ('on_mark_poi_ack', False, []),
    ('on_arrived', False, ['OUT sent: on_arrived request_id=r sub_id=2']),
    ('on_error', False, ['OUT sent: on_error request_id=r sub_id=2']),
    ('on_progress', True, ['OUT RESENT(断联重发): on_progress request_id=r sub_id=2']),
])
def test_success_audit_only_logs_terminal_or_replayed_frames(event, resend, expected):
    """Healthy progress/admin traffic remains quiet; replay is always visible."""
    lines = []
    OutboundAudit(lines.append).on_sent(json.dumps({
        'event_type': event, 'request_id': 'r', 'sub_id': 2}), resend)
    assert lines == expected


def test_type_fallback_and_missing_identity_keep_legacy_text():
    """Replay of a non-event frame retains type and None identity formatting."""
    lines = []
    OutboundAudit(lines.append).on_sent('{"type": "ping"}', True)
    assert lines == ['OUT RESENT(断联重发): ping request_id=None sub_id=None']


def test_failed_send_and_reconnect_have_exact_audit_order():
    """Error audit precedes each send; success audit only follows successful replay."""
    async def scenario():
        lines = []
        audit = OutboundAudit(lines.append)
        outbox = ReliableOutbox(on_sending=audit.on_sending, on_sent=audit.on_sent)
        outbox.put({'event_type': 'on_error', 'request_id': 'r', 'sub_id': 2,
                    'error_message': 'failure'})

        async def failed(encoded):
            lines.append('failed send')
            raise ConnectionError

        async def sent(encoded):
            lines.append('successful send')

        with pytest.raises(ConnectionError):
            await outbox.drain(failed)
        task = asyncio.create_task(outbox.drain(sent))
        await asyncio.sleep(0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        before = 'NAV ERROR reported to server: request_id=r sub_id=2 error_message=failure'
        assert lines == [before, 'failed send', before, 'successful send',
                         'OUT RESENT(断联重发): on_error request_id=r sub_id=2']

    asyncio.run(scenario())


def test_pose_failure_is_visible_without_losing_task_context():
    """Pose-read exceptions retain the old exception-class marker."""
    def unavailable():
        raise LookupError('TF not ready')

    snapshot = TaskSnapshot(TaskState.NAVIGATING, 'r', target_id='p', phase='to_anchor')
    assert format_status_line(snapshot, SimpleNamespace(current_map_pose=unavailable)) == (
        'STATUS state=navigating poi=p phase=to_anchor d2g=0.00m pose=n/a (LookupError)')


def test_status_cadence_retains_transition_and_active_only_heartbeat():
    """One snapshot per tick, no idle spam, no reset solely from reused request IDs."""
    async def scenario():
        idle = TaskSnapshot(TaskState.IDLE)
        active = TaskSnapshot(TaskState.NAVIGATING, 'a', target_id='p', phase='to_snap')
        sequence = [idle, idle, active, replace(active, task_id='b'), active, active,
                    active, replace(active, phase='to_anchor'), idle, idle]
        tick = -1
        lines = []
        polls = []
        reads = []

        async def sleep(seconds):
            nonlocal tick
            polls.append(seconds)
            tick += 1
            if tick == len(sequence):
                raise asyncio.CancelledError

        def status():
            reads.append(tick)
            return sequence[tick]

        with pytest.raises(asyncio.CancelledError):
            await print_status_loop(
                SimpleNamespace(status=status), _poses(), 1.0,
                lambda line: lines.append((tick, line)), now=lambda: tick * 0.25, sleep=sleep)
        assert [index for index, _ in lines] == [0, 2, 6, 7, 8]
        assert reads == list(range(len(sequence)))
        assert polls == [0.25] * (len(sequence) + 1)

    asyncio.run(scenario())
