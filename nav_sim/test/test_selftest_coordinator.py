"""Coordinator sequencing with explicit memory ports; these are not Nav2 executions."""

import asyncio
from dataclasses import replace
import json
from types import SimpleNamespace

import pytest

from nav_sim.selftest_coordinator import drive, Peer
from nav_sim.selftest_trace import EvidenceTrace
from test_selftest_evidence import CASES, _trace


def _ports(tmp_path, fault=None):
    cases = (replace(CASES[0], goal=(1, 0, 0)), CASES[1])
    scene = SimpleNamespace(root=tmp_path, namespace='private', cases=cases,
                            graph=SimpleNamespace(processes=(
                                SimpleNamespace(name='controller', ros_node=True),)))
    row = dict(event='start', pid=100, name='controller-1', node='/private/controller')
    (tmp_path / 'process_events.jsonl').write_text(json.dumps(row) + '\n')
    state = SimpleNamespace(pose=(0, 0, 0), odom=(0, 0), sent=[])
    trace = EvidenceTrace(tmp_path)
    observer = SimpleNamespace(step=lambda: None, ready=lambda: fault != 'not_ready',
                               pose=lambda: state.pose, odometry=lambda: state.odom)
    nav, ws = _trace()

    async def send(raw):
        command = json.loads(raw)
        index = len(state.sent)
        assert command['action'] == 'navigate_to'
        assert command['target_id'] == cases[index].target_id
        state.sent.append(command)
        if fault != 'wrong_pose':
            state.pose = cases[index].goal
        if fault != 'no_displacement':
            state.odom = (index + 1, 0)
        for event in nav:
            if event['data']['task_id'] == cases[index].task_id:
                if fault == 'missing_dock' and event['source'] == 'dock':
                    continue
                if fault == 'task_failure' and event['event_type'] == 'task_succeeded':
                    event['event_type'] = 'task_failed'
                trace.receive('nav', json.dumps(event))
        frame = dict(ws[index])
        if fault == 'wrong_subtask':
            frame['sub_id'] += 10
        trace.receive('ws', json.dumps(frame))

    peer = SimpleNamespace(socket=SimpleNamespace(send=send), error=None)
    child = SimpleNamespace(poll=lambda: 0 if fault == 'early_exit' else None)
    return scene, trace, peer, observer, child, state


def test_coordinator_orders_both_tasks_and_requires_independent_pose(tmp_path):
    """Actual oracle checks are retained while only the transport/motion ports are fake."""
    scene, trace, peer, observer, child, state = _ports(tmp_path)
    with trace:
        measurements = asyncio.run(drive(scene, trace, peer, observer, child, 1, False))
    assert [row['task_id'] for row in measurements] == [case.task_id for case in scene.cases]
    assert [row['pose'] for row in measurements] == [(1, 0, 0), (2, 0, 0)]
    assert all(row['odometry_displacement'] == 1 for row in measurements)
    assert len(state.sent) == 2


@pytest.mark.parametrize('fault', [
    'not_ready', 'wrong_pose', 'no_displacement',
    'missing_dock', 'wrong_subtask', 'early_exit', 'task_failure'])
def test_success_frames_cannot_hide_missing_real_evidence(tmp_path, fault):
    """No readiness, wrong pose, no motion, bad events or premature exit cannot pass."""
    scene, trace, peer, observer, child, state = _ports(tmp_path, fault)
    with trace, pytest.raises((RuntimeError, TimeoutError)):
        asyncio.run(drive(scene, trace, peer, observer, child, 0.1, False))
    if fault in ('not_ready', 'early_exit'):
        assert not state.sent
    elif fault in ('wrong_pose', 'no_displacement', 'task_failure', 'wrong_subtask'):
        assert len(state.sent) == 1


def test_startup_only_never_sends_a_command_or_claims_measurements(tmp_path):
    """A fully ready fake port cannot make startup-only dispatch a goal."""
    scene, trace, peer, observer, child, state = _ports(tmp_path)
    with trace:
        assert asyncio.run(drive(scene, trace, peer, observer, child, 1, True)) == []
    assert not state.sent and not trace.nav and not trace.ws


def test_peer_disconnect_reconnect_and_transport_errors_are_sticky(tmp_path):
    """Reconnection cannot replay the experiment, even after a normal first close."""
    class Socket:
        def __init__(self, frames):
            self.frames = iter(frames)
            self.sent = []
            self.closed = False

        def __aiter__(self):
            return self

        async def __anext__(self):
            try:
                return next(self.frames)
            except StopIteration:
                raise StopAsyncIteration

        async def send(self, frame):
            self.sent.append(json.loads(frame))

        async def close(self, **_kwargs):
            self.closed = True

    peer = Peer()
    with EvidenceTrace(tmp_path) as trace:
        peer.trace = trace
        socket = Socket(['{"type":"ping"}'])
        asyncio.run(peer.handle(socket))
        assert socket.sent == [{'type': 'pong'}]
        assert peer.error == 'client_disconnected'
        other = Socket([])
        asyncio.run(peer.handle(other))
        assert other.closed and peer.error == 'unexpected_or_reconnected_client'
        assert len(trace.ws) == 1
