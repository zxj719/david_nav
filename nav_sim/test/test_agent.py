"""WebSocket protocol scenarios without sockets, ROS nodes or navigation goals."""

import asyncio
import json

import pytest

from nav_sim.fake_agent_server import abort, AgentSession, load_script, main, navigate_to
from nav_sim.fake_agent_server import ScriptStep


class FakeSocket:
    """An in-memory socket with deterministic inbound frames."""

    def __init__(self, frames=()):
        """Keep a copy of incoming frames and capture outbound bytes."""
        self.frames = list(frames)
        self.sent = []

    async def send(self, frame):
        """Capture one frame without transmitting it."""
        self.sent.append(frame)

    async def recv(self):
        """Pop one frame or wait forever to exercise timeout handling."""
        if self.frames:
            return json.dumps(self.frames.pop(0))
        await asyncio.Future()

    def __aiter__(self):
        """Iterate over the remaining in-memory frames."""
        return self

    async def __anext__(self):
        """Read until the finite scenario ends."""
        if not self.frames:
            raise StopAsyncIteration
        return await self.recv()


def test_wire_bytes_and_abort_action():
    """V1 commands preserve exact action names and ordinary JSON serialization."""
    assert json.dumps(navigate_to('任务', 'poi-a', 2), ensure_ascii=False) == (
        '{"action": "navigate_to", "request_id": "任务", "sub_id": 2, "target_id": "poi-a"}')
    assert json.dumps(abort('task-a')) == (
        '{"action": "abort_navigation", "request_id": "task-a"}')


def test_event_barrier_matches_both_ids_and_retains_status_frames():
    """Other requests, subtasks and status messages cannot advance a scenario."""
    frames = [
        {'type': 'state_report', 'state': 'IDLE'},
        {'event_type': 'on_arrived', 'request_id': 'elsewhere', 'sub_id': 2},
        {'event_type': 'on_arrived', 'request_id': 'a', 'sub_id': 3},
        {'event_type': 'on_progress', 'request_id': 'a', 'sub_id': 2},
        {'event_type': 'on_arrived', 'request_id': 'a', 'sub_id': 2},
        {'type': 'state_report', 'state': 'IDLE'},
    ]
    socket = FakeSocket(frames)
    session = AgentSession([ScriptStep(navigate_to('a', 'poi-a', 2), 'on_arrived'),
                            ScriptStep(abort('a'))])
    asyncio.run(session.run(socket))
    assert session.completed
    assert session.events == frames
    assert [json.loads(frame)['action'] for frame in socket.sent] == (
        ['navigate_to', 'abort_navigation'])


def test_timeout_prevents_next_command():
    """A missing terminal event fails without sending the following command."""
    socket = FakeSocket()
    session = AgentSession([ScriptStep(navigate_to('a', 'poi-a'), 'on_arrived'),
                            ScriptStep(abort('a'))], timeout=0.01)
    with pytest.raises(asyncio.TimeoutError):
        asyncio.run(session.run(socket))
    assert len(socket.sent) == 1
    assert not session.completed


def test_passive_session_never_sends():
    """Default fake server records events without issuing commands."""
    socket = FakeSocket([{'type': 'heartbeat'}])
    session = AgentSession()
    asyncio.run(session.run(socket))
    assert not socket.sent
    assert session.events == [{'type': 'heartbeat'}]


def test_ping_pong_during_barrier_and_passive_collection():
    """The executor application keepalive works independently of its task events."""
    socket = FakeSocket([{'type': 'ping'},
                         {'event_type': 'on_arrived', 'request_id': 'a', 'sub_id': 0},
                         {'type': 'ping'}])
    session = AgentSession([ScriptStep(navigate_to('a', 'p'), 'on_arrived')])
    asyncio.run(session.run(socket))
    assert [json.loads(frame) for frame in socket.sent[1:]] == [{'type': 'pong'}] * 2
    assert len(session.events) == 3


def test_rejects_nonobject_frames():
    """Invalid peer JSON does not enter the recorded event stream."""
    session = AgentSession()
    with pytest.raises(ValueError, match='objects'):
        asyncio.run(session.run(FakeSocket([[]])))
    assert not session.events


@pytest.mark.parametrize('command', [
    {'action': 'abort'},
    {'action': 'navigate_to', 'request_id': 'a'},
    {'action': 'navigate_to', 'request_id': '', 'target_id': 'p'},
    {'action': 'navigate_to', 'request_id': 'a', 'target_id': 'p', 'sub_id': True},
    {'action': 'abort_navigation', 'request_id': None},
])
def test_invalid_script_commands(tmp_path, command):
    """Only well-formed v1 navigate and abort commands are permitted."""
    path = tmp_path / 'scenario.json'
    path.write_text(json.dumps([{'command': command}]))
    with pytest.raises(ValueError):
        load_script(path)


def test_load_valid_script(tmp_path):
    """The on-disk scenario is a finite list with optional event barriers."""
    path = tmp_path / 'scenario.json'
    path.write_text(json.dumps([{'command': navigate_to('a', 'p'), 'wait_event': 'on_arrived'},
                                {'command': abort('a')}]))
    steps = load_script(path)
    assert len(steps) == 2
    assert steps[0].wait_event == 'on_arrived'
    assert steps[1].command == abort('a')


def test_cli_requires_explicit_script_permission_before_starting(monkeypatch):
    """Even in domain 99 a supplied script does not run by default."""
    monkeypatch.setenv('ROS_DOMAIN_ID', '99')
    with pytest.raises(SystemExit) as error:
        main(['--script', '/not/read/or/executed.json'])
    assert error.value.code == 2


def test_installed_websocket_api_with_passive_loopback_peer():
    """A local real socket checks library compatibility using only ping/pong."""
    from websockets.legacy.client import connect
    from websockets.legacy.server import serve

    async def check():
        session = AgentSession()
        async with serve(session.run, '127.0.0.1', 0) as server:
            port = server.sockets[0].getsockname()[1]
            async with connect(f'ws://127.0.0.1:{port}', ping_interval=None) as websocket:
                await websocket.send(json.dumps({'type': 'ping'}))
                assert json.loads(await asyncio.wait_for(websocket.recv(), 2.0)) == {
                    'type': 'pong'}
        assert session.completed
        assert session.events == [{'type': 'ping'}]

    asyncio.run(check())
