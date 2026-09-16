"""WS lifecycle and legacy frames using in-memory sockets, never ROS or motion I/O."""

import asyncio
from contextlib import asynccontextmanager
import json
import os

import pytest

from nav_executor.adapters.transport.outbox import ReliableOutbox
from nav_executor.adapters.transport.proxy import bypass_proxy_for_server
from nav_executor.adapters.transport.ws_client import (
    receive_messages, reconnect, send_heartbeat, WsOptions,
)
from nav_executor.protocol.task_adapter import TaskCommandAdapter

from test_runtime_assembly import _Node, _runtime, ros_io  # noqa: F401


class _Socket:
    def __init__(self, messages=(), fail_send=False):
        self.inbound = asyncio.Queue()
        for message in messages:
            self.inbound.put_nowait(message)
        self.sent = []
        self.sent_queue = asyncio.Queue()
        self.fail_send = fail_send
        self.reading = asyncio.Event()

    def __aiter__(self):
        return self

    async def __anext__(self):
        self.reading.set()
        value = await self.inbound.get()
        if isinstance(value, BaseException):
            raise value
        if value is None:
            raise StopAsyncIteration
        return value

    async def send(self, encoded):
        if self.fail_send:
            raise OSError('send failed')
        self.sent.append(encoded)
        self.sent_queue.put_nowait(encoded)


class _Adapter:
    def __init__(self, error=None):
        self.handled = []
        self.error = error

    async def handle_message(self, payload):
        self.handled.append(payload)
        if self.error is not None:
            raise self.error
        return [{'event_type': 'on_arrived', 'request_id': '汉字', 'sub_id': 1}]


async def _cancel(task):
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task


def test_receive_keeps_wire_frames_and_legacy_ignore_logs():
    """Invalid JSON, nonobjects and protocol pings do not reach the command adapter."""
    async def scenario():
        socket = _Socket(['bad', '[1]', '{"type":"pong"}', '{"type":"ping"}',
                          '{"action":"navigate_to"}', None])
        adapter, outbox, lines, last = _Adapter(), ReliableOutbox(), [], [0]
        await receive_messages(socket, adapter, outbox, last, emit=lines.append, now=lambda: 7)
        assert last == [7]
        assert adapter.handled == [{'action': 'navigate_to'}]
        assert socket.sent == ['{"type": "pong"}']
        assert lines == ["ignoring non-JSON message: 'bad'", "ignoring non-object message: '[1]'"]
        drain = asyncio.create_task(outbox.drain(socket.send))
        await socket.sent_queue.get()  # pong
        assert await socket.sent_queue.get() == (
            '{"event_type": "on_arrived", "request_id": "汉字", "sub_id": 1}')
        await _cancel(drain)

    asyncio.run(scenario())


@pytest.mark.parametrize(
    'error', [ValueError('unsupported'), KeyError('missing'), TypeError('bad')])
def test_receive_isolates_application_failures_after_r1(error):
    """Bad application messages are contained while the old unsupported log stays stable."""
    async def scenario():
        socket, lines = _Socket(['{}', None]), []
        operation = receive_messages(
            socket, _Adapter(error), ReliableOutbox(), [0], emit=lines.append)
        if isinstance(error, ValueError):
            await operation
            assert lines == ['ignoring unsupported message: unsupported']
        else:
            await operation
            assert lines == [f'ignoring failed message: {type(error).__name__}: {error}']

    asyncio.run(scenario())


def test_heartbeat_warns_once_and_rearms_after_recent_inbound():
    """Frozen ping and silence logs retain old thresholds and reset behavior."""
    async def scenario():
        socket, last, lines = _Socket(), [0], []
        ages = iter([31, 40, 1, 32])
        current = [0]

        async def sleep(delay):
            assert delay == 10
            try:
                current[0] = next(ages)
            except StopIteration:
                raise asyncio.CancelledError

        with pytest.raises(asyncio.CancelledError):
            await send_heartbeat(socket, 10, last, emit=lines.append,
                                 now=lambda: current[0], sleep=sleep)
        assert socket.sent == ['{"type": "ping"}'] * 4
        assert lines == [
            '⚠ 已 31s 未收到服务端任何帧（连接可能半死，等待超时断开）',
            '⚠ 已 32s 未收到服务端任何帧（连接可能半死，等待超时断开）',
        ]

    asyncio.run(scenario())


@pytest.mark.parametrize('failure', ['send', 'receive', 'normal'])
def test_reconnect_retains_outbox_and_joins_connection_children(failure):
    """Reconnect keeps pending frames and observers, cancellation leaves no child tasks."""
    async def scenario():
        before = set(asyncio.all_tasks())
        first = _Socket(fail_send=failure == 'send')
        if failure == 'normal':
            first.inbound.put_nowait(None)
        elif failure == 'receive':
            first.inbound.put_nowait(OSError('receive transport failed'))
        second = _Socket()
        sockets = iter([first, second])
        events, lines = [], []
        adapter = _Adapter()
        outbox = ReliableOutbox()
        outbox.put({'event_type': 'on_arrived', 'request_id': 'r', 'sub_id': 1})

        @asynccontextmanager
        async def connect(uri, **kwargs):
            assert uri == 'ws://agent' and kwargs == {'ping_interval': None}
            socket = next(sockets)
            events.append('open')
            try:
                yield socket
            finally:
                events.append('closed')

        async def stop():
            events.append('stop')

        runner = asyncio.create_task(reconnect(
            WsOptions('ws://agent', 100, 0), adapter, outbox, connect=connect,
            on_connected=lambda: events.append('start'), on_disconnected=stop, emit=lines.append))
        await asyncio.wait_for(second.reading.wait(), 1)
        if failure == 'send':
            # A send failure wakes a blocked receiver, then the second socket gets
            # the exact retained frame rather than waiting for inbound traffic.
            encoded = await asyncio.wait_for(second.sent_queue.get(), 1)
            assert json.loads(encoded)['request_id'] == 'r'
            assert any('待重发事件 1 条' in line for line in lines)
        await _cancel(runner)
        assert events == ['open', 'start', 'stop', 'closed'] * 2
        assert set(asyncio.all_tasks()) == before
        assert len(first.sent) + len(second.sent) == 1
        assert outbox.pending_count == outbox.in_flight_count == 0

    asyncio.run(scenario())


def test_failed_connect_throttles_log_and_has_no_observer_cleanup():
    """Attempts one and ten log while refusal never starts connection observers."""
    async def scenario():
        lines, attempts = [], [0]

        @asynccontextmanager
        async def connect(*args, **kwargs):
            attempts[0] += 1
            raise OSError('offline')
            yield  # pragma: no cover

        async def sleep(delay):
            assert delay == 3
            if attempts[0] == 11:
                raise asyncio.CancelledError

        def unexpected():
            pytest.fail('observer must not start')

        with pytest.raises(asyncio.CancelledError):
            await reconnect(WsOptions('ws://agent'), _Adapter(), ReliableOutbox(), connect=connect,
                            on_connected=unexpected, on_disconnected=unexpected,
                            emit=lines.append, sleep=sleep)
        assert lines == [
            'connecting to ws://agent',
            'connection failed: offline(连不上时每10次提示一行)',
            'reconnecting in 3.0s',
            'connecting to ws://agent (第10次尝试)',
            'connection failed: offline(连不上时每10次提示一行)',
            'reconnecting in 3.0s',
        ]

    asyncio.run(scenario())


@pytest.mark.usefixtures('ros_io')
def test_socket_command_uses_actual_public_runtime_and_persistent_event_pump(tmp_path):
    """Actual runtime, storage, skill and backend operate behind the socket adapter."""
    async def scenario():
        runtime, node, _ = await _runtime(tmp_path)
        adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
        outbox = ReliableOutbox()
        socket = _Socket([
            json.dumps({'action': 'mark_current_poi', 'request_id': 'm', 'sub_id': 0,
                        'poi': {'id': 'p', 'name': 'test'}}),
            json.dumps({'action': 'navigate_to', 'request_id': 'r',
                        'sub_id': 1, 'target_id': 'p'}),
        ])

        @asynccontextmanager
        async def connect(*args, **kwargs):
            yield socket

        pump = asyncio.create_task(adapter.pump_events(outbox.put))
        runner = asyncio.create_task(reconnect(
            WsOptions('ws://agent', 100, 0), adapter, outbox, connect=connect,
            on_connected=runtime.start_loop_closure, on_disconnected=runtime.stop_loop_closure,
            emit=lambda line: None))
        try:
            frames = [json.loads(await asyncio.wait_for(socket.sent_queue.get(), 1))
                      for _ in range(3)]
            assert frames[-1]['event_type'] == 'on_arrived'
            assert frames[-1]['request_id'] == 'r'
            assert len(node.goals) == 1
        finally:
            await _cancel(runner)
            await _cancel(pump)
            adapter.close()
            await runtime.close()
        assert len(node.destroyed) == 3

    asyncio.run(scenario())


@pytest.mark.parametrize('uri,host', [
    ('ws://agent:8101', 'agent'), ('ws://[::1]:8', '::1'), ('relative', None)])
def test_proxy_bypass_preserves_other_hosts_and_is_idempotent(monkeypatch, uri, host):
    """Both proxy variables keep their previous entries and append the explicit host once."""
    monkeypatch.setenv('no_proxy', ' localhost, example ')
    monkeypatch.setenv('NO_PROXY', 'existing')
    bypass_proxy_for_server(uri)
    first = os.environ['no_proxy'], os.environ['NO_PROXY']
    bypass_proxy_for_server(uri)
    assert (os.environ['no_proxy'], os.environ['NO_PROXY']) == first
    if host:
        assert first == (f'localhost,example,{host}', f'existing,{host}')
    else:
        assert first == (' localhost, example ', 'existing')


@pytest.mark.usefixtures('ros_io')
def test_real_loopback_ws_abort_and_reconnect(tmp_path):
    """Real localhost sockets drive the actual task runtime; ROS/action I/O stays fake."""
    import websockets

    async def scenario():
        runtime, node, _ = await _runtime(tmp_path, _Node([None, 4]))
        adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
        outbox = ReliableOutbox()
        observations, errors, connections = [], [], [0]
        finished = asyncio.Event()

        async def server(socket):
            connections[0] += 1
            try:
                if connections[0] == 1:
                    await socket.send(json.dumps({
                        'action': 'mark_current_poi', 'request_id': 'm', 'sub_id': 0,
                        'poi': {'id': 'p', 'name': 'test'}}))
                    observations.extend([json.loads(await socket.recv()) for _ in range(2)])
                    await socket.send(json.dumps({
                        'action': 'navigate_to', 'request_id': 'r',
                        'sub_id': 1, 'target_id': 'p'}))
                    await node.started.get()
                    await socket.send(json.dumps({
                        'action': 'abort_navigation', 'request_id': 'r'}))
                    observations.append(json.loads(await socket.recv()))
                    await socket.close()
                else:
                    await socket.send(json.dumps({
                        'action': 'navigate_to', 'request_id': 'next',
                        'sub_id': 2, 'target_id': 'p'}))
                    observations.append(json.loads(await socket.recv()))
                    finished.set()
                    await socket.wait_closed()
            except Exception as error:
                errors.append(error)
                finished.set()

        pump = asyncio.create_task(adapter.pump_events(outbox.put))
        try:
            async with websockets.serve(server, '127.0.0.1', 0) as listener:
                port = listener.sockets[0].getsockname()[1]
                runner = asyncio.create_task(reconnect(
                    WsOptions(f'ws://127.0.0.1:{port}', 100, 0), adapter, outbox,
                    connect=websockets.connect, on_connected=runtime.start_loop_closure,
                    on_disconnected=runtime.stop_loop_closure, emit=lambda line: None))
                try:
                    await asyncio.wait_for(finished.wait(), 5)
                finally:
                    await _cancel(runner)
            assert not errors
            assert connections == [2]
            assert observations[2]['status'] == 'paused'
            assert observations[3] == {
                'event_type': 'on_arrived', 'request_id': 'next', 'sub_id': 2}
            assert len(node.goals) == 2
        finally:
            await _cancel(pump)
            adapter.close()
            await runtime.close()

    asyncio.run(scenario())
