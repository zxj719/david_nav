"""R1 bad-frame isolation through the real parser, adapter and fake-motion runtime."""

import asyncio
import json

import pytest

from nav_executor.adapters.transport.outbox import ReliableOutbox
from nav_executor.adapters.transport.ws_client import receive_messages, reconnect, WsOptions
from nav_executor.protocol import codec, messages
from nav_executor.protocol.task_adapter import TaskCommandAdapter

from test_runtime_assembly import _runtime, ros_io  # noqa: F401
from test_ws_client import _cancel, _Socket


@pytest.mark.parametrize('payload', [
    None, [], 'text',
    {'action': 'mark_current_poi', 'request_id': 'r'},
    {'action': 'mark_current_poi', 'request_id': 'r', 'sub_id': 2, 'poi': None},
    {'action': 'mark_current_poi', 'request_id': 'r', 'sub_id': 2, 'poi': []},
    {'action': 'mark_current_poi', 'request_id': 'r', 'sub_id': 2, 'poi': {'id': 'p'}},
    {'action': 'mark_current_poi', 'request_id': 'r', 'sub_id': 'bad',
     'poi': {'id': 'p', 'name': 'name'}},
    {'action': 'mark_current_poi', 'request_id': 'r', 'sub_id': 2,
     'poi': {'id': [], 'name': 'name'}},
    {'action': 'abort_navigation'},
    {'action': 'abort_navigation', 'request_id': None},
    {'action': 'abort_navigation', 'request_id': {}},
    {'action': 'navigate_to', 'request_id': 'r'},
    {'action': 'update_poi_list', 'request_id': 'r', 'poi_list': None},
    {'action': 'update_poi_list', 'request_id': 'r', 'poi_list': 'text'},
    {'action': 'update_poi_list', 'request_id': 'r', 'poi_list': [None]},
    {'action': 'update_poi_list', 'request_id': 'r', 'poi_list': [{'id': 'unknown'}]},
    {'action': 'update_poi_list', 'request_id': 'r', 'version': float('inf')},
])
def test_decoder_normalizes_bad_fields_and_recovers_context(payload):
    """No missing field/type/conversion exception escapes as an untyped parser failure."""
    with pytest.raises(ValueError) as caught:
        codec.parse_command(payload)
    assert isinstance(caught.value, messages.MalformedCommandError)
    expected = 'r' if isinstance(payload, dict) and payload.get('request_id') == 'r' else None
    assert caught.value.request_id == expected
    assert isinstance(caught.value.sub_id, int)


def test_valid_scalar_coercions_and_directory_whitespace_remain_compatible():
    """R1 validates shapes without changing accepted legacy coercion or directory values."""
    command = codec.parse_command({'action': 'mark_current_poi', 'request_id': 7,
                                   'sub_id': '2', 'poi': {'id': 123, 'name': ' label '}})
    assert (command.request_id, command.sub_id, command.poi_id, command.poi_name) == (
        '7', 2, '123', 'label')
    items = [{'id': ' p ', 'name': ' name '}]
    assert codec.parse_command({'action': 'update_poi_list', 'poi_list': items}).poi_list == items
    navigation = codec.parse_command({'action': 'navigate_to', 'request_id': 'r',
                                      'sub_id': float('inf'), 'target_id': 'p'})
    assert navigation.sub_id == 0
    assert issubclass(messages.MalformedCommandError, messages.CommandParseError)
    assert issubclass(messages.ProtocolError, ValueError)


@pytest.mark.parametrize('error', [KeyError('missing'), TypeError('wrong'), RuntimeError('bug')])
def test_unexpected_single_message_failure_does_not_end_receive(error):
    """Application errors are isolated, not mistaken for broken transport."""
    async def scenario():
        calls, lines = [], []

        class Adapter:
            async def handle_message(self, payload):
                calls.append(payload)
                if len(calls) == 1:
                    raise error
                return []

        await receive_messages(_Socket(['{"n":1}', '{"n":2}', None]), Adapter(),
                               ReliableOutbox(), [0], emit=lines.append)
        assert calls == [{'n': 1}, {'n': 2}]
        assert len(lines) == 1 and type(error).__name__ in lines[0]

    asyncio.run(scenario())


def test_cancellation_and_ping_transport_failure_still_propagate():
    """R1 isolation must not suppress shutdown or leave a dead socket connected."""
    async def scenario():
        class Adapter:
            async def handle_message(self, payload):
                raise asyncio.CancelledError

        with pytest.raises(asyncio.CancelledError):
            await receive_messages(_Socket(['{}']), Adapter(), ReliableOutbox(), [0])
        with pytest.raises(OSError, match='send failed'):
            await receive_messages(_Socket(['{"type":"ping"}'], fail_send=True), Adapter(),
                                   ReliableOutbox(), [0])

    asyncio.run(scenario())


@pytest.mark.usefixtures('ros_io')
def test_bad_catalog_does_not_partially_mutate_persisted_or_in_memory_names(tmp_path):
    """Decode the complete directory before invoking administrative mutation."""
    async def scenario():
        runtime, _, _ = await _runtime(tmp_path)
        adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
        try:
            await adapter.handle_message({'action': 'mark_current_poi', 'request_id': 'm',
                                          'sub_id': 0, 'poi': {'id': 'p', 'name': 'original'}})
            before = (tmp_path / 'store.yaml').read_bytes()
            frames = await adapter.handle_message({
                'action': 'update_poi_list', 'request_id': 'catalog',
                'poi_list': [{'id': 'p', 'name': 'partial'}, {'id': 'unknown'}]})
            assert len(frames) == 1 and frames[0]['event_type'] == 'on_error'
            assert frames[0]['request_id'] == 'catalog'
            assert (tmp_path / 'store.yaml').read_bytes() == before
            # Force a later valid persistence operation to expose any hidden memory mutation.
            await adapter.handle_message({'action': 'update_poi_list', 'poi_list': []})
            assert (tmp_path / 'store.yaml').read_bytes() == before
        finally:
            adapter.close()
            await runtime.close()

    asyncio.run(scenario())


@pytest.mark.usefixtures('ros_io')
def test_real_socket_bad_frames_then_valid_navigation_stay_on_one_connection(tmp_path):
    """Real loopback WS plus actual runtime; all ROS/action operations are in-memory fakes."""
    import websockets

    async def scenario():
        runtime, node, _ = await _runtime(tmp_path)
        adapter, outbox = TaskCommandAdapter(runtime.tasks, runtime.pois), ReliableOutbox()
        observed, errors, lines, connections = [], [], [], []
        finished = asyncio.Event()

        async def server(socket):
            connections.append(socket)
            try:
                await socket.send(json.dumps({'action': 'mark_current_poi', 'request_id': 'm',
                                              'sub_id': 0, 'poi': {'id': 'p', 'name': 'name'}}))
                observed.extend([json.loads(await socket.recv()) for _ in range(2)])
                for bad in ('{"action":"mark_current_poi","request_id":"bad"}',
                            '{"action":"abort_navigation"}', 'not json', '[1,2]', b'\xff'):
                    await socket.send(bad)
                await socket.send(json.dumps({'action': 'navigate_to', 'request_id': 'ok',
                                              'sub_id': 1, 'target_id': 'p'}))
                observed.extend([json.loads(await socket.recv()) for _ in range(2)])
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
                    on_disconnected=runtime.stop_loop_closure, emit=lines.append))
                try:
                    await asyncio.wait_for(finished.wait(), 5)
                finally:
                    await _cancel(runner)
            assert not errors
            assert len(connections) == 1 and len(node.goals) == 1
            assert observed[2]['event_type'] == 'on_error'
            assert observed[2]['request_id'] == 'bad'
            assert observed[3] == {'event_type': 'on_arrived', 'request_id': 'ok', 'sub_id': 1}
            assert len([line for line in lines if line.startswith('ignoring')]) == 4
        finally:
            await _cancel(pump)
            adapter.close()
            await runtime.close()

    asyncio.run(scenario())
