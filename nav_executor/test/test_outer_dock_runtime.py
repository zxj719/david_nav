"""One outer Dock/WS lifetime with real task assembly and only in-memory ROS I/O."""

import asyncio
from concurrent.futures import ThreadPoolExecutor
from contextlib import asynccontextmanager
import json
from types import SimpleNamespace as NS

import pytest

from nav_tasks.api import ClosedError, ConfigError, PoseRecord, TaskRequest
from nav_executor.app.config import load_configuration
from nav_executor.app.runtime import run

from test_dock_configuration import _file, PROFILE
from test_dock_ingress import _Handle, _Node, ingress_ros  # noqa: F401
from test_dock_runtime import (  # noqa: F401
    _create, _DockBuffer, _map, dock_ros, ros_io,
)
from test_ws_client import _Socket


class _ParametersNode(_Node):
    def __init__(self, outcomes=(4, 4), overrides=None):
        super().__init__(outcomes)
        self.parameters = {}
        self.overrides = overrides or {}
        self.remaps = {}

    def declare_parameter(self, name, value):
        self.parameters[name] = self.overrides.get(name, value)

    def get_parameter(self, name):
        return NS(value=self.parameters[name])

    def resolve_topic_name(self, name):
        return self.remaps.get(name, '/' + name.lstrip('/'))


class _Host:
    def __init__(self, node=None):
        self.node = node or _ParametersNode()
        self.tf_buffer = _DockBuffer()
        self.pool = ThreadPoolExecutor(max_workers=2)
        self.ticks = self.closed = 0
        self.quiesced = False
        self.quiescing = asyncio.Event()
        self.release_quiescence = asyncio.Event()
        self.release_quiescence.set()

    def open(self, args, *, dock=False):
        assert dock
        self.args = args

    def spin_once(self):
        assert not self.quiescing.is_set()
        self.ticks += 1

    async def quiesce(self):
        self.quiescing.set()
        await self.release_quiescence.wait()
        await asyncio.to_thread(self.pool.shutdown, wait=True)
        self.quiesced = True

    def close(self):
        assert self.quiesced
        assert all(client.destroy_count == 1 for client in self.node.clients)
        assert all(server.destroy_count == 1 for server in self.node.servers)
        assert len(self.node.destroyed) == len(self.node.subscriptions)
        self.closed += 1


def _config(tmp_path, *, websocket=False, extra=()):
    return load_configuration([
        '--dock-config-file', str(_file(tmp_path)), '--poi-store-file',
        str(tmp_path / 'pois.yaml'), '--websocket' if websocket else '--no-websocket',
        *extra], profile_path=PROFILE, environ={})


async def _ready(host, runner):
    async def wait():
        while not host.node.servers:
            if runner.done():
                await runner
            await asyncio.sleep(0.001)
    await asyncio.wait_for(wait(), 2)
    host.node.subscriptions[-2].callback(_map())
    host.node.subscriptions[0].callback(host.tf_buffer.odom())
    return host.node.servers[0]


@pytest.mark.usefixtures('dock_ros')
def test_settle_seals_motion_but_retains_ros_resources(tmp_path):
    """The host can join callback threads between motion settlement and resource release."""
    async def scenario():
        runtime, node, _ = await _create(tmp_path)
        await runtime.settle()
        assert not node.destroyed and all(client.destroy_count == 0 for client in node.clients)
        with pytest.raises(ClosedError):
            await runtime.tasks.submit(TaskRequest('r', 'dock', {}))
        with pytest.raises(ClosedError):
            runtime.start_loop_closure()
        await runtime.settle()
        await runtime.close()
        assert len(node.destroyed) == 4
        assert all(client.destroy_count == 1 for client in node.clients)

    asyncio.run(scenario())


@pytest.mark.usefixtures('dock_ros')
def test_lost_settle_waiter_does_not_abandon_motion_or_release_clients(tmp_path):
    """The new two-phase API retains its cleanup task independently of individual waiters."""
    async def scenario():
        node = _Node([None])
        node.complete_on_cancel = False
        runtime, _, _ = await _create(tmp_path, node)
        node.subscriptions[-1].callback(_map())
        await runtime.tasks.submit(TaskRequest(
            'r', 'dock', {'pose': PoseRecord('world', 1, 2, 0, 0, 0)}))
        handle = await asyncio.wait_for(node.started.get(), 2)
        waiter = asyncio.create_task(runtime.settle())
        await asyncio.wait_for(node.canceled.wait(), 1)
        waiter.cancel()
        with pytest.raises(asyncio.CancelledError):
            await waiter
        assert not node.destroyed and node.clients[0].destroy_count == 0
        handle.result.set_result(NS(status=5))
        await asyncio.wait_for(runtime.settle(), 2)
        assert not node.destroyed and node.clients[0].destroy_count == 0
        await runtime.close()

    asyncio.run(scenario())


@pytest.mark.usefixtures('ingress_ros')
def test_ros_only_mode_uses_override_and_retains_resources_until_quiescence(
    tmp_path, monkeypatch,
):
    """No WS connection/import is required; SDK result tail precedes every ROS destroy."""
    import builtins
    original = builtins.__import__

    def guarded(name, *args, **kwargs):
        assert name != 'websockets'
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', guarded)

    async def scenario():
        node = _ParametersNode(overrides={'action_name': '/overridden_dock', 'yaw_ok': 0.2})
        host, stop, logs = _Host(node), asyncio.Event(), []
        host.release_quiescence.clear()
        runner = asyncio.create_task(run(
            _config(tmp_path), stop, host_factory=lambda: host, emit=logs.append))
        try:
            server = await _ready(host, runner)
            assert server.name == '/overridden_dock'
            assert len(node.parameters) == 16
            handle = _Handle()

            def execute():
                server.execute(handle)
                assert not node.destroyed and server.destroy_count == 0
                node.events.append('SDK result stored')

            await asyncio.wait_for(asyncio.wrap_future(host.pool.submit(execute)), 3)
            assert handle.terminal == ['succeeded']
            assert len(node.goals) == 2 and len(node.clients) == 2
            stop.set()
            await asyncio.wait_for(host.quiescing.wait(), 1)
            assert not runner.done() and not node.destroyed
            assert all(client.destroy_count == 0 for client in node.clients)
            assert server.destroy_count == 0
            effective = next(line for line in logs if line.startswith('effective config:'))
            values = json.loads(effective.removeprefix('effective config: '))
            assert values['dock']['runtime']['yaw_ok'] == 0.2
            assert values['values']['dock_action_name'] == '/overridden_dock'
        finally:
            stop.set()
            host.release_quiescence.set()
            await asyncio.wait_for(runner, 3)
        assert host.closed == 1
        assert node.events.index('SDK result stored') < node.events.index('client destroyed')

    asyncio.run(scenario())


@pytest.mark.parametrize('cancel_waiter', [False, True])
@pytest.mark.usefixtures('ingress_ros')
def test_stop_waits_for_action_cleanup_while_ros_and_loop_remain_live(
    tmp_path, cancel_waiter,
):
    """A late result and repeated runner cancellation never release an active backend."""
    async def scenario():
        node = _ParametersNode([None])
        node.complete_on_cancel = False
        host, stop = _Host(node), asyncio.Event()
        warned = asyncio.Event()

        def emit(line):
            if 'cleanup pending' in line:
                warned.set()

        resolved = _config(tmp_path, extra=['--cancel-timeout-sec', '0.001'])
        runner = asyncio.create_task(run(resolved, stop, host_factory=lambda: host,
                                         emit=emit, cleanup_retry_sec=0.001))
        server = await _ready(host, runner)
        action = asyncio.wrap_future(host.pool.submit(server.execute, _Handle()))
        handle = await asyncio.wait_for(node.started.get(), 2)
        try:
            if cancel_waiter:
                runner.cancel()
            else:
                stop.set()
            await asyncio.wait_for(node.canceled.wait(), 1)
            await asyncio.sleep(0.03)
            ticks = host.ticks
            if cancel_waiter:
                runner.cancel()
            await asyncio.sleep(0.03)
            assert host.ticks > ticks and not host.quiescing.is_set()
            assert not action.done() and not runner.done()
            assert not node.destroyed and node.clients[0].destroy_count == 0
        finally:
            handle.result.set_result(NS(status=5))
            stop.set()
            await asyncio.wait_for(action, 2)
            if cancel_waiter:
                with pytest.raises(asyncio.CancelledError):
                    await asyncio.wait_for(runner, 2)
            else:
                await asyncio.wait_for(runner, 2)
        assert host.closed == 1

    asyncio.run(scenario())


@pytest.mark.parametrize('invalid', ['parameter', 'same_name', 'remapped_name', 'spin_name'])
@pytest.mark.usefixtures('ingress_ros')
def test_invalid_ros_values_and_recursive_endpoints_fail_before_storage_or_clients(
    tmp_path, invalid,
):
    """ROS overrides are validated before constructing the runtime or admitting tasks."""
    async def scenario():
        node = _ParametersNode(overrides={'yaw_ok': -1} if invalid == 'parameter' else {})
        if invalid == 'same_name':
            node.overrides['action_name'] = 'navigate_to_pose'
        if invalid == 'remapped_name':
            node.remaps['dock_to_pose'] = '/navigate_to_pose'
        if invalid == 'spin_name':
            node.overrides['action_name'] = 'spin'
        host = _Host(node)
        with pytest.raises(ConfigError):
            await run(_config(tmp_path), asyncio.Event(), host_factory=lambda: host,
                      emit=lambda s: None)
        assert not node.clients and not node.subscriptions
        assert not (tmp_path / 'pois.yaml').exists()
        assert host.closed == 1

    asyncio.run(scenario())


@pytest.mark.usefixtures('ingress_ros')
def test_partial_ingress_open_rolls_back_with_host_owned(tmp_path):
    """A failed ActionServer constructor does not leak the already-created topic input."""
    async def scenario():
        node = _ParametersNode()
        node.fail_server = True
        host = _Host(node)
        with pytest.raises(RuntimeError, match='server construction'):
            await run(_config(tmp_path), asyncio.Event(), host_factory=lambda: host,
                      emit=lambda s: None)
        assert host.closed == 1 and not node.goals
        assert len(node.destroyed) == 5

    asyncio.run(scenario())


@pytest.mark.usefixtures('ingress_ros')
def test_websocket_and_ros_use_one_manager_and_backend(tmp_path):
    """Optional WS maps Dock POIs directly to the shared worker, not a second action client."""
    source = tmp_path / 'named.json'
    source.write_text('[{"id":"p","position":{"x":1,"y":2}}]')

    async def scenario():
        host, stop, socket = _Host(), asyncio.Event(), _Socket()

        @asynccontextmanager
        async def connect(*args, **kwargs):
            yield socket

        resolved = _config(tmp_path, websocket=True, extra=[
            '--dock-poi-id', 'p', '--pose-file', str(source)])
        runner = asyncio.create_task(run(resolved, stop, host_factory=lambda: host,
                                         connect=connect, emit=lambda s: None))
        try:
            await _ready(host, runner)
            await asyncio.wait_for(socket.reading.wait(), 1)
            await socket.inbound.put(json.dumps(dict(
                action='navigate_to', target_id='p', request_id='ws-r', sub_id=1)))

            async def arrived():
                while not any('on_arrived' in value for value in socket.sent):
                    await asyncio.sleep(0.001)
            await asyncio.wait_for(arrived(), 3)
            assert len(host.node.goals) == 2 and len(host.node.clients) == 2
        finally:
            stop.set()
            await asyncio.wait_for(runner, 3)
        assert host.closed == 1

    asyncio.run(scenario())


def test_websocket_boolean_configuration_and_precedence(tmp_path):
    """Explicit booleans disable WS without changing frozen help or forwarding flags to ROS."""
    resolved = _config(tmp_path)
    assert resolved.config.websocket_enabled is False
    assert not resolved.ros_args
    assert _config(tmp_path, websocket=True).config.websocket_enabled is True
    invalid = tmp_path / 'bad.yaml'
    invalid.write_text('websocket_enabled: "false"')
    with pytest.raises(ConfigError):
        load_configuration(['--config', str(invalid)], profile_path=PROFILE, environ={})
