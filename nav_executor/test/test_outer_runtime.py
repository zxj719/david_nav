"""Outer process lifetime with real task composition and fake ROS/action I/O."""

import asyncio
from contextlib import asynccontextmanager
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from nav_tasks.api import ConfigError, create_runtime, RuntimeAssemblyError

from nav_executor.app.config import load_configuration
from nav_executor.app.runtime import run

from test_runtime_assembly import _Buffer, _Node, ros_io  # noqa: F401
from test_ws_client import _Socket

PROFILE = Path(__file__).resolve().parents[2] / 'david_config' / 'robot' / 'david.yaml'


class _Host:
    def __init__(self, node=None):
        self.node = node or _Node()
        self.tf_buffer = _Buffer()
        self.ticks = 0
        self.opened = []
        self.closed = 0

    def open(self, args):
        self.opened.append(tuple(args))

    def spin_once(self):
        self.ticks += 1

    def close(self):
        assert all(client.destroy_count == 1 for client in self.node.clients)
        assert len(self.node.destroyed) == len(self.node.subscriptions)
        self.closed += 1


def _config(tmp_path):
    resolved = load_configuration([], profile_path=PROFILE, environ={})
    return replace(resolved, config=replace(
        resolved.config, poi_store_file=str(tmp_path / 'pois.yaml'), heartbeat_interval=100))


@pytest.mark.usefixtures('ros_io')
def test_stop_joins_connections_before_task_and_host_teardown(tmp_path):
    """Normal stop drains owned connection work and then releases all task/ROS resources."""
    async def scenario():
        host, stop, socket = _Host(), asyncio.Event(), _Socket()
        before = set(asyncio.all_tasks())

        @asynccontextmanager
        async def connect(*args, **kwargs):
            yield socket

        task = asyncio.create_task(run(
            _config(tmp_path), stop, host_factory=lambda: host,
            connect=connect, emit=lambda s: None))
        await asyncio.wait_for(socket.reading.wait(), 1)
        stop.set()
        await asyncio.wait_for(task, 1)
        assert host.closed == 1 and host.ticks > 0
        assert len(host.node.destroyed) == 3
        assert set(asyncio.all_tasks()) == before

    asyncio.run(scenario())


@pytest.mark.usefixtures('ros_io')
@pytest.mark.parametrize('cancel_waiter', [False, True])
def test_pending_motion_keeps_spin_alive_until_terminal_result(tmp_path, cancel_waiter):
    """Shutdown deadline or repeated caller cancellation cannot destroy active ROS resources."""
    source = tmp_path / 'named.json'
    source.write_text('[{"id":"p","position":{"x":1}}]')

    async def scenario():
        node = _Node([None])
        node.complete_on_cancel = False
        host, stop, logs, warned = _Host(node), asyncio.Event(), [], asyncio.Event()
        socket = _Socket(['{"action":"navigate_to","request_id":"r",'
                          '"sub_id":1,"target_id":"p"}'])

        @asynccontextmanager
        async def connect(*args, **kwargs):
            yield socket

        def emit(line):
            logs.append(line)
            if 'cleanup pending (task runtime)' in line:
                warned.set()

        resolved = _config(tmp_path)
        resolved = replace(resolved, config=replace(
            resolved.config, pose_file=str(source), cancel_timeout_sec=0.001))
        runner = asyncio.create_task(run(
            resolved, stop, host_factory=lambda: host, connect=connect,
            emit=emit, cleanup_retry_sec=0.001))
        handle = await asyncio.wait_for(node.started.get(), 1)
        try:
            if cancel_waiter:
                runner.cancel()
            else:
                stop.set()
            await asyncio.wait_for(node.canceled.wait(), 1)
            await asyncio.wait_for(warned.wait(), 1)
            if cancel_waiter:
                runner.cancel()
            ticks = host.ticks
            await asyncio.sleep(0.03)
            assert not runner.done()
            assert host.closed == 0 and host.ticks > ticks
            assert node.clients[0].destroy_count == 0 and not node.destroyed
        finally:
            if not handle.result.done():
                handle.result.set_result(NS(status=5))
            stop.set()
            if cancel_waiter:
                with pytest.raises(asyncio.CancelledError):
                    await runner
            else:
                await runner
        assert host.closed == 1
        assert len(node.destroyed) == 3 and node.clients[0].destroy_count == 1

    asyncio.run(scenario())


@pytest.mark.usefixtures('ros_io')
def test_failed_task_construction_cleanup_retries_before_host_close(tmp_path):
    """An assembly failure's retained cleanup runs with ROS available before error propagation."""
    async def scenario():
        node = _Node()
        node.fail_client_destroy = True
        host, warned = _Host(node), asyncio.Event()

        def emit(line):
            if 'cleanup pending (task construction)' in line:
                warned.set()

        resolved = _config(tmp_path)
        resolved = replace(resolved, config=replace(
            resolved.config, progress_min_interval_sec=float('nan')))
        runner = asyncio.create_task(run(
            resolved, asyncio.Event(), host_factory=lambda: host, connect=lambda: None,
            emit=emit, cleanup_retry_sec=0.001))
        try:
            await asyncio.wait_for(warned.wait(), 1)
            assert host.closed == 0 and len(node.destroyed) == 3
        finally:
            node.fail_client_destroy = False
        with pytest.raises(RuntimeAssemblyError):
            await runner
        assert host.closed == 1 and node.clients[0].destroy_count == 1

    asyncio.run(scenario())


@pytest.mark.usefixtures('ros_io')
def test_stop_during_construction_never_starts_ingress(tmp_path):
    """A stop request arriving during assembly cleans resources without accepting WS work."""
    async def scenario():
        host, stop = _Host(), asyncio.Event()

        async def construct(*args, **kwargs):
            result = await create_runtime(*args, **kwargs)
            stop.set()
            return result

        def no_connect(*args, **kwargs):
            pytest.fail('ingress started after stop')

        await run(_config(tmp_path), stop, host_factory=lambda: host,
                  runtime_factory=construct, connect=no_connect, emit=lambda s: None)
        assert host.closed == 1 and not host.node.goals

    asyncio.run(scenario())


@pytest.mark.usefixtures('ros_io')
def test_spin_failure_stops_ingress_and_cleans_idle_runtime(tmp_path):
    """Callback errors trigger orderly shutdown instead of silently losing ROS processing."""
    async def scenario():
        host = _Host()

        def broken():
            raise OSError('spin failed')

        host.spin_once = broken

        @asynccontextmanager
        async def connect(*args, **kwargs):
            yield _Socket()

        with pytest.raises(OSError, match='spin failed'):
            await run(_config(tmp_path), asyncio.Event(), host_factory=lambda: host,
                      connect=connect, emit=lambda s: None)
        assert host.closed == 1

    asyncio.run(scenario())


def test_dock_refusal_and_preexisting_stop_happen_before_host_creation(tmp_path):
    """The incomplete Dock mode cannot accidentally initialize an ordinary-navigation host."""
    async def scenario():
        def no_host():
            pytest.fail('host created')

        resolved = _config(tmp_path)
        dock = replace(resolved, config=replace(resolved.config, dock_poi_ids=('p',)))
        with pytest.raises(ConfigError, match='Dock'):
            await run(dock, asyncio.Event(), host_factory=no_host)
        stop = asyncio.Event()
        stop.set()
        await run(resolved, stop, host_factory=no_host)

    asyncio.run(scenario())
