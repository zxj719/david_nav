"""Outer ROS/WS lifetime: stop ingress, settle tasks, then release host resources."""

import asyncio
from contextlib import suppress
import time
from typing import Callable

from nav_tasks.api import ConfigError, create_runtime, RuntimeAssemblyError

from nav_executor.adapters.ros.host import RosHost
from nav_executor.adapters.transport.outbox import ReliableOutbox
from nav_executor.adapters.transport.proxy import bypass_proxy_for_server
from nav_executor.adapters.transport.ws_client import reconnect
from nav_executor.app.config import ResolvedConfig
from nav_executor.app.logging import log_line, OutboundAudit
from nav_executor.app.status import print_status_loop
from nav_executor.protocol.task_adapter import TaskCommandAdapter


async def _join(tasks):
    for task in tasks:
        task.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)


async def _retry(operation, label, emit, delay):
    while True:
        try:
            await operation()
            return
        except (Exception, asyncio.CancelledError) as error:
            # Diagnostic failure must never drop resource ownership during cleanup.
            with suppress(Exception):
                emit(f'cleanup pending ({label}): {type(error).__name__}: {error}; retrying')
            await asyncio.sleep(delay)


async def _spin(host, fault, emit):
    while True:
        try:
            host.spin_once()
        except Exception as error:
            if not fault.done():
                fault.set_result(error)
                with suppress(Exception):
                    emit(f'ROS callback failed; stopping ingress and retaining cleanup: {error}')
            # Continue attempting callbacks while shutdown settles any owned goal.
            await asyncio.sleep(0.1)
        else:
            # Preserve the old 100 Hz polling model; R6 measurement is separate.
            await asyncio.sleep(0.01)


async def _finish(host, runtime, rollback, adapter, workers, spin, emit, delay):
    await _join(workers)
    if adapter is not None:
        adapter.close()
    if runtime is not None:
        await _retry(runtime.close, 'task runtime', emit, delay)
    if rollback is not None:
        await _retry(rollback, 'task construction', emit, delay)
    if spin is not None:
        await _join([spin])

    async def close_host():
        host.close()

    await _retry(close_host, 'ROS host', emit, delay)


async def run(
    resolved: ResolvedConfig, stop: asyncio.Event, *, host_factory: Callable = RosHost,
    runtime_factory: Callable = create_runtime, connect: Callable | None = None,
    emit: Callable[[str], None] = log_line, now: Callable[[], float] = time.time,
    cleanup_retry_sec: float = 1.0,
) -> None:
    """Own one ordinary-POI runtime and retain shutdown even if its caller is canceled.

    The CLI owns signal-to-stop translation; embedding callers supply their own
    stop event. Repeated cancellation cannot interrupt owned cleanup. Failed
    cleanup retries without a deadline that would abandon motion or ROS resources.
    A lost ROS context or unprovable executor shutdown may require human recovery;
    this coroutine deliberately does not claim success or release the host then.

    Args:
        resolved: Validated immutable task/transport settings and ROS arguments.
        stop: Graceful process stop request, never direct context shutdown.
        host_factory: Empty host owner, injectable for pure tests.
        runtime_factory: Public task factory, injectable for lifecycle fault tests.
        connect: Socket context factory, or None for the installed WebSocket library.
        emit: Unprefixed startup/status/audit log sink.
        now: Existing wall-clock pose freshness reader, injectable for fake timestamps.
        cleanup_retry_sec: Delay between failed cleanup attempts; production default one second.

    Raises:
        ConfigError: Dock is not integrated; refused before opening ROS or storage.
        asyncio.CancelledError: Caller canceled; raised only after cleanup succeeds.
        Exception: Construction or an essential background worker failed, after cleanup.
    """
    config = resolved.config
    if config.dock_poi_ids or resolved.dock is not None:
        raise ConfigError('Dock runtime assembly is not integrated yet; refusing ordinary routing')
    if stop.is_set():
        return
    if connect is None:
        import websockets
        emit(f'websockets version={websockets.__version__} file={websockets.__file__}')
        connect = websockets.connect
    emit(resolved.log_line())
    bypass_proxy_for_server(config.server_uri)
    host = host_factory()
    runtime = adapter = rollback = spin = None
    workers = []
    fault = asyncio.get_running_loop().create_future()
    try:
        host.open(resolved.ros_args)
        spin = asyncio.create_task(_spin(host, fault, emit), name='executor-ros-spin')
        try:
            runtime = await runtime_factory(resolved.task_config(), host.node, host.tf_buffer,
                                            log=emit, now=now)
        except RuntimeAssemblyError as error:
            rollback = error.cleanup
            raise
        if stop.is_set():
            return
        if fault.done():
            raise fault.result()
        adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
        audit = OutboundAudit(emit)
        outbox = ReliableOutbox(on_sending=audit.on_sending, on_sent=audit.on_sent)
        emit(f'nav action: {config.action_name} '
             '(nav WS v1.1: target_id + abort_navigation 随时中断→idle)')
        emit(f'nav failure debounce: retry_count={config.nav_retry_count} '
             f'retry_delay={config.nav_retry_delay_sec:.1f}s '
             '(单次失败静默重发,连败才上报 on_error)')
        workers.append(asyncio.create_task(
            adapter.pump_events(outbox.put), name='executor-events'))
        workers.append(asyncio.create_task(print_status_loop(
            runtime.tasks, runtime.poses, config.status_print_interval_sec, emit),
            name='executor-status'))
        workers.append(asyncio.create_task(reconnect(
            config.ws_options(), adapter, outbox, connect=connect,
            on_connected=runtime.start_loop_closure, on_disconnected=runtime.stop_loop_closure,
            emit=emit), name='executor-ws'))
        waiter = asyncio.create_task(stop.wait(), name='executor-stop')
        workers.append(waiter)
        done, _ = await asyncio.wait([fault, spin, *workers], return_when=asyncio.FIRST_COMPLETED)
        if fault in done:
            raise fault.result()
        for worker in [spin, *workers]:
            if worker in done and worker is not waiter:
                worker.result()
                raise RuntimeError(f'essential runtime worker ended: {worker.get_name()}')
    finally:
        cleanup = asyncio.create_task(_finish(
            host, runtime, rollback, adapter, workers, spin, emit, cleanup_retry_sec),
            name='executor-cleanup')
        canceled = False
        while not cleanup.done():
            try:
                await asyncio.shield(cleanup)
            except asyncio.CancelledError:
                canceled = True
        cleanup.result()
        if canceled:
            raise asyncio.CancelledError
