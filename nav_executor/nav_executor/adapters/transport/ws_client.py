"""Reconnect and connection-scoped WS work, independent of ROS and motion policy."""

import asyncio
from dataclasses import dataclass
import json
import time
from typing import AsyncContextManager, AsyncIterator, Awaitable, Callable, Protocol

from nav_executor.adapters.transport.outbox import ReliableOutbox
from nav_executor.adapters.transport.logging import log_line
from nav_executor.protocol.task_adapter import TaskCommandAdapter
from nav_executor.protocol.messages import MalformedCommandError


class WebSocketLike(Protocol):
    """Minimal connected transport contract for real sockets or in-memory tests."""

    def __aiter__(self) -> AsyncIterator[str | bytes]:
        """Iterate inbound text/binary frames until the connection closes.

        Returns:
            Async frame iterator.
        """
        ...

    async def send(self, message: str) -> None:
        """Send one serialized text frame.

        Args:
            message: JSON text, preserving the existing wire format.
        """
        ...


@dataclass(frozen=True)
class WsOptions:
    """Transport configuration; the robot-aware outer root supplies the server URI.

    Attributes:
        heartbeat_interval: 应用层 JSON 心跳周期（秒）。
        reconnect_delay: 连接失败后的重连间隔（秒）。
        server_uri: 连接 Agent 的 WebSocket URI。
    """

    server_uri: str
    heartbeat_interval: float = 30.0
    reconnect_delay: float = 3.0


async def receive_messages(
    socket: WebSocketLike, adapter: TaskCommandAdapter, outbox: ReliableOutbox,
    last_inbound: list[float], *, emit: Callable[[str], None] = log_line,
    now: Callable[[], float] = time.monotonic,
) -> None:
    """Decode frames and isolate a bad application message without dropping its socket.

    JSON/non-object/unsupported frames retain their ignore policy. Malformed
    commands with request context are answered by the adapter; uncorrelated
    errors and unexpected per-message failures are logged and skipped. Socket
    iteration/send errors and BaseException cancellation still reach the owner.

    Args:
        socket: Connected frame transport.
        adapter: Process-lifetime public task command adapter.
        outbox: Same reliable outbox across reconnects.
        last_inbound: One-element monotonic timestamp shared with heartbeat.
        emit: Unprefixed legacy log sink.
        now: Monotonic timestamp source.
    """
    async for raw in socket:
        last_inbound[0] = now()
        try:
            payload = json.loads(raw)
        except (json.JSONDecodeError, UnicodeError):
            emit(f'ignoring non-JSON message: {raw[:200]!r}')
            continue
        except Exception as error:
            emit(f'ignoring failed message: {type(error).__name__}: {error}')
            continue
        if not isinstance(payload, dict):
            emit(f'ignoring non-object message: {raw[:200]!r}')
            continue
        if payload.get('type') == 'pong':
            continue
        if payload.get('type') == 'ping':
            await socket.send(json.dumps({'type': 'pong'}, ensure_ascii=False))
            continue
        try:
            frames = await adapter.handle_message(payload)
            for frame in frames:
                outbox.put(frame)
        except MalformedCommandError as error:
            emit(f'ignoring malformed message: {error}')
        except ValueError as error:
            emit(f'ignoring unsupported message: {error}')
        except Exception as error:
            emit(f'ignoring failed message: {type(error).__name__}: {error}')


async def send_heartbeat(
    socket: WebSocketLike, interval_sec: float, last_inbound: list[float], *,
    emit: Callable[[str], None] = log_line, now: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], Awaitable[None]] = asyncio.sleep,
) -> None:
    """Send legacy JSON pings and warn once per prolonged inbound silence episode.

    Args:
        socket: Connected frame transport.
        interval_sec: Existing heartbeat interval.
        last_inbound: Shared one-element monotonic timestamp.
        emit: Unprefixed legacy log sink.
        now: Monotonic timestamp source.
        sleep: Injectable asynchronous delay.
    """
    warned = False
    while True:
        await sleep(interval_sec)
        await socket.send(json.dumps({'type': 'ping'}, ensure_ascii=False))
        age = now() - last_inbound[0]
        if age > 3 * interval_sec and not warned:
            warned = True
            emit(f'⚠ 已 {age:.0f}s 未收到服务端任何帧（连接可能半死，等待超时断开）')
        elif age <= interval_sec:
            warned = False


async def _connection(socket, adapter, outbox, options, last_inbound, emit, now):
    # Strong local references survive until all children have been joined. A
    # failed sender must wake the connection owner even if receive is blocked.
    children = (
        asyncio.create_task(receive_messages(
            socket, adapter, outbox, last_inbound, emit=emit, now=now), name='ws-receive'),
        asyncio.create_task(send_heartbeat(
            socket, options.heartbeat_interval, last_inbound, emit=emit, now=now),
            name='ws-heartbeat'),
        asyncio.create_task(outbox.drain(socket.send), name='ws-outbound'),
    )
    try:
        done, _ = await asyncio.wait(children, return_when=asyncio.FIRST_COMPLETED)
        # Inspect every completed child, including simultaneous receive/send ends.
        for child in children:
            if child in done:
                child.result()
    finally:
        for child in children:
            child.cancel()
        await asyncio.gather(*children, return_exceptions=True)


async def reconnect(
    options: WsOptions, adapter: TaskCommandAdapter, outbox: ReliableOutbox, *,
    connect: Callable[..., AsyncContextManager[WebSocketLike]],
    on_connected: Callable[[], None], on_disconnected: Callable[[], Awaitable[None]],
    emit: Callable[[str], None] = log_line, now: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], Awaitable[None]] = asyncio.sleep,
) -> None:
    """Reconnect without replacing the adapter/outbox or canceling navigation.

    The outer root owns the persistent event pump, status loop and this coroutine.
    It passes its configured library's connect function; no monkeypatch or library
    version choice occurs here. Connection children are strongly owned and joined.
    Sender failure triggers reconnect even when no inbound frame arrives, so a
    failed outbox cannot silently strand terminal events on an apparently live link.

    Args:
        options: Explicit server and timing configuration.
        adapter: Same process-lifetime adapter on every attempt.
        outbox: Reliable queued/in-flight frames retained between attempts.
        connect: Async context-manager factory, called with ping_interval=None.
        on_connected: Start the runtime's connection-scoped observer.
        on_disconnected: Stop/join that observer; must not cancel motion.
        emit: Unprefixed log sink.
        now: Monotonic timestamp source.
        sleep: Reconnection delay implementation.

    Raises:
        asyncio.CancelledError: Outer shutdown; connection children are joined first.
    """
    attempt = 0
    disconnected_since = None
    last_inbound = [now()]
    while True:
        established = False
        connected_at = None
        attempt += 1
        verbose = attempt == 1 or attempt % 10 == 0
        try:
            if verbose:
                suffix = f'(第{attempt}次尝试)' if attempt > 1 else ''
                emit(f'connecting to {options.server_uri} {suffix}'.rstrip())
            async with connect(options.server_uri, ping_interval=None) as socket:
                established = True
                connected_at = now()
                gap = (f', 断联 {connected_at - disconnected_since:.1f}s'
                       if disconnected_since is not None else '')
                pending = (f', 待重发事件 {outbox.in_flight_count} 条'
                           if outbox.in_flight_count else '')
                emit(f'connected (第{attempt}次尝试{gap}{pending})')
                attempt = 0
                last_inbound[0] = now()
                try:
                    on_connected()
                    await _connection(socket, adapter, outbox, options, last_inbound, emit, now)
                finally:
                    await on_disconnected()
        except asyncio.CancelledError:
            raise
        except Exception as error:
            if established:
                disconnected_since = now()
                unsent = outbox.pending_count
                emit(
                    f'connection closed: {error} '
                    f'(本次连接 {disconnected_since - connected_at:.1f}s, '
                    f'距最后收到服务端帧 {disconnected_since - last_inbound[0]:.1f}s, '
                    f'未送达事件 {unsent} 条'
                    f'{"，重连后自动重发" if unsent else ""})')
            elif verbose:
                emit(f'connection failed: {error}(连不上时每10次提示一行)')
        else:
            if established:
                disconnected_since = now()
                emit(f'connection closed (本次连接 {disconnected_since - connected_at:.1f}s, '
                     f'未送达事件 {outbox.pending_count} 条)')
        if verbose or established:
            emit(f'reconnecting in {options.reconnect_delay:.1f}s')
        await sleep(options.reconnect_delay)
