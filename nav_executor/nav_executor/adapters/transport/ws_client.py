"""管理 WS 重连和单连接任务，不依赖 ROS 或承担导航策略."""

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
    """定义已连接传输的最小接口，可由真实 WebSocket 或内存替身实现."""

    def __aiter__(self) -> AsyncIterator[str | bytes]:
        """异步迭代收到的文本或二进制帧，直到连接结束.

        Returns:
            入站帧的异步迭代器；传输错误可由具体实现向外传播。
        """
        ...

    async def send(self, message: str) -> None:
        """发送一条已序列化文本帧，返回不等于服务端业务确认.

        Args:
            message: 保持既有线上格式的 JSON 文本。
        """
        ...


@dataclass(frozen=True)
class WsOptions:
    """保存外层提供的连接与时间选项，构造时不自行验证数值.

    Attributes:
        heartbeat_interval: 应用层 JSON 心跳周期，单位秒。
        reconnect_delay: 一次连接尝试或已建立连接结束后的重试等待，单位秒。
        server_uri: 外层按机器人配置选定的 Agent WebSocket URI。
    """

    server_uri: str
    heartbeat_interval: float = 30.0
    reconnect_delay: float = 3.0


async def receive_messages(
    socket: WebSocketLike, adapter: TaskCommandAdapter, outbox: ReliableOutbox,
    last_inbound: list[float], *, emit: Callable[[str], None] = log_line,
    now: Callable[[], float] = time.monotonic,
) -> None:
    """逐帧解码，按既有策略记录并跳过应用层坏消息，不主动因此断开连接.

    任何入站帧先更新时间，包括坏 JSON 和非对象帧；ping 直接回复 pong，
    pong 忽略。带可恢复身份的坏命令由适配器生成错误应答，其它消息级错误记录后继续。
    标记 POI 的异步处理期间本收包循环会等待，导航请求则不等导航终态。

    Args:
        socket: 已建立的帧传输连接。
        adapter: 跨重连复用的进程生命周期命令适配器。
        outbox: 应答帧进入的同一个跨连接发送队列。
        last_inbound: 与心跳共享的单元素单调时间秒数列表。
        emit: 接收不含前缀的既有日志文本。
        now: 返回单调时钟秒数的函数。

    Raises:
        Exception: 迭代连接、发送 pong、计时或日志回调在消息隔离范围外失败。
        asyncio.CancelledError: 收包任务被取消，交给连接所有者处理。
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
    """周期发送既有 JSON ping，对持续未收任何帧的状态单次告警.

    先等待再发送；入站静默严格超过三个周期时告警，回到不超过一个周期时重置。
    本函数不主动断开连接，不核验 pong 内容，也不保证链路或导航健康。

    Args:
        socket: 已建立的传输连接。
        interval_sec: 秒制心跳间隔，由调用方提供；本函数不自行校验。
        last_inbound: 任意入站帧更新的单元素单调时间列表。
        emit: 接收不含前缀的既有日志文本。
        now: 返回单调时钟秒数的函数。
        sleep: 可注入的秒制异步等待操作。

    Raises:
        Exception: 等待、发送、计时或日志操作失败，向连接所有者传播。
        asyncio.CancelledError: 心跳任务被取消。
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
    """持有单连接的收包、心跳和发送任务，任一结束后取消并等待其它任务.

    不关闭跨连接适配器或 outbox，不取消导航。子任务创建在 try 前；
    finally 的 gather 未用 shield，不应承诺重复取消下仍保留全部清理责任。
    等待没有独立超时，不把进入 finally 当成子任务已经全部退出。

    Args:
        socket: 已建立的 WebSocket 连接。
        adapter: 进程生命周期的命令适配器。
        outbox: 跨连接保留待发帧的队列，不含服务端确认协议。
        options: 含秒制心跳间隔的传输选项。
        last_inbound: 与心跳共享的单元素单调时间秒数列表。
        emit: 接收既有无前缀日志的函数。
        now: 返回单调时钟秒数的函数。

    Raises:
        Exception: 子任务失败；进入 finally 后会尝试取消并等待其它任务。
        asyncio.CancelledError: 连接所有者被取消；清理等待本身也可能再次被取消。
    """
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
    """循环连接，不替换适配器或发送队列，也不在断连时取消导航.

    外层持有跨连接事件泵、本协程及状态任务，并注入选定库的 connect。
    任一发送任务失败会唤醒连接所有者，不必等待新入站帧。普通连接异常记录后重试；
    断连回调仍可能覆盖原异常，日志/计时/重试等待异常也不统一吞掉。
    子任务清理由 _connection 尝试完成，不承诺重复取消或回调失败下全部清理成功。

    Args:
        options: 显式服务地址与秒制时间配置。
        adapter: 每次重试复用的进程生命周期适配器。
        outbox: 重试间保留队列和发送槽的同一对象，不保证服务端确认。
        connect: 异步上下文管理器工厂，调用时禁用库级 ping_interval。
        on_connected: 同步启动本连接的观察任务。
        on_disconnected: 异步停止并等待该观察任务，不应取消运动任务。
        emit: 接收无前缀日志文本的函数。
        now: 返回单调时钟秒数的函数。
        sleep: 秒制重试等待实现。

    Raises:
        asyncio.CancelledError: 外层取消向外传播，除非被收尾回调的其它异常覆盖。
        Exception: 未被重试处理包围的时间、日志或等待操作失败。
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
