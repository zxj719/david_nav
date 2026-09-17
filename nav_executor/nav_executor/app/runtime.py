"""持有外层 ROS/WS 生命周期，按入口、任务、回调及宿主资源顺序收尾."""

import asyncio
from contextlib import suppress
from dataclasses import replace
import time
from types import MappingProxyType
from typing import Callable

from nav_tasks.api import (
    ConfigError, create_dock_ingress, create_runtime, declare_dock_parameters,
    RuntimeAssemblyError,
)

from nav_executor.adapters.ros.host import RosHost
from nav_executor.adapters.transport.outbox import ReliableOutbox
from nav_executor.adapters.transport.proxy import bypass_proxy_for_server
from nav_executor.adapters.transport.ws_client import reconnect
from nav_executor.app.config import ResolvedConfig
from nav_executor.app.logging import log_line, OutboundAudit
from nav_executor.app.status import print_status_loop
from nav_executor.protocol.task_adapter import TaskCommandAdapter


async def _join(tasks):
    """取消所持有的后台任务并收集结束结果，不重新抛出其工作异常.

    Args:
        tasks: 由外层持有、需要一起取消并等待的任务序列。

    Raises:
        asyncio.CancelledError: 本等待协程被取消；本函数不自行屏蔽外部取消。
    """
    for task in tasks:
        task.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)


async def _retry(operation, label, emit, delay):
    """重试清理操作，操作失败或被取消时不把清理误报为成功.

    操作本身的 Exception 和 CancelledError 会进入重试；日志的普通异常被忽略。
    重试间隔内的取消仍可向外传播，持续清理的所有权由外层受屏蔽任务保证。

    Args:
        operation: 每次调用返回可等待清理操作的无参函数。
        label: 失败日志中的资源名称。
        emit: 同步诊断日志函数。
        delay: 重试之间的等待秒数；此处不设置总清理期限。

    Raises:
        asyncio.CancelledError: 在重试等待或日志回调等未捕获位置被取消。
    """
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
    """驱动唯一 ROS 宿主，首个回调错误通知外层并继续为清理处理回调.

    Args:
        host: 持有上下文、节点和执行器的宿主。
        fault: 仅首次异常时写入的 asyncio Future，不在此销毁资源。
        emit: 诊断日志函数，其异常不会打断错误后的清理。

    Raises:
        asyncio.CancelledError: 外层在任务清理完成后取消 spin。
    """
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


async def _finish(host, runtime, rollback, adapter, ingress, dock, workers, spin, emit, delay):
    """按入口、运动、spin、回调和资源顺序清理，失败重试时保留所有权.

    Dock 先 settle 再停止 spin、等待回调，最后销毁运行时和入口；普通导航
    在停止 spin 前关闭运行时。重试没有放弃资源的超时期限。

    Args:
        host: 最后关闭的 ROS 宿主。
        runtime: 已完成构建的任务运行时，或 None。
        rollback: 部分构建失败后仍持有资源的异步清理函数，或 None。
        adapter: 需解除事件订阅的协议适配器，或 None。
        ingress: Dock 入口所有者，或 None。
        dock: 是否采用 Dock 多线程回调收尾顺序。
        workers: 先取消并等待的外层后台任务列表。
        spin: 运动清理期间必须保持运行的 ROS 驱动任务，或 None。
        emit: 清理失败日志函数。
        delay: 清理重试间隔（秒）。

    Raises:
        Exception: 未由重试包装的入口停止或适配器关闭失败。
    """
    if ingress is not None:
        ingress.stop()
    await _join(workers)
    if adapter is not None:
        adapter.close()
    if ingress is not None:
        await _retry(ingress.close, 'Dock ingress', emit, delay)
    if runtime is not None:
        await _retry(runtime.settle if dock else runtime.close, 'task runtime', emit, delay)
    if rollback is not None:
        await _retry(rollback, 'task construction', emit, delay)
    if spin is not None:
        await _join([spin])
    if dock:
        await _retry(host.quiesce, 'ROS callbacks', emit, delay)
        if runtime is not None:
            await _retry(runtime.close, 'task resources', emit, delay)
        if ingress is not None:
            async def destroy_ingress():
                ingress.destroy()
            await _retry(destroy_ingress, 'Dock resources', emit, delay)

    async def close_host():
        host.close()

    await _retry(close_host, 'ROS host', emit, delay)


def _bind_dock(resolved, node):
    """应用 ROS Dock 参数并拒绝入口动作与后端动作解析到同一名称.

    Args:
        resolved: 含完整 Dock 参数的不可变配置。
        node: 用于声明参数与解析 ROS 名称的已创建节点。

    Returns:
        回填有效入口动作名及来源的新配置，不修改原配置。

    Raises:
        ConfigError: Dock 参数无效或入口与后端动作名冲突。
        Exception: ROS 参数声明或名称解析失败。
    """
    dock = declare_dock_parameters(
        node, resolved.dock_parameters, spin_action_name=resolved.dock.runtime.spin_action_name,
        pose_timeout_sec=resolved.dock.runtime.pose_timeout_sec)
    ingress = node.resolve_topic_name(dock.action_name)
    endpoints = (resolved.config.action_name, dock.runtime.spin_action_name)
    if any(ingress == node.resolve_topic_name(name) for name in endpoints):
        raise ConfigError('Dock ingress action must differ from backend action endpoints')
    sources = dict(resolved.sources)
    if dock.action_name != resolved.config.dock_action_name:
        sources['dock_action_name'] = 'ros:action_name'
    return replace(resolved, dock=dock, sources=MappingProxyType(sources),
                   config=replace(resolved.config, dock_action_name=dock.action_name))


async def run(
    resolved: ResolvedConfig, stop: asyncio.Event, *, host_factory: Callable = RosHost,
    runtime_factory: Callable = create_runtime, connect: Callable | None = None,
    emit: Callable[[str], None] = log_line, now: Callable[[], float] = time.time,
    cleanup_retry_sec: float = 1.0,
) -> None:
    """持有一个 POI/Dock 运行时，以独立受屏蔽任务执行关闭流程.

    CLI 负责把信号转换为 stop，嵌入调用方提供自己的事件。进入运行 try 后，
    重复取消等待者不会取消已创建的清理任务；被 _retry 包装的步骤无总放弃期限。
    这不是所有故障必定完成清理的保证：入口 stop、适配器 close 等未包装步骤
    失败仍可让 _finish 抛异常；清理异常也可覆盖最初运行异常。
    前置配置检查、日志、代理环境调整及 host_factory 位于该 try 之前。

    Args:
        resolved: 已校验的任务/传输配置及保留的 ROS 参数。
        stop: 请求有序退出的事件，不直接关闭 ROS 上下文。
        host_factory: 创建空宿主所有者的工厂，可注入纯测试替身。
        runtime_factory: 公共任务工厂，可注入生命周期故障替身。
        connect: 连接上下文工厂，None 时使用当前安装的 WebSocket 库。
        emit: 接收无前缀启动、状态和审计日志的函数。
        now: 既有位姿新鲜度使用的墙钟秒数函数，可注入测试时间戳。
        cleanup_retry_sec: 已包装清理失败后的等待秒数，默认一秒，不是总超时。

    Raises:
        ConfigError: Dock ID 缺完整策略，或有效 ROS 接线不合法。
        asyncio.CancelledError: 外部取消；已进入 try 时先等待清理，清理失败可覆盖取消。
        Exception: 构造、必要后台任务或清理失败。无法证明 ROS 安全收尾时可能需人工处理。
    """
    config = resolved.config
    dock = resolved.dock is not None
    if (config.dock_poi_ids and not dock) or (dock and resolved.dock_parameters is None):
        raise ConfigError('Dock runtime requires complete explicit Dock parameters')
    if stop.is_set():
        return
    if config.websocket_enabled and connect is None:
        import websockets
        emit(f'websockets version={websockets.__version__} file={websockets.__file__}')
        connect = websockets.connect
    if not dock:
        emit(resolved.log_line())
    if config.websocket_enabled:
        bypass_proxy_for_server(config.server_uri)
    host = host_factory()
    runtime = adapter = rollback = spin = ingress = None
    workers = []
    fault = asyncio.get_running_loop().create_future()
    try:
        host.open(resolved.ros_args, **({'dock': True} if dock else {}))
        if dock:
            resolved = _bind_dock(resolved, host.node)
            config = resolved.config
            emit(resolved.log_line())
        spin = asyncio.create_task(_spin(host, fault, emit), name='executor-ros-spin')

        def feedback(*values):
            if ingress is not None:
                ingress.on_feedback(*values)

        try:
            runtime = await runtime_factory(resolved.task_config(), host.node, host.tf_buffer,
                                            log=emit, now=now,
                                            **({'on_dock_feedback': feedback} if dock else {}))
        except RuntimeAssemblyError as error:
            rollback = error.cleanup
            raise
        if stop.is_set():
            return
        if fault.done():
            raise fault.result()
        if dock:
            ingress = create_dock_ingress(runtime.tasks, resolved.dock,
                                          cleanup_retry_sec=cleanup_retry_sec)
            ingress.open(host.node)
        if config.websocket_enabled:
            adapter = TaskCommandAdapter(runtime.tasks, runtime.pois)
            _start_websocket(runtime, config, connect, emit, adapter, workers)
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
            host, runtime, rollback, adapter, ingress, dock, workers, spin, emit,
            cleanup_retry_sec), name='executor-cleanup')
        canceled = False
        while not cleanup.done():
            try:
                await asyncio.shield(cleanup)
            except asyncio.CancelledError:
                canceled = True
        cleanup.result()
        if canceled:
            raise asyncio.CancelledError


def _start_websocket(runtime, config, connect, emit, adapter, workers):
    """创建事件、状态和重连任务，并立即登记到外层持有的列表.

    Args:
        runtime: 提供任务状态、位姿及回环观察器的运行时。
        config: 已解析的执行器配置，时间字段使用秒。
        connect: WebSocket 异步连接上下文工厂。
        emit: 保持现有文本的日志函数。
        adapter: 进程生命周期的命令与事件适配器。
        workers: 原地追加后台任务的所有者列表，部分失败时仍用于清理。

    Raises:
        Exception: 日志输出或任务创建失败；已登记的任务由外层等待回收。
    """
    # Mutate the outer owner's list as each worker starts, including partial failure.
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
