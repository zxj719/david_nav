"""由 CLI 将信号转为有序退出请求，让任务清理期间 ROS 保持可用."""

import asyncio
from contextlib import suppress
import signal
from typing import Callable

from nav_executor.app.config import ResolvedConfig
from nav_executor.app.logging import log_line
from nav_executor.app.runtime import run


async def run_with_signals(
    resolved: ResolvedConfig, *, runner: Callable = run,
    emit: Callable[[str], None] = log_line,
) -> int:
    """把 INT/TERM/HUP 转成同一个停止事件，离开时顺序尝试恢复旧处理器.

    重复信号只再次设置事件，不调用 Task.cancel 或直接关闭 ROS。
    仅用于主线程 CLI；嵌入调用方应直接给 run 提供停止事件。
    处理器恢复不是不可失败事务，某一步失败会跳过后续恢复并传播异常。

    Args:
        resolved: 已校验的执行器配置。
        runner: 外层运行协程，可注入信号生命周期测试替身。
        emit: 无前缀退出日志函数；首次信号日志的普通异常被抑制。

    Returns:
        runner 正常结束时返回 0，或 128 加首次收到的信号编号；
        不以重复信号改变返回码。

    Raises:
        Exception: 运行器、处理器注册或恢复失败。
        asyncio.CancelledError: 本协程被外部直接取消，行为不同于收到停止信号。
    """
    loop = asyncio.get_running_loop()
    stop = asyncio.Event()
    previous = {}
    first = []

    def request(signum):
        if not first:
            first.append(signum)
            with suppress(Exception):
                emit(f'shutdown requested: {signal.Signals(signum).name}; '
                     'waiting for task cleanup')
        stop.set()

    try:
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous[signum] = signal.getsignal(signum)
            loop.add_signal_handler(signum, request, signum)
        await runner(resolved, stop, emit=emit)
        return 128 + first[0] if first else 0
    finally:
        for signum, handler in previous.items():
            loop.remove_signal_handler(signum)
            signal.signal(signum, handler)
