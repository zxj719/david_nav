"""CLI-owned graceful signals; ROS remains alive while task cancellation settles."""

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
    """Translate INT/TERM/HUP to one graceful stop request and restore prior handlers.

    Repeated signals do not cancel cleanup or shut down ROS. This CLI-only helper
    requires the main thread; embedding callers should call run with their event.

    Args:
        resolved: Validated executor configuration.
        runner: Outer runtime coroutine, injectable for signal lifecycle tests.
        emit: Unprefixed shutdown log sink.

    Returns:
        Zero on a normal return or 128 plus the first received signal number.
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
