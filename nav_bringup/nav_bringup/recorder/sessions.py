"""Stop processes that were started as leaders of their own session.

``navrun`` starts helpers such as ``navrun monitor`` and ``navrun
dump-params`` with ``start_new_session=True``. A Ctrl-C in the terminal then
does not reach them directly, and they can be stopped as a group, children
included, without signaling any other process.
"""

from __future__ import annotations

import os
import signal
import subprocess


def stop_session(process: subprocess.Popen, timeout_sec: float) -> int:
    """Interrupt a session, and kill it if it does not exit in time.

    Sends SIGINT to the process group led by ``process``, waits, and sends
    SIGKILL to the group when the timeout expires. A process that already
    exited is not signaled.

    Args:
        process: A process started with ``start_new_session=True``.
        timeout_sec: Seconds to wait after SIGINT.

    Returns:
        The exit status of ``process``, negative for a death by signal as in
        :attr:`subprocess.Popen.returncode`.
    """
    if process.poll() is not None:
        return process.returncode
    _signal_group(process.pid, signal.SIGINT)
    try:
        return process.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        _signal_group(process.pid, signal.SIGKILL)
        return process.wait()


def _signal_group(process_group: int, signum: int) -> None:
    """向调用方选定的辅助进程组发信号，忽略进程组已消失的竞态.

    调用方必须持有自己启动的会话；数字组号本身不是所有权凭据，
    本辅助函数不执行 PID 复用防护，也不查找或选择外部进程。

    Args:
        process_group: 调用方拥有的辅助进程组编号。
        signum: 要发送的信号编号。

    Raises:
        OSError: 除进程组不存在之外的信号发送错误。
    """
    try:
        os.killpg(process_group, signum)
    except ProcessLookupError:
        pass
