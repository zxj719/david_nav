"""停止那些以自身会话首进程身份启动的进程.

``navrun`` 用 ``start_new_session=True`` 启动 ``navrun monitor``、
``navrun dump-params`` 这类辅助进程。终端里的 Ctrl-C 因此不会直接打到它们身上，
而它们可以连同各自的子进程一起按组停掉，不会波及任何其它进程。
"""

from __future__ import annotations

import os
import signal
import subprocess


def stop_session(process: subprocess.Popen, timeout_sec: float) -> int:
    """中断一个会话；到时仍未退出就杀掉.

    先向以 ``process`` 为首的进程组发 SIGINT，等待，超时后再向该组发 SIGKILL。
    已经退出的进程不会被发信号。只处理这一个会话，不触碰别的进程组。

    Args:
        process: 以 ``start_new_session=True`` 启动的进程。
        timeout_sec: SIGINT 之后等待的秒数。

    Returns:
        ``process`` 的退出状态；被信号杀死时为负值，与
        :attr:`subprocess.Popen.returncode` 一致。
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
