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
    try:
        os.killpg(process_group, signum)
    except ProcessLookupError:
        pass
