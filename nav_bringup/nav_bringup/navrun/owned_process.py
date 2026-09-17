"""Linux child groups whose leader PID stays reserved until cleanup is complete.

Unlike Popen.poll(), waitid(WNOWAIT) observes exit without reaping the leader.
Its unreaped PID pins the process-group identity even if the command exits
before its children. No process name, stale PID file or system-wide signal is
used. Only commands whose descendants stay in their inherited process group
belong here (including Humble launch ExecuteProcess). Independently detached
helpers must instead be started and owned separately by the supervisor.

The creating thread owns this object exclusively. No other code may wait for
its child, install SIGCHLD=SIG_IGN, or reap arbitrary children with waitpid(-1).
"""

from __future__ import annotations

import math
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Mapping, Optional, Sequence


class OwnedProcess:
    """One private process group with bounded, idempotent shutdown."""

    def __init__(
        self, argv: Sequence[str], environ: Mapping[str, str], log: Path,
        pass_fds: Sequence[int] = (),
    ) -> None:
        """Start a command with a new session, closed stdin and exclusive output log.

        Args:
            argv: Exact argument vector, never interpreted by a shell.
            environ: Explicit inherited environment for the child.
            log: New output file within the caller-owned run directory.
            pass_fds: Explicit capabilities for this child only; empty closes all
                nonstandard descriptors, as before. The parent retains ownership.

        Raises:
            ValueError: The command is empty or contains invalid arguments.
            RuntimeError: SIGCHLD is ignored and safe child ownership is unavailable.
            OSError: Opening the log or executing the command fails.
        """
        if not argv or any(not isinstance(arg, str) or '\0' in arg for arg in argv):
            raise ValueError('expected a nonempty argument vector without NUL')
        if any(type(fd) is not int or fd < 3 for fd in pass_fds):
            raise ValueError('passed descriptors must be integers greater than two')
        for descriptor in pass_fds:
            os.fstat(descriptor)
        if signal.getsignal(signal.SIGCHLD) == signal.SIG_IGN:
            raise RuntimeError('owned processes require waitable children')
        self._owner_pid = os.getpid()
        self._closed = False
        with Path(log).open('xb') as output:
            self._process = subprocess.Popen(
                list(argv), env=dict(environ), stdin=subprocess.DEVNULL,
                stdout=output, stderr=subprocess.STDOUT, start_new_session=True,
                close_fds=True, pass_fds=tuple(pass_fds))

    @property
    def pid(self) -> int:
        """int: The reserved leader PID, not authority after close()."""
        return self._process.pid

    def poll(self) -> Optional[int]:
        """Observe the leader status without releasing its process-group identity.

        Returns:
            None while running; exit code, or negative signal number after exit.

        Raises:
            RuntimeError: Called by a fork child rather than the creating process.
            ChildProcessError: Some other code reaped the leader; do not signal it.
        """
        self._check_owner()
        if self._closed:
            return self._process.returncode
        status = os.waitid(os.P_PID, self.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
        if status is None:
            return None
        return status.si_status if status.si_code == os.CLD_EXITED else -status.si_status

    def close(self, timeout_sec: float = 10.0) -> int:
        """Interrupt the group, allow flushing, kill leftovers, then reap the leader.

        The leader remains waitable throughout the grace period. Children are
        still stopped when it has already exited; a completed close never
        signals that numeric group again. Failed ownership checks abort before
        signaling. Commands must not detach descendants into another group.

        Args:
            timeout_sec: Nonnegative finite grace period before SIGKILL.

        Returns:
            Leader exit status using subprocess conventions.

        Raises:
            ValueError: The grace period is invalid.
            RuntimeError: Called by a fork child.
            ChildProcessError: The leader was reaped outside this object.
            OSError: The owned group could not be signaled.
            subprocess.TimeoutExpired: A killed leader is not reapable within five seconds.
        """
        if (type(timeout_sec) not in (int, float) or not math.isfinite(timeout_sec)
                or timeout_sec < 0):
            raise ValueError('timeout_sec must be nonnegative and finite')
        self._check_owner()
        if self._closed:
            return self._process.returncode
        self.poll()  # Verify that this leader remains our waitable child.
        self._signal(signal.SIGINT)
        deadline = time.monotonic() + timeout_sec
        while _group_running(self.pid):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(remaining, 0.05))
        self.poll()  # Recheck ownership before escalation, still without reaping.
        self._signal(signal.SIGKILL)
        killed_deadline = time.monotonic() + 5.0
        while _group_running(self.pid):
            if time.monotonic() >= killed_deadline:
                # Keep the leader reserved if even SIGKILL has not drained the
                # group (for example an uninterruptible child). A retry remains
                # safe; releasing ownership at this point would not be safe.
                raise subprocess.TimeoutExpired('owned process group', 5.0)
            time.sleep(0.05)
        status = self._process.wait(timeout=5.0)
        self._closed = True
        return status

    def _check_owner(self) -> None:
        """拒绝 fork 后的子进程使用继承对象操作原进程的子进程组.

        这里只核创建者 PID；领导进程仍可等待等条件由 poll 和 close 另行核对。

        Raises:
            RuntimeError: 当前进程不是创建此对象的进程。
        """
        if self._owner_pid != os.getpid():
            raise RuntimeError('only the creating process owns this child group')

    def _signal(self, signum: int) -> None:
        """向本对象记录的子进程组发送信号，只忽略组已经消失的竞态.

        调用方必须先核创建者与仍可等待的领导进程，且尚未完成 close；
        本辅助函数自身不验证数字 PID 的所有权，不可用于任意外部进程。

        Args:
            signum: 向整个自有进程组发送的信号编号。

        Raises:
            OSError: 除进程组不存在之外的信号发送错误。
        """
        try:
            os.killpg(self.pid, signum)
        except ProcessLookupError:
            pass


def _group_running(group: int) -> bool:
    # /proc stat exposes group and state without inspecting argv or secrets.
    # Insufficient visibility only costs the remaining grace period: it never
    # authorizes signaling another group or implies that children have exited.
    """只读检查给定进程组是否还有非僵尸成员，观察不确定时保守返回存活.

    Args:
        group: 外层已经确认由自己持有的进程组号。

    Returns:
        有成员或 /proc 读取/解析不完整时为 True；完整扫描无成员才为 False。
        不读 argv、不扩大信号目标，也不在此发送信号。
    """
    try:
        for entry in Path('/proc').iterdir():
            if not entry.name.isdecimal():
                continue
            try:
                fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
                if int(fields[2]) == group and fields[0] not in ('Z', 'X'):
                    return True
            except (FileNotFoundError, ProcessLookupError):
                continue
            except (OSError, ValueError, IndexError):
                return True
    except OSError:
        return True
    return False
