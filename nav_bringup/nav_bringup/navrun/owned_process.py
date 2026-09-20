"""Linux 子进程组：组长 PID 一直被占住，直到清理真正完成.

与 Popen.poll() 不同，waitid(WNOWAIT) 只观察退出而不回收组长。未回收的 PID 把进程组
身份钉死，即使命令本身先于它的子进程退出，这个组号也不会被系统分配给别人——因此这里
从不靠进程名、过期 PID 文件或全系统范围的信号去找目标。

只有"后代始终留在继承来的进程组里"的命令才适合用本类（Humble 的 launch ExecuteProcess
属于这一类）。会把自己脱离出去的辅助进程必须由监督方单独启动、单独持有，
本类无法回收它们。

本对象由创建它的线程独占：其它代码不得等待它的子进程、不得设置 SIGCHLD=SIG_IGN，
也不得用 waitpid(-1) 回收任意子进程，否则组长会被别人抢先回收，本类将失去归属证据。
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
    """一个私有进程组，关停过程有界且幂等.

    close() 可重复调用：已完成关停后再调用不会再次向那个组号发信号。
    """

    def __init__(
        self, argv: Sequence[str], environ: Mapping[str, str], log: Path,
        pass_fds: Sequence[int] = (),
    ) -> None:
        """以新会话启动命令，关闭标准输入并独占一份输出日志.

        新会话意味着子进程不再属于终端的前台组：终端 Ctrl-C 不会直接打到它身上，
        必须由本对象显式发信号。

        Args:
            argv: 精确的参数向量，绝不经过 shell 解释。
            environ: 显式给子进程继承的环境。
            log: 调用方自有运行目录里的新输出文件。
            pass_fds: 只传给这个子进程的显式能力；为空则照旧关闭所有非标准描述符。
                父进程仍然持有这些描述符的所有权。

        Raises:
            ValueError: 参数向量为空，或其中有非字符串、含 NUL 字符的项。
                空字符串本身是合法参数，只会在 exec 阶段失败并抛 OSError。
            RuntimeError: SIGCHLD 被设为忽略，无法安全地持有子进程。
            OSError: 打开日志或执行命令失败。
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
        """int: 被占住的组长 PID；close() 之后它不再构成向该组发信号的依据."""
        return self._process.pid

    def poll(self) -> Optional[int]:
        """观察组长状态，同时不放开它占住的进程组身份.

        组长已退出不等于整组已退出：它的子进程可能仍在运行，回收要走 close()。

        Returns:
            仍在运行时返回 None；已退出时返回退出码，或负的信号编号。

        Raises:
            RuntimeError: 由 fork 出来的子进程调用，而不是创建者本身。
            ChildProcessError: 组长已被别处回收；此时绝不可再向该组号发信号。
        """
        self._check_owner()
        if self._closed:
            return self._process.returncode
        status = os.waitid(os.P_PID, self.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
        if status is None:
            return None
        return status.si_status if status.si_code == os.CLD_EXITED else -status.si_status

    def close(self, timeout_sec: float = 10.0) -> int:
        """中断整组、留出落盘时间、杀掉残余，最后回收组长.

        宽限期内组长始终可等待；组长已经先行退出时，子进程照样会被停掉。
        一次完成的 close 之后不会再向那个组号发信号（组号可能已被系统分配给别人）。
        归属检查不通过时在发信号之前就中止。被托管的命令不得把后代挪到别的进程组，
        否则那些后代不在本方法的回收范围内。

        Args:
            timeout_sec: SIGKILL 之前的宽限期，非负且有限。

        Returns:
            按 subprocess 约定表示的组长退出状态。

        Raises:
            ValueError: 宽限期非法。
            RuntimeError: 由 fork 出来的子进程调用。
            ChildProcessError: 组长被本对象之外的代码回收。
            OSError: 无法向自有进程组发信号。
            subprocess.TimeoutExpired: 被杀的组长在五秒内仍不可回收。
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
