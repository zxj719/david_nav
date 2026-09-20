"""校验继承来的 Linux flock 能力，既不加锁也不解锁.

监督方始终持有自己的 RunLock；它唯一的键盘子进程显式继承同一个打开文件描述，
凭据是这个描述本身，而不是 PID、进程名或环境变量豁免。Linux 的 fdinfo 把 flock
记录关联到该描述上，独立地再打开同一路径不会继承这条记录。本模块不发出任何
flock 系统调用，因此校验通过不等于本进程新获得了锁，校验失败也不会释放监督方的锁。
"""

from __future__ import annotations

import os
from pathlib import Path
import stat
from typing import Optional

from nav_bringup.navrun.locking import DEFAULT_LOCK_PATH


class DelegatedLock:
    """已被独占持有的导航租约的只读副本，只能关闭，不能解锁.

    Attributes:
        path: 预期导航锁文件的绝对路径。
        source: 显式继承的原始描述符；本对象仅管理其副本。
    """

    def __init__(self, descriptor: int, path: Path = DEFAULT_LOCK_PATH) -> None:
        """只记下能力选择子，不打开也不改动任何资源.

        构造阶段不做 dup、不读 /proc、不校验描述符是否真的持有锁；
        这些都推迟到进入上下文时进行。

        Args:
            descriptor: 显式继承的描述符，不接受标准输入/输出/错误。
            path: 预期的导航锁路径；测试用私有文件。

        Raises:
            ValueError: 描述符非法或路径不是绝对路径。
        """
        if type(descriptor) is not int or descriptor < 3 or not Path(path).is_absolute():
            raise ValueError('delegation requires an explicit descriptor and absolute lock path')
        self.source = descriptor
        self.path = Path(path)
        self._descriptor: Optional[int] = None

    def __enter__(self) -> DelegatedLock:
        """复制描述符并校验独占归属，全程不改动共享的 flock 记录.

        校验失败时只关闭本次复制出来的副本，监督方的锁不受影响。

        Returns:
            已校验的副本，带 close-on-exec，不会被后续 exec 继承。

        Raises:
            RuntimeError: 上下文已经进入过。
            ValueError: 不是预期的、被独占持有的那个打开文件描述。
            OSError: 描述符、路径或 Linux 归属证据读不到。
        """
        if self._descriptor is not None:
            raise RuntimeError('delegated lock is already open')
        self._descriptor = os.dup(self.source)
        try:
            os.set_inheritable(self._descriptor, False)
            self.verify()
        except BaseException:
            self.close()
            raise
        return self

    def fileno(self) -> int:
        """返回本对象自己的副本，而不是监督方的原描述符.

        Returns:
            本对象持有的副本；调用方只可读取与关闭，绝不可对它解锁。

        Raises:
            RuntimeError: 上下文尚未进入或已关闭。
        """
        if self._descriptor is None:
            raise RuntimeError('delegated lock is not open')
        return self._descriptor

    def verify(self) -> None:
        """重新核对路径身份与独占 flock 证据，不发出任何加锁系统调用.

        证据取自 /proc/self/fdinfo 的快照：只说明这一刻该描述仍持有写锁，
        不保证之后不被监督方释放，因此需要在关键时刻重新核对。

        Raises:
            RuntimeError: 上下文已关闭。
            ValueError: 归属已改变，或证据不是独占写锁。
            OSError: 读不到所需的 Linux 归属证据。
        """
        descriptor = self.fileno()
        info, current = os.fstat(descriptor), self.path.lstat()
        if (not stat.S_ISREG(info.st_mode) or not stat.S_ISREG(current.st_mode)
                or info.st_uid != os.geteuid() or info.st_nlink != 1 or info.st_mode & 0o022
                or (info.st_dev, info.st_ino) != (current.st_dev, current.st_ino)):
            raise ValueError('delegated lock is not the expected owned file')
        evidence = Path('/proc/self/fdinfo', str(descriptor)).read_text(encoding='ascii')
        locks = [line.split() for line in evidence.splitlines() if line.startswith('lock:')]
        if not any(len(fields) == 9 and fields[2:5] == ['FLOCK', 'ADVISORY', 'WRITE']
                   and fields[-2:] == ['0', 'EOF'] for fields in locks):
            raise ValueError('open file description does not hold an exclusively held flock')

    def close(self) -> None:
        """只关闭本对象的副本，绝不对共享描述发 LOCK_UN.

        可重复调用：未进入上下文或已关闭时不做任何事，也不报错。
        """
        if self._descriptor is not None:
            descriptor, self._descriptor = self._descriptor, None
            os.close(descriptor)

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        """即使键盘子进程启动失败，也只丢弃本副本.

        不吞异常（返回 None），不解锁，也不改动锁文件本身。

        Args:
            exc_type: 上下文异常类。
            exc: 上下文异常实例。
            traceback: 上下文异常回溯。
        """
        self.close()
