"""navrun 各模式的单实例归属：立即成功或立即失败，不排队等待.

锁文件从不删除：删掉一个已被锁住的 inode，第二个进程就会在新建的替身文件上
成功加锁，两边都以为自己是唯一的运行。文件内容只作诊断用——是否已有运行在跑
由内核 flock 归属决定，不看文件里可能早已过期的 PID，也不得拿该 PID 去发信号。
"""

from __future__ import annotations

import fcntl
import json
import os
from pathlib import Path
import stat
from typing import Any, Mapping, Optional


DEFAULT_LOCK_PATH = Path('/tmp/navrun.lock')


class LockBusy(RuntimeError):
    """另一个进程持有所请求的导航锁."""


class RunLock:
    """导航独占租约：退出上下文或进程死亡时释放.

    Attributes:
        path: 导航租约对应的锁文件路径；文件长期保留，只有内核归属会变。
    """

    def __init__(self, path: Path = DEFAULT_LOCK_PATH) -> None:
        """只描述一把锁，不创建也不打开文件.

        Args:
            path: 长期保留的锁文件路径；测试必须各自使用私有的临时路径，
                否则会互相争抢同一把真锁。
        """
        self.path = Path(path)
        self._descriptor: Optional[int] = None
        self._owner_pid: Optional[int] = None

    def acquire(self) -> None:
        """立即取得内核归属，失败时不改动任何归属数据.

        三道检查缺一不可：打开时用 O_NOFOLLOW 拒绝软链；加锁成功后再比对路径此刻的
        dev/ino 与已打开的是否仍是同一个文件，防止打开与加锁之间被人换掉；确认之后
        才写入自己的 PID。文件还必须是本人所有、无别名（硬链接数为 1）、组与其他人
        不可写的普通文件。

        Raises:
            LockBusy: 另一个进程已持有这把锁。
            RuntimeError: 本对象已经持有描述符。
            ValueError: 锁文件不是本人所有、无别名的普通文件，或加锁期间路径被换掉。
            OSError: 打开、检查或写入归属信息失败。
        """
        if self._descriptor is not None:
            raise RuntimeError('lock already acquired')
        flags = os.O_RDWR | os.O_CREAT | os.O_CLOEXEC | os.O_NOFOLLOW | os.O_NONBLOCK
        descriptor = os.open(self.path, flags, 0o600)
        try:
            info = os.fstat(descriptor)
            if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid()
                    or info.st_nlink != 1 or info.st_mode & 0o022):
                raise ValueError('lock must be an owned, unaliased regular file')
            try:
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise LockBusy(f'another navrun owns {self.path}') from error
            current = self.path.lstat()
            if (info.st_dev, info.st_ino) != (current.st_dev, current.st_ino):
                raise ValueError('lock path changed while acquiring ownership')
            payload = (json.dumps({'pid': os.getpid()}) + '\n').encode('ascii')
            os.ftruncate(descriptor, 0)
            remaining = memoryview(payload)
            while remaining:
                written = os.write(descriptor, remaining)
                if written <= 0:
                    raise OSError('could not write lock ownership')
                remaining = remaining[written:]
        except BaseException:
            os.close(descriptor)
            raise
        self._descriptor = descriptor
        self._owner_pid = os.getpid()

    def fileno(self) -> int:
        """返回已取得的锁描述符（带 close-on-exec）.

        Returns:
            锁描述符；调用方只能把它显式交给子进程继承，不得自行关闭或解锁。

        Raises:
            RuntimeError: 本对象没有持有锁。
        """
        if self._descriptor is None:
            raise RuntimeError('lock not acquired')
        return self._descriptor

    def assert_owned(self) -> None:
        """确认本进程仍持有这份租约，且路径仍指向当初锁住的那个文件.

        只检查描述符归属与路径身份（dev/ino、属主、权限、无别名），不再向内核重新
        确认 flock：同一进程内没有别的途径会悄悄丢掉它。

        Raises:
            RuntimeError: 归属已释放，或这是 fork 子进程里继承来的副本。
            ValueError: 该路径已不再指向当初锁住的文件。
            OSError: 描述符或路径不可用。
        """
        if self._descriptor is None or self._owner_pid != os.getpid():
            raise RuntimeError('lease is not owned by this process')
        info = os.fstat(self._descriptor)
        current = self.path.lstat()
        if (not stat.S_ISREG(current.st_mode) or current.st_uid != os.geteuid()
                or current.st_nlink != 1 or current.st_mode & 0o022
                or (info.st_dev, info.st_ino) != (current.st_dev, current.st_ino)):
            raise ValueError('lease path identity changed')

    def annotate(self, control: Mapping[str, Any]) -> None:
        """写入有限的发现用元数据，但不因此授予按 PID 发信号的权限.

        整条记录被重写：先从偏移 0 覆盖，再截断到新长度。两步之间读者可能读到新内容
        后面跟着上一条的残尾，所以读者只解析第一行，并且只当诊断信息看——是否有运行
        在跑仍以 flock 归属为准。

        Args:
            control: 协作端点标识与收尾诊断信息；序列化后不得超过 4096 字节，
                且不允许 NaN 或 Infinity。

        Raises:
            ValueError: 元数据超限、含非有限数值，或归属身份已变。
            RuntimeError: 本进程已不再持有这份租约。
            OSError: 元数据没有完整写入。
        """
        self.assert_owned()
        payload = json.dumps({'pid': os.getpid(), 'control': dict(control)},
                             allow_nan=False).encode('utf-8') + b'\n'
        if len(payload) > 4096:
            raise ValueError('lease metadata exceeds limit')
        descriptor = self.fileno()
        if os.pwrite(descriptor, payload, 0) != len(payload):
            raise OSError('incomplete lease metadata write')
        os.ftruncate(descriptor, len(payload))

    def release(self) -> None:
        """关闭归属，但不删除长期保留的锁文件.

        重复调用无害。fork 子进程调用时只关闭自己继承的那个描述符：在子进程里执行
        LOCK_UN 会把父进程共享的同一份租约一起解锁。
        """
        if self._descriptor is None:
            return
        descriptor, owner_pid = self._descriptor, self._owner_pid
        self._descriptor = None
        self._owner_pid = None
        try:
            if owner_pid == os.getpid():
                fcntl.flock(descriptor, fcntl.LOCK_UN)
        finally:
            os.close(descriptor)

    def __enter__(self) -> RunLock:
        """为这个上下文取得归属.

        Returns:
            已取得锁的本对象。
        """
        self.acquire()
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        """运行过程中抛异常也照样释放归属.

        Args:
            exc_type: 异常类型，没有异常时为 None。
            exc_value: 异常对象，没有异常时为 None。
            traceback: 异常栈，没有异常时为 None。
        """
        self.release()
