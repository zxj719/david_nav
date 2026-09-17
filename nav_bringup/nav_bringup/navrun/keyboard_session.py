"""父进程持有的终端快照，以及显式的键盘启用能力.

父进程在启动任何子进程之前先打开终端并保存 termios 设置，子进程崩溃时由父进程
负责恢复。交给子进程的是显式继承的描述符（租约、控制套接字、终端），不是路径，
也不是环境变量开关；控制套接字只传"就绪"和"放行"两个字节，速度指令从不走它。
"""

from __future__ import annotations

import os
import socket
import termios
from typing import Optional, Sequence, Tuple

from nav_bringup.navrun.locking import RunLock


class KeyboardSession:
    """保留终端恢复状态，直到自己启动的子进程全部收尾.

    Attributes:
        device: 配置选择的终端设备路径。
        lease: 父进程已持有的导航锁，本对象只借用它的描述符，从不释放它。
    """

    def __init__(self, lease: RunLock, device: str) -> None:
        """只描述这次交接，不打开终端也不建套接字.

        Args:
            lease: 父进程已取得的导航锁；本对象不会释放它。
            device: 档案选定的终端，在父进程原来的会话里打开。
        """
        self.lease, self.device = lease, device
        self._tty: Optional[int] = None
        self._settings = None
        self._parent: Optional[socket.socket] = None
        self._child: Optional[socket.socket] = None

    def __enter__(self) -> KeyboardSession:
        """在任何子进程或平台操作开始之前先抓取终端状态.

        只确认父进程手上有锁描述符，不重新校验锁的路径身份；随后以只读、不接管控制
        终端的方式打开设备，保存当前 termios 设置，并建立一对本地控制套接字。

        Returns:
            已备好能力集的本对象，描述符均为 close-on-exec。

        Raises:
            RuntimeError: 父进程没有持锁，或本对象已经打开。
            ValueError: 选定的设备不是终端。
            OSError: 打开或检查终端失败。
        """
        self.lease.fileno()
        if self._tty is not None:
            raise RuntimeError('keyboard session is already open')
        try:
            self._tty = os.open(self.device, os.O_RDONLY | os.O_NOCTTY
                                | os.O_NONBLOCK | os.O_CLOEXEC)
            if not os.isatty(self._tty):
                raise ValueError('interactive TTY required for teleop')
            self._settings = termios.tcgetattr(self._tty)
            self._parent, self._child = socket.socketpair()
            return self
        except BaseException:
            self.close()
            raise

    def command(self, argv: Sequence[str]) -> Tuple[Tuple[str, ...], Tuple[int, ...]]:
        """绑定要继承的描述符，只供一个由本进程直接启动的键盘子进程使用.

        Args:
            argv: 受管的键盘命令，含其已固化的 ROS 参数。

        Returns:
            补好 --lock-fd、--control-fd、--tty-fd 的参数元组，以及需要显式传递的
            描述符元组（供 OwnedProcess 的 pass_fds 使用）。

        Raises:
            RuntimeError: 会话未打开，或子进程端点已经交出去了。
        """
        if self._tty is None or self._child is None:
            raise RuntimeError('keyboard capabilities are not available')
        descriptors = (self.lease.fileno(), self._child.fileno(), self._tty)
        arguments = tuple(argv)
        for name, descriptor in zip(('lock', 'control', 'tty'), descriptors):
            arguments += (f'--{name}-fd', str(descriptor))
        return arguments, descriptors

    def child_started(self) -> None:
        """关掉父进程手里多余的那一端，好让子进程死亡时能读到真正的 EOF."""
        if self._child is not None:
            self._child.close()
            self._child = None

    def ready(self) -> bool:
        """查询一次子进程的"独占发布者"握手是否已完成.

        Returns:
            收到就绪字节为 True；仍在发现阶段为 False。

        Raises:
            RuntimeError: 子进程在就绪之前断开，或发来非约定数据。
            OSError: 本地通道失败。
        """
        if self._parent is None:
            raise RuntimeError('keyboard owner channel is closed')
        try:
            message = self._parent.recv(1, socket.MSG_DONTWAIT)
        except BlockingIOError:
            return False
        if message != b'R':
            raise RuntimeError('keyboard disconnected before readiness or sent invalid data')
        return True

    def permit(self) -> None:
        """等父进程自己的底盘图起来之后，才放行键盘.

        Raises:
            RuntimeError: 控制通道已关闭，或放行字节没能立即送达。
            OSError: 键盘子进程已断开。
        """
        if self._parent is None or self._parent.send(b'G', socket.MSG_DONTWAIT) != 1:
            raise RuntimeError('keyboard activation failed')

    def revoke(self) -> None:
        """只关闭父进程这一端的套接字，以此要求键盘退出."""
        if self._parent is not None:
            self._parent.close()
            self._parent = None

    def close(self) -> None:
        """在子进程收尾之后恢复终端设置，且不解锁租约.

        调用方必须先把自己启动的子进程全部收掉：子进程若还活着并继续改终端设置，
        这里恢复的结果会被它再次覆盖。无论恢复是否成功，描述符都会关闭。

        Raises:
            OSError: 恢复终端设置失败。
        """
        self.revoke()
        self.child_started()
        descriptor, settings = self._tty, self._settings
        self._tty, self._settings = None, None
        if descriptor is not None:
            try:
                if settings is not None:
                    termios.tcsetattr(descriptor, termios.TCSANOW, settings)
            finally:
                os.close(descriptor)

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        """等调用方收完所有子进程之后，再恢复并关闭.

        Args:
            exc_type: 异常类型，没有异常时为 None。
            exc: 异常对象，没有异常时为 None。
            traceback: 异常栈，没有异常时为 None。
        """
        self.close()
