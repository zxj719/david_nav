"""键盘侧的本地通道：宣告就绪、等待放行、监视持有方是否还在.

这是一个显式继承的套接字能力，不是路径，也不是环境变量开关。键盘进程必须自己
先校验继承到的导航租约，再使用它。通道只传"就绪"和"放行"，速度值从不经过这里。
"""

from __future__ import annotations

import os
import socket
from typing import Optional


class KeyboardOwnerLost(RuntimeError):
    """持有方消失，或违反了放行协议."""


class KeyboardChannel:
    """键盘启动与存活检查用的套接字副本，关闭时只关自己这一份.

    Attributes:
        source: 显式继承的 Unix 流套接字描述符，不是速度通道。
    """

    def __init__(self, descriptor: int) -> None:
        """只记住用哪个描述符，不打开也不发送任何东西.

        Args:
            descriptor: 显式继承的 Unix 流套接字描述符；必须是 int，且不小于 3。

        Raises:
            ValueError: 描述符类型不对，或指向标准输入输出。
        """
        if type(descriptor) is not int or descriptor < 3:
            raise ValueError('owner channel requires an explicit inherited descriptor')
        self.source = descriptor
        self._socket: Optional[socket.socket] = None

    def __enter__(self) -> KeyboardChannel:
        """复制并校验这条已连接的本地流套接字.

        校验只看协议族、类型和是否已连接，不校验对端身份；随后把副本设为不可继承，
        原描述符保持原样，连阻塞标志都不改。

        Returns:
            本对象；后续收发一律用 MSG_DONTWAIT，不改套接字自身的阻塞标志。

        Raises:
            RuntimeError: 已经打开。
            ValueError: 描述符不是已连接的 Unix 流套接字。
            OSError: 复制或检查描述符失败。
        """
        if self._socket is not None:
            raise RuntimeError('owner channel is already open')
        descriptor = os.dup(self.source)
        try:
            self._socket = socket.socket(fileno=descriptor)
        except BaseException:
            os.close(descriptor)
            raise
        try:
            if (self._socket.family != socket.AF_UNIX
                    or self._socket.getsockopt(socket.SOL_SOCKET, socket.SO_TYPE)
                    != socket.SOCK_STREAM):
                raise ValueError('owner channel must be a Unix stream socket')
            self._socket.getpeername()
            self._socket.set_inheritable(False)
        except BaseException:
            self.close()
            raise
        return self

    def ready(self) -> None:
        """在父进程启动底盘适配之前，宣告发布者独占检查已完成.

        Raises:
            RuntimeError: 上下文已关闭。
            KeyboardOwnerLost: 就绪字节无法立即送达。
        """
        try:
            if self._opened().send(b'R', socket.MSG_DONTWAIT) != 1:
                raise KeyboardOwnerLost('unable to announce keyboard readiness')
        except OSError as error:
            raise KeyboardOwnerLost('owner channel is unavailable') from error

    def permitted(self) -> bool:
        """查询一次放行，能区分"还没放行"与"持有方已断开".

        Returns:
            收到放行字节为 True；还没收到为 False。

        Raises:
            RuntimeError: 上下文已关闭。
            KeyboardOwnerLost: 持有方断开，或发来非约定消息。
        """
        try:
            message = self._opened().recv(1, socket.MSG_DONTWAIT)
        except BlockingIOError:
            return False
        except OSError as error:
            raise KeyboardOwnerLost('owner channel failed') from error
        if message != b'G':
            raise KeyboardOwnerLost('owner disconnected or sent an invalid activation')
        return True

    def check_owner(self) -> None:
        """在处理下一条键盘指令之前，拒绝 EOF 与多余数据.

        用 MSG_PEEK 只看不取：持有方还在、且没有发来东西时立即返回。

        Raises:
            RuntimeError: 上下文已关闭。
            KeyboardOwnerLost: 持有方消失，或发来多余的协议数据。
        """
        try:
            self._opened().recv(1, socket.MSG_PEEK | socket.MSG_DONTWAIT)
        except BlockingIOError:
            return
        except OSError as error:
            raise KeyboardOwnerLost('owner channel failed') from error
        raise KeyboardOwnerLost('owner disconnected or sent unexpected data')

    def _opened(self):
        if self._socket is None:
            raise RuntimeError('owner channel is not open')
        return self._socket

    def close(self) -> None:
        """只关闭这一份副本；不通知持有方，也不动它的锁.

        继承进来的原描述符仍然开着，持有方因此不会立刻读到 EOF；真正的断开发生在
        键盘进程退出、所有副本都关闭之后。
        """
        if self._socket is not None:
            connection, self._socket = self._socket, None
            connection.close()

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        """键盘正常退出或失败退出时，释放这份副本.

        Args:
            exc_type: 异常类型，没有异常时为 None。
            exc: 异常对象，没有异常时为 None。
            traceback: 异常栈，没有异常时为 None。
        """
        self.close()
