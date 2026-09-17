"""通过显式持有的描述符读取交互终端，不依赖 launch 管理的标准输入."""

import math
import os
import select
import termios
import time
import tty
from typing import Any, Optional


class TerminalUnavailable(RuntimeError):
    """交互终端打开、读取、配置或恢复失败时使用的运行错误."""


class TerminalKeyboard:
    """持有一个不跨 exec 继承的终端描述符和有界转义序列缓冲区.

    Attributes:
        device: 待打开的终端设备路径；本对象只持有自己的描述符。
    """

    def __init__(self, device: str = '/dev/tty', descriptor: Optional[int] = None) -> None:
        """保存终端选择，不立即打开或复制任何描述符.

        Args:
            device: 显式终端路径；测试应使用自己创建的伪终端。
            descriptor: 可选监督者已打开的终端描述符；进入上下文时使用其副本，
                调用方仍持有原始描述符。本构造只检查整数范围，不验证其当前可用性。

        Raises:
            ValueError: 给出的描述符不是大于二的严格 int 值，包括拒绝 bool。
        """
        if descriptor is not None and (type(descriptor) is not int or descriptor < 3):
            raise ValueError('terminal descriptor must be an integer greater than two')
        self.device = device
        self._source = descriptor
        self._fd: Optional[int] = None
        self._settings = None
        self._sequence = ''
        self._overflow = False

    def __enter__(self) -> 'TerminalKeyboard':
        """打开或复制终端，保存属性并切到逐字符读取模式.

        自己的描述符设置为不可继承；使用 cbreak 而非宣称完整 raw 模式。
        失败时调用 close 尝试恢复和关闭，清理失败可能覆盖原打开错误。

        Returns:
            本读取器实例，后续操作使用自己的描述符。

        Raises:
            RuntimeError: 读取器已经打开。
            TerminalUnavailable: 设备不可用、不是 TTY 或配置、属性恢复失败。
            OSError: 失败后的描述符关闭本身出错。
        """
        if self._fd is not None:
            raise RuntimeError('terminal reader is already open')
        try:
            flags = os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC
            self._fd = (os.open(self.device, flags) if self._source is None
                        else os.dup(self._source))
            os.set_inheritable(self._fd, False)
            if not os.isatty(self._fd):
                raise TerminalUnavailable(f'TTY required: {self.device}')
            self._settings = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
            self._sequence, self._overflow = '', False
            return self
        except (OSError, termios.error, TerminalUnavailable) as error:
            self.close()
            raise TerminalUnavailable(
                f'Cannot open interactive TTY {self.device}: {error}') from error

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        """离开上下文时调用 close 恢复终端并关闭副本，不抑制调用方异常.

        Args:
            exc_type: 上下文中的异常类型，此处不作分支判断。
            exc: 上下文中的异常对象。
            traceback: 上下文中的异常回溯。

        Raises:
            TerminalUnavailable: 恢复终端属性失败。
            OSError: 关闭描述符失败，可能覆盖上下文原异常。
        """
        self.close()

    def fileno(self) -> int:
        """返回当前由读取器持有的描述符，不返回调用方传入的原描述符.

        Returns:
            当前文件描述符；它不依赖标准输入是否重定向。

        Raises:
            RuntimeError: 尚未打开或已经关闭。
        """
        if self._fd is None:
            raise RuntimeError('terminal reader is not open')
        return self._fd

    def close(self) -> None:
        """先清除本对象的持有记录，再尝试恢复终端属性和关闭描述符.

        没有持有描述符时直接返回；即使恢复失败，finally 仍尝试关闭。
        关闭失败后本对象也不再记录该描述符，不能把再次调用无操作当作关闭成功证明。

        Raises:
            TerminalUnavailable: 恢复属性失败，随后仍尝试关闭。
            OSError: 描述符关闭失败，可能覆盖恢复异常。
        """
        descriptor, settings = self._fd, self._settings
        self._fd, self._settings = None, None
        if descriptor is None:
            return
        try:
            if settings is not None:
                termios.tcsetattr(descriptor, termios.TCSANOW, settings)
        except (OSError, termios.error) as error:
            raise TerminalUnavailable(f'Unable to restore TTY {self.device}: {error}') from error
        finally:
            os.close(descriptor)

    def _consume(self, character: str) -> Optional[str]:
        """消费一个 ASCII 字符，跨读取保留有界转义序列而不把碎片当按键.

        Args:
            character: read_key 已解码的单个非空 ASCII 字符。

        Returns:
            普通字符或已完成的转义序列；未完成时为 None，超出 32 字符
            缓冲区的序列完成时返回空串并复位，不把尾部误作运动按键。
        """
        if not self._sequence:
            if character == '\x1b':
                self._sequence = character
                return None
            return character
        if len(self._sequence) < 32:
            self._sequence += character
        else:
            self._overflow = True
        complete = (len(self._sequence) == 2 and character not in ('[', 'O'))
        complete |= len(self._sequence) >= 3 and '@' <= character <= '~'
        if complete:
            result = '' if self._overflow else self._sequence
            self._sequence, self._overflow = '', False
            return result
        return None

    def read_key(self, timeout_sec: float) -> str:
        """在有限轮询预算内读取一个完整按键，保留跨轮询的转义序列碎片.

        使用单调时钟，最多消耗 256 字节；ASCII 解码忽略其它字节，
        转义缓冲最多保留 32 个 ASCII 字符，防止截断尾部被当成独立运动按键。

        Args:
            timeout_sec: 有限非负轮询时长，单位秒；零允许一次即时可读检查。

        Returns:
            一个字符或完整序列；无输入、未完成、超长或达到本次字节预算时为空串。

        Raises:
            RuntimeError: 读取器尚未打开或已经关闭。
            ValueError: 超时参数无效。
            TerminalUnavailable: 终端断开，或读取、终端操作失败。
        """
        descriptor = self.fileno()
        if (isinstance(timeout_sec, bool) or not isinstance(timeout_sec, (int, float))
                or not math.isfinite(timeout_sec) or timeout_sec < 0.0):
            raise ValueError('terminal timeout must be finite and nonnegative')
        deadline = time.monotonic() + timeout_sec
        try:
            for _ in range(256):
                wait = max(deadline - time.monotonic(), 0)
                ready, _, _ = select.select([descriptor], [], [], wait)
                if not ready:
                    return ''
                data = os.read(descriptor, 1)
                if not data:
                    raise TerminalUnavailable('TTY disconnected')
                character = data.decode('ascii', errors='ignore')
                if character:
                    result = self._consume(character)
                    if result is not None:
                        return result
                if time.monotonic() >= deadline:
                    return ''
        except (OSError, termios.error) as error:
            raise TerminalUnavailable(f'TTY read failed: {error}') from error
        return ''
