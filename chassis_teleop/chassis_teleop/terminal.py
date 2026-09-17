"""Read an explicitly owned terminal without relying on launch-managed stdin."""

import math
import os
import select
import termios
import time
import tty
from typing import Any, Optional


class TerminalUnavailable(RuntimeError):
    """An interactive terminal cannot be opened, read or restored."""


class TerminalKeyboard:
    """Own one close-on-exec terminal descriptor and a bounded escape-sequence buffer.

    Attributes:
        device: 待打开的终端设备路径；本对象只持有自己的描述符。
    """

    def __init__(self, device: str = '/dev/tty', descriptor: Optional[int] = None) -> None:
        """Store the requested device without opening anything.

        Args:
            device: Explicit terminal path; tests must use their own pseudoterminal.
            descriptor: Optional supervisor-opened terminal capability. Its duplicate
                is used instead of opening device; the caller retains the original.

        Raises:
            ValueError: An inherited descriptor is not an explicit nonstandard integer.
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
        """Open the terminal, save its settings and enable character-at-a-time input.

        Returns:
            This reader, using its own noninheritable descriptor.

        Raises:
            RuntimeError: The reader is already open.
            TerminalUnavailable: The path is unavailable, not a TTY or cannot be configured.
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
        """Restore terminal settings even when the caller exits with an exception.

        Args:
            exc_type: Context manager exception type.
            exc: Context manager exception value.
            traceback: Context manager traceback.
        """
        self.close()

    def fileno(self) -> int:
        """Return the currently owned descriptor.

        Returns:
            File descriptor, never stdin's descriptor.

        Raises:
            RuntimeError: The reader has not been opened or was closed.
        """
        if self._fd is None:
            raise RuntimeError('terminal reader is not open')
        return self._fd

    def close(self) -> None:
        """Restore settings and close our descriptor, idempotently.

        Raises:
            TerminalUnavailable: Restoring attributes failed; descriptor is still closed.
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
        """Read a complete event within a bounded poll, retaining fragmented escape input.

        Args:
            timeout_sec: Finite nonnegative maximum polling time.

        Returns:
            One key/sequence, or empty when incomplete/idle/overlong. The escape
            buffer is capped at 32 bytes and each poll at 256 consumed bytes.

        Raises:
            RuntimeError: The reader is closed.
            ValueError: The timeout is invalid.
            TerminalUnavailable: The terminal disconnects or cannot be read.
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
