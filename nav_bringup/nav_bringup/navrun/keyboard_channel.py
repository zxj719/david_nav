"""Inherited, local keyboard readiness and supervisor-liveness channel.

This is an explicit socket capability, never a pathname or environment flag.
The keyboard must independently validate its inherited navigation lease before
using it. Only ready/permit are transferred; velocity values are never carried.
"""

from __future__ import annotations

import os
import socket
from typing import Optional


class KeyboardOwnerLost(RuntimeError):
    """The owning supervisor disappeared or violated the activation protocol."""


class KeyboardChannel:
    """Close-only socket duplicate for a keyboard's startup and liveness checks."""

    def __init__(self, descriptor: int) -> None:
        """Store a socket selector without opening or sending anything.

        Args:
            descriptor: Explicit inherited Unix stream socket descriptor.

        Raises:
            ValueError: Descriptor is malformed or denotes a standard stream.
        """
        if type(descriptor) is not int or descriptor < 3:
            raise ValueError('owner channel requires an explicit inherited descriptor')
        self.source = descriptor
        self._socket: Optional[socket.socket] = None

    def __enter__(self) -> KeyboardChannel:
        """Duplicate and validate the connected local stream socket.

        Returns:
            This nonblocking channel.

        Raises:
            RuntimeError: Already open.
            ValueError: The descriptor is not a connected Unix stream socket.
            OSError: Duplicating or inspecting the descriptor fails.
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
        """Announce completed publisher exclusion before the parent starts its adapter.

        Raises:
            RuntimeError: The context is closed.
            KeyboardOwnerLost: The ready byte cannot be delivered immediately.
        """
        try:
            if self._opened().send(b'R', socket.MSG_DONTWAIT) != 1:
                raise KeyboardOwnerLost('unable to announce keyboard readiness')
        except OSError as error:
            raise KeyboardOwnerLost('owner channel is unavailable') from error

    def permitted(self) -> bool:
        """Poll once for activation, distinguishing a pending owner from disconnection.

        Returns:
            True for the parent's activation byte; False while it has not arrived.

        Raises:
            RuntimeError: The context is closed.
            KeyboardOwnerLost: The parent disconnected or sent an invalid message.
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
        """Reject EOF or unexpected data before processing another keyboard command.

        Raises:
            RuntimeError: The context is closed.
            KeyboardOwnerLost: The parent disappeared or sent extra protocol data.
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
        """Close this duplicate only; never signal the parent or change its lock."""
        if self._socket is not None:
            connection, self._socket = self._socket, None
            connection.close()

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        """Release the duplicate on normal or failed keyboard shutdown.

        Args:
            exc_type: Context exception class.
            exc: Context exception instance.
            traceback: Context exception traceback.
        """
        self.close()
