"""Parent-owned terminal snapshot and explicit keyboard activation capabilities."""

from __future__ import annotations

import os
import socket
import termios
from typing import Optional, Sequence, Tuple

from nav_bringup.navrun.locking import RunLock


class KeyboardSession:
    """Retain terminal recovery state until every owned child has drained."""

    def __init__(self, lease: RunLock, device: str) -> None:
        """Describe the handoff without opening any terminal or socket.

        Args:
            lease: Parent's acquired navigation lock, never released by this object.
            device: Profile-selected terminal, opened in the parent's original session.
        """
        self.lease, self.device = lease, device
        self._tty: Optional[int] = None
        self._settings = None
        self._parent: Optional[socket.socket] = None
        self._child: Optional[socket.socket] = None

    def __enter__(self) -> KeyboardSession:
        """Capture terminal state before any child or platform operation starts.

        Returns:
            The parent-owned, close-on-exec capability set.

        Raises:
            RuntimeError: Not locked or already open.
            ValueError: The selected device is not a terminal.
            OSError: Opening or inspecting the terminal fails.
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
        """Bind inherited descriptors for exactly one directly owned keyboard child.

        Args:
            argv: Managed keyboard command including its frozen ROS parameters.

        Returns:
            Argument vector and explicit pass_fds tuple for OwnedProcess.

        Raises:
            RuntimeError: The session is closed or already handed off.
        """
        if self._tty is None or self._child is None:
            raise RuntimeError('keyboard capabilities are not available')
        descriptors = (self.lease.fileno(), self._child.fileno(), self._tty)
        arguments = tuple(argv)
        for name, descriptor in zip(('lock', 'control', 'tty'), descriptors):
            arguments += (f'--{name}-fd', str(descriptor))
        return arguments, descriptors

    def child_started(self) -> None:
        """Close the parent's spare child endpoint so child death produces real EOF."""
        if self._child is not None:
            self._child.close()
            self._child = None

    def ready(self) -> bool:
        """Poll the child's completed publisher-exclusion handshake.

        Returns:
            True for readiness; False while discovery is still pending.

        Raises:
            RuntimeError: The child disconnected or sent invalid protocol data.
            OSError: The local channel failed.
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
        """Activate only after the parent has started its owned chassis graph.

        Raises:
            RuntimeError: The owner channel is closed or permission cannot be sent.
            OSError: The keyboard disconnected.
        """
        if self._parent is None or self._parent.send(b'G', socket.MSG_DONTWAIT) != 1:
            raise RuntimeError('keyboard activation failed')

    def revoke(self) -> None:
        """Request keyboard shutdown by dropping only the parent-owned socket endpoint."""
        if self._parent is not None:
            self._parent.close()
            self._parent = None

    def close(self) -> None:
        """Restore the saved terminal after child cleanup, without unlocking the lease.

        Raises:
            OSError: Restoration failed; descriptors are closed regardless.
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
        """Restore and close only after the caller has drained all owned children.

        Args:
            exc_type: Context exception class.
            exc: Context exception instance.
            traceback: Context exception traceback.
        """
        self.close()
