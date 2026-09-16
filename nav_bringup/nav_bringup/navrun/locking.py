"""Nonblocking single-instance ownership for every navrun mode.

The persistent file is never unlinked: deleting a locked inode would let a
second process lock a replacement inode. File contents are diagnostic only;
kernel flock ownership, not a possibly stale PID, decides whether a run exists.
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
    """Another process owns the requested navigation lock."""


class RunLock:
    """Exclusive navigation lease released on context exit or process death."""

    def __init__(self, path: Path = DEFAULT_LOCK_PATH) -> None:
        """Describe a lock without creating or opening a file.

        Args:
            path: Persistent lock file; tests must use a private temporary path.
        """
        self.path = Path(path)
        self._descriptor: Optional[int] = None
        self._owner_pid: Optional[int] = None

    def acquire(self) -> None:
        """Acquire kernel ownership immediately or fail without changing owner data.

        Raises:
            LockBusy: Another process holds this lock.
            RuntimeError: This object already owns a descriptor.
            ValueError: The lock is not an unaliased regular file owned by this user.
            OSError: Opening, inspecting or recording ownership fails.
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
        """Return the acquired, close-on-exec descriptor.

        Returns:
            The descriptor; callers must not close or unlock it themselves.

        Raises:
            RuntimeError: This object does not hold a lock.
        """
        if self._descriptor is None:
            raise RuntimeError('lock not acquired')
        return self._descriptor

    def assert_owned(self) -> None:
        """Verify this process still owns the pinned lease object and path identity.

        Raises:
            RuntimeError: Ownership was released or this is an inherited child copy.
            ValueError: The persistent path no longer denotes the pinned owned file.
            OSError: The descriptor or path is unavailable.
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
        """Publish bounded discovery metadata without granting PID signaling authority.

        Args:
            control: Cooperative endpoint identity and finalization diagnostics.

        Raises:
            ValueError: The metadata is oversized or ownership identity changed.
            RuntimeError: This process no longer owns the lease.
            OSError: The metadata cannot be written completely.
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
        """Close ownership without deleting the persistent lock inode.

        Repeated calls are harmless. A fork child only closes its inherited
        descriptor; LOCK_UN there would also unlock the parent's shared lease.
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
        """Acquire ownership for the context.

        Returns:
            This acquired lock.
        """
        self.acquire()
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        """Release ownership even when the run raises an exception.

        Args:
            exc_type: Exception class, if any.
            exc_value: Exception instance, if any.
            traceback: Exception traceback, if any.
        """
        self.release()
