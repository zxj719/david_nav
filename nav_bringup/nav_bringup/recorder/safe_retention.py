"""Descriptor-relative inspection and removal of completed run artifacts.

The selected root's ancestors are trusted. Descendant symlinks are never followed;
directory descriptors pin the authorized objects across path replacement. This
is fail-closed, incremental deletion, not a transaction or protection against an
administrator changing mounts. Manifests and summaries are never removed.
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
import datetime
import os
from pathlib import Path
import stat
from typing import Dict, Iterator, Tuple

import yaml

from nav_bringup.recorder.run_dir import RunDirectory, RUN_ID_RE

_FLAGS = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC
_KEPT = ('manifest.yaml', 'summary.md')


@dataclass(frozen=True)
class Snapshot:
    """Read-only inventory of one pinned run.

    Attributes:
        identity: Device and inode of the record directory.
        entries: Relative component tuples mapped to no-follow file metadata.
        complete: A matching manifest has an end timestamp and an unaliased summary.
    """

    identity: Tuple[int, int]
    entries: Dict[Tuple[str, ...], os.stat_result]
    complete: bool

    @property
    def size(self) -> int:
        """int: Logical regular-file bytes, not allocated disk blocks."""
        return sum(info.st_size for info in self.entries.values() if stat.S_ISREG(info.st_mode))

    @property
    def pruned(self) -> bool:
        """bool: Only permanently retained files remain."""
        return all(path[0] in _KEPT for path in self.entries)

    @property
    def has_bag(self) -> bool:
        """bool: A real bag directory contains at least one entry."""
        return any(len(path) > 1 and path[0] == 'bag' for path in self.entries)


@contextmanager
def open_root(root: Path) -> Iterator[int]:
    """Pin an owned run root without creating it or following its final symlink.

    Args:
        root: Trusted, explicit runs directory.

    Yields:
        Read-only directory descriptor; callers must not close it themselves.

    Raises:
        ValueError: The root is not owned by the current user or is a broad root.
        OSError: The directory cannot be opened safely.
    """
    path = root.expanduser().absolute()
    if path in (Path('/'), Path.home()) or '..' in path.parts:
        raise ValueError('refusing a broad or unnormalized retention root')
    descriptor = os.open(path, _FLAGS)
    try:
        if os.fstat(descriptor).st_uid != os.geteuid():
            raise ValueError('retention root must be owned by this user')
        yield descriptor
    finally:
        os.close(descriptor)


def _identity(info: os.stat_result) -> Tuple[int, int]:
    return info.st_dev, info.st_ino


def _complete(descriptor: int, name: str, entries: Dict[tuple, os.stat_result]) -> bool:
    for filename in _KEPT:
        info = entries.get((filename,))
        if info is None or not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            return False
    info = entries[('manifest.yaml',)]
    if info.st_size > 1048576:
        return False
    fd = os.open('manifest.yaml', os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK
                 | os.O_CLOEXEC, dir_fd=descriptor)
    try:
        if _identity(os.fstat(fd)) != _identity(info):
            return False
        with os.fdopen(os.dup(fd), 'rb') as stream:
            payload = stream.read(1048577)
        if len(payload) > 1048576:
            return False
        data = yaml.safe_load(payload)
        if (not isinstance(data, dict) or type(data.get('schema_version')) is not int
                or data['schema_version'] != 1 or data.get('run_id') != name):
            return False
        ended = data.get('ended_at')
        if isinstance(ended, str):
            ended = datetime.datetime.fromisoformat(ended)
        return isinstance(ended, datetime.datetime)
    except (ValueError, TypeError, yaml.YAMLError, RecursionError):
        return False
    finally:
        os.close(fd)


def inspect_at(root_fd: int, name: str) -> Snapshot:
    """Inventory one direct record directory without opening payload file contents.

    Args:
        root_fd: Pinned runs root.
        name: Exact run ID below the root.

    Returns:
        A bounded inventory with conservative completion evidence.

    Raises:
        ValueError: Identity, ownership, traversal limits or file types are unsafe.
        OSError: Files change or are inaccessible during inspection.
    """
    if RUN_ID_RE.fullmatch(name) is None:
        raise ValueError('invalid run ID')
    descriptor = os.open(name, _FLAGS, dir_fd=root_fd)
    try:
        info = os.fstat(descriptor)
        if info.st_uid != os.geteuid() or info.st_dev != os.fstat(root_fd).st_dev:
            raise ValueError('record ownership or filesystem mismatch')
        entries: Dict[Tuple[str, ...], os.stat_result] = {}

        def walk(parent: int, prefix: tuple) -> None:
            if len(prefix) > 32:
                raise ValueError('retention depth limit exceeded')
            with os.scandir(parent) as children:
                for child in children:
                    if len(entries) >= 200000:
                        raise ValueError('retention entry limit exceeded')
                    item = child.stat(follow_symlinks=False)
                    if item.st_dev != info.st_dev or not (
                            stat.S_ISDIR(item.st_mode) or stat.S_ISREG(item.st_mode)
                            or stat.S_ISLNK(item.st_mode)):
                        raise ValueError('unsupported retention entry')
                    relative = prefix + (child.name,)
                    entries[relative] = item
                    if stat.S_ISDIR(item.st_mode):
                        nested = os.open(child.name, _FLAGS, dir_fd=parent)
                        try:
                            if _identity(os.fstat(nested)) != _identity(item):
                                raise ValueError('record directory changed during inspection')
                            walk(nested, relative)
                        finally:
                            os.close(nested)

        walk(descriptor, ())
        complete = _complete(descriptor, name, entries)
        return Snapshot(_identity(info), entries, complete)
    finally:
        os.close(descriptor)


def inspect_run(run: RunDirectory) -> Snapshot:
    """Inspect a record using its explicitly selected root.

    Args:
        run: Record path pair.

    Returns:
        No-follow bounded inventory.
    """
    with open_root(run.root) as descriptor:
        return inspect_at(descriptor, run.run_id)


def remove_at(root_fd: int, name: str, snapshot: Snapshot, bag_only: bool) -> int:
    """Remove only inventoried payload entries from the same completed record.

    Args:
        root_fd: Pinned root descriptor.
        name: Exact run ID.
        snapshot: Inventory revalidated immediately before deletion.
        bag_only: Restrict deletion to the actual bag directory and descendants.

    Returns:
        Removed logical regular-file bytes; hard links and sparse allocation can
        make this differ from physical disk space reclaimed.

    Raises:
        ValueError: Directory or file identity changes, or completion is unproven.
        OSError: A deletion fails, possibly after earlier entries were removed.
    """
    if not snapshot.complete:
        raise ValueError('record completion is unproven')
    run_fd = os.open(name, _FLAGS, dir_fd=root_fd)
    try:
        if _identity(os.fstat(run_fd)) != snapshot.identity:
            raise ValueError('record incarnation changed before deletion')
        targets = [path for path in snapshot.entries if path[0] not in _KEPT
                   and (not bag_only or path[0] == 'bag')]
        bag = snapshot.entries.get(('bag',))
        if bag_only and (bag is None or not stat.S_ISDIR(bag.st_mode)):
            raise ValueError('bag target is not a real directory')
        if not _complete(run_fd, name, snapshot.entries):
            raise ValueError('record completion changed before deletion')
        removed = 0
        for path in sorted(targets, key=lambda item: (-len(item), item)):
            parent = os.dup(run_fd)
            try:
                for depth, part in enumerate(path[:-1], 1):
                    nested = os.open(part, _FLAGS, dir_fd=parent)
                    os.close(parent)
                    parent = nested
                    if _identity(os.fstat(parent)) != _identity(snapshot.entries[path[:depth]]):
                        raise ValueError('payload directory incarnation changed')
                expected = snapshot.entries[path]
                current = os.stat(path[-1], dir_fd=parent, follow_symlinks=False)
                if (_identity(current) != _identity(expected)
                        or stat.S_IFMT(current.st_mode) != stat.S_IFMT(expected.st_mode)):
                    raise ValueError('payload entry changed before deletion')
                if stat.S_ISDIR(current.st_mode):
                    os.rmdir(path[-1], dir_fd=parent)
                else:
                    if (current.st_size, current.st_mtime_ns) != (
                            expected.st_size, expected.st_mtime_ns):
                        raise ValueError('payload was modified before deletion')
                    os.unlink(path[-1], dir_fd=parent)
                    if stat.S_ISREG(current.st_mode):
                        removed += current.st_size
            finally:
                os.close(parent)
        return removed
    finally:
        os.close(run_fd)
