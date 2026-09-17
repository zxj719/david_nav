"""Bounded, descriptor-relative reads of run records, without following record links.

The caller selects the runs root; its ancestors are trusted. All descendants are
opened relative to pinned directory descriptors with O_NOFOLLOW. No record is
created, locked, repaired or treated as proof that a supervisor is still alive.
Concurrent writers can produce incomplete data: this is not an atomic snapshot.
"""

from __future__ import annotations

from contextlib import contextmanager
import os
from pathlib import Path
import stat
from typing import Dict, Iterator, Optional

from nav_bringup.recorder.run_dir import RUN_ID_RE

MAX_FILE_BYTES = 8 * 1024 * 1024
MAX_TOTAL_BYTES = 32 * 1024 * 1024
MAX_ENTRIES = 4096
_DIRECTORY_FLAGS = os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW


class RecordReader:
    """Read a single pinned run directory with shared byte and traversal budgets.

    Attributes:
        run_id: 正在读取的固定运行目录标识，不表示该运行仍存活。
    """

    def __init__(self, descriptor: int, run_id: str) -> None:
        """Borrow a directory descriptor for the lifetime of an open_record context."""
        self._descriptor = descriptor
        self.run_id = run_id
        self._remaining = MAX_TOTAL_BYTES
        self._entries = MAX_ENTRIES

    @contextmanager
    def _directory(self, parts: tuple) -> Iterator[int]:
        """相对于固定记录目录逐级打开子目录，并在退出上下文时关闭描述符.

        只复制初始描述符，不关闭借入的根描述符；逐级 O_NOFOLLOW 拒绝链接。
        路径组件应由调用方先校验，本函数不自行拒绝 .. 等输入。

        Args:
            parts: 调用方已校验的相对目录组件元组；空元组表示记录根。

        Yields:
            当前子目录的借用描述符，仅在 with 上下文内有效。

        Raises:
            OSError: 描述符复制、目录安全打开或关闭失败。
        """
        descriptor = os.dup(self._descriptor)
        try:
            for part in parts:
                child = os.open(part, _DIRECTORY_FLAGS, dir_fd=descriptor)
                os.close(descriptor)
                descriptor = child
            yield descriptor
        finally:
            os.close(descriptor)

    def read(self, relative: str) -> Optional[str]:
        """Read UTF-8 from one regular, unaliased file; missing files return None.

        Args:
            relative: Relative path under the pinned run directory.

        Returns:
            Exact file text, or None when this artifact has not been recorded.

        Raises:
            ValueError: A path, file type, size or byte budget is invalid.
            OSError: A component is inaccessible or cannot be opened safely.
            UnicodeError: The content is not UTF-8.
        """
        parts = relative.split('/')
        if any(part in ('', '.', '..') for part in parts):
            raise ValueError('invalid record artifact path')
        try:
            with self._directory(tuple(parts[:-1])) as parent:
                descriptor = os.open(parts[-1], os.O_RDONLY | os.O_NOFOLLOW
                                     | os.O_NONBLOCK | os.O_CLOEXEC, dir_fd=parent)
        except FileNotFoundError:
            return None
        try:
            info = os.fstat(descriptor)
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                raise ValueError('artifact must be an unaliased regular file')
            limit = min(MAX_FILE_BYTES, self._remaining)
            if info.st_size > limit:
                raise ValueError('record inspection byte budget exceeded')
            chunks = []
            count = 0
            while count <= limit:
                chunk = os.read(descriptor, min(65536, limit + 1 - count))
                if not chunk:
                    break
                chunks.append(chunk)
                count += len(chunk)
            if count > limit:
                raise ValueError('record inspection byte budget exceeded')
            self._remaining -= count
            return b''.join(chunks).decode('utf-8')
        finally:
            os.close(descriptor)

    def tree(self, relative: str) -> Dict[str, str]:
        """Export saved artifact text, without following links or special files.

        Args:
            relative: Artifact directory, such as params/effective.

        Returns:
            Paths relative to that artifact directory mapped to exact contents.

        Raises:
            ValueError: Traversal depth, entry count or byte budget is exceeded.
            OSError: A record component is unsafe or inaccessible.
        """
        result: Dict[str, str] = {}

        def visit(prefix: str, depth: int) -> None:
            """按固定目录描述符遍历文本制品，消耗共享项数与读取字节预算.

            不跟随目录链接；非目录仍交由 read 检查普通文件与单硬链接条件。
            起始目录缺失可返回空结果，子目录消失则报错；并发写入不是原子快照。
            失败前已消耗的预算不回滚，也不据记录存在推断运行进程存活。

            Args:
                prefix: 记录根下已校验的相对目录路径。
                depth: 从所选制品目录起计的深度，超过八层拒绝。

            Raises:
                ValueError: 深度/项数/字节预算超限，文件不安全或遍历中消失。
                OSError: 子目录打开或文件读取失败。
                UnicodeError: 制品不是有效 UTF-8。
            """
            if depth > 8:
                raise ValueError('record directory depth exceeded')
            try:
                with self._directory(tuple(prefix.split('/'))) as descriptor:
                    with os.scandir(descriptor) as entries:
                        for entry in entries:
                            self._entries -= 1
                            if self._entries < 0:
                                raise ValueError('record entry budget exceeded')
                            path = prefix + '/' + entry.name
                            if entry.is_dir(follow_symlinks=False):
                                visit(path, depth + 1)
                            else:
                                contents = self.read(path)
                                if contents is None:
                                    raise ValueError('artifact disappeared during inspection')
                                result[path[len(relative) + 1:]] = contents
            except FileNotFoundError:
                if depth != 0:
                    raise

        if any(part in ('', '.', '..') for part in relative.split('/')):
            raise ValueError('invalid record artifact directory')
        visit(relative, 0)
        return dict(sorted(result.items()))


@contextmanager
def open_record(root: Path, selector: str) -> Iterator[RecordReader]:
    """Pin an existing record selected by an ID or a direct relative latest link.

    Args:
        root: Explicit, trusted runs root; the root itself must not be a symlink.
        selector: Exact run ID or latest.

    Yields:
        A reader whose descriptors remain open for the context lifetime.

    Raises:
        ValueError: The selector or latest link is not an exact run ID.
        OSError: The root or record cannot be opened without following links.
    """
    parent = os.open(root.expanduser(), _DIRECTORY_FLAGS)
    try:
        run_id = os.readlink('latest', dir_fd=parent) if selector == 'latest' else selector
        if RUN_ID_RE.fullmatch(run_id) is None:
            raise ValueError('expected an exact run ID or direct latest link')
        descriptor = os.open(run_id, _DIRECTORY_FLAGS, dir_fd=parent)
        try:
            yield RecordReader(descriptor, run_id)
        finally:
            os.close(descriptor)
    finally:
        os.close(parent)


def record_ids(root: Path) -> list[str]:
    """List real record directories newest first, without creating a missing root.

    Args:
        root: Selected runs root.

    Returns:
        Existing run IDs; non-record entries and symlink directories are ignored.

    Raises:
        ValueError: The directory exceeds the entry budget.
        OSError: An existing root cannot be read safely.
    """
    try:
        descriptor = os.open(root.expanduser(), _DIRECTORY_FLAGS)
    except FileNotFoundError:
        return []
    try:
        result = []
        with os.scandir(descriptor) as entries:
            for count, entry in enumerate(entries):
                if count >= MAX_ENTRIES:
                    raise ValueError('run listing entry budget exceeded')
                if (RUN_ID_RE.fullmatch(entry.name) is not None
                        and entry.is_dir(follow_symlinks=False)):
                    result.append(entry.name)
        return sorted(result, reverse=True)
    finally:
        os.close(descriptor)
