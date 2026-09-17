"""使用固定目录描述符及共享预算读取运行记录，不跟随记录内链接.

调用方选择并信任根目录的祖先；后代逐级以 O_NOFOLLOW 打开。
不创建、锁定或修复记录，也不把记录存在当作监督者仍存活的证据。
并发写入可能得到不完整或不同时刻的数据，这不是原子快照。
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
    """表示借用固定运行目录描述符、共享字节及遍历预算的读取器.

    只能在持有描述符的上下文内使用，不自行拥有或关闭借入的根描述符。
    预算随读取/遍历消耗，失败时不统一回滚，也不提供并发访问同步。

    Attributes:
        run_id: 正在读取的固定运行目录标识，不表示该运行仍存活。
    """

    def __init__(self, descriptor: int, run_id: str) -> None:
        """借用运行目录描述符，并为本读取器初始化累计预算.

        Args:
            descriptor: open_record 上下文持有的目录描述符，不在此复制或验证。
            run_id: 由外层校验的运行标识。
        """
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
        """读取单硬链接普通文件的 UTF-8 文本，缺失路径返回 None.

        逐级拒绝链接和特殊文件；文件大小与实际读取字节均受单文件及共享预算限制。
        重复读取重复扣额，UTF-8 解码失败也已扣除读取字节；缺失文件不扣字节。
        不锁定文件内容，读取结果不是相对于并发写者的原子快照。

        Args:
            relative: 固定运行目录下的相对路径，不得含空组件、点或父目录组件。

        Returns:
            原始文本，文件或其父目录不存在时为 None。

        Raises:
            ValueError: 路径、文件类型、硬链接数、大小或字节预算不符。
            OSError: 路径不可访问、不能安全打开或读取/关闭失败。
            UnicodeError: 内容不是有效 UTF-8。
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
        """有界导出制品目录文本，拒绝链接和特殊文件，不重建配置.

        共享项数与字节预算跨本读取器保留，最大递归深度为八；失败不退还预算。
        起始目录缺失返回空字典，遍历中子目录或文件消失则报错，不保证原子快照。

        Args:
            relative: 运行目录内的相对制品目录，例如 params/effective。

        Returns:
            相对此制品目录的文件路径到原始文本的映射，按路径排序。

        Raises:
            ValueError: 路径/文件不安全、深度/项数/字节超限或文件遍历中消失。
            OSError: 子目录或记录无法安全读取。
            UnicodeError: 制品内容不是有效 UTF-8。
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
    """固定运行目录，接受精确 ID 或直接指向 ID 的相对 latest 链接.

    根目录本身及记录路径不跟随链接，但信任根目录祖先。
    latest 只解析一次，之后依靠已打开描述符，不以路径持续指向同一项为前提。

    Args:
        root: 调用方明确选择并信任的记录根目录，本身不能是符号链接。
        selector: 精确运行 ID 或 latest。

    Yields:
        只在 with 上下文生命期内有效的读取器。

    Raises:
        ValueError: 选择器或 latest 目标不是精确运行 ID。
        OSError: 根目录、链接或记录不能安全读取/打开，或描述符关闭失败。
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
    """只读列出真实记录目录，按标识逆序返回，不创建缺失根目录.

    按名称排序而非读取实际启动时间；所有扫描项都计入条目预算，包括无关文件。

    Args:
        root: 明确选择的记录根目录，信任其祖先但不跟随根目录链接。

    Returns:
        现有运行 ID 列表，忽略非记录名称和链接目录；根目录缺失时为空。

    Raises:
        ValueError: 根目录条目数超过预算。
        OSError: 现有根目录无法安全打开、扫描或关闭。
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
