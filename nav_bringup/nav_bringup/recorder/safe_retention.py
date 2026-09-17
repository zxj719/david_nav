"""以固定目录描述符检查和逐项移除已完成运行的制品.

信任所选根目录的祖先；不跟随后代符号链接，以目录描述符与身份复核限制
路径替换风险。不提供对管理员改挂载的防护，也不是可回滚事务。
失败可能已删除前面的项目；永久保留 manifest.yaml 与 summary.md。
完成标记不代替进程存活检查，调用方仍须持有维护/启动租约并验证可清理性。
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
    """表示固定运行目录的一次元数据盘点，不保证整树原子或深度不可变.

    冻结只限制字段重赋值，entries 字典仍可变；调用方不应修改它。
    派生属性读取当前字典，complete 是盘点时的判断，不证明进程已退出。

    Attributes:
        identity: 记录目录的设备号与 inode。
        entries: 相对组件元组到不跟随链接的文件元数据映射。
        complete: 匹配的清单有可解析结束时间，且摘要为单硬链接普通文件。
    """

    identity: Tuple[int, int]
    entries: Dict[Tuple[str, ...], os.stat_result]
    complete: bool

    @property
    def size(self) -> int:
        """int: 当前盘点中普通文件的逻辑字节和，不是实际磁盘占用或可回收量."""
        return sum(info.st_size for info in self.entries.values() if stat.S_ISREG(info.st_mode))

    @property
    def pruned(self) -> bool:
        """bool: 所有盘点路径均位于永久保留项下；空盘点也为 True，不证明完成."""
        return all(path[0] in _KEPT for path in self.entries)

    @property
    def has_bag(self) -> bool:
        """bool: 盘点中是否有 bag 下的后代条目，不保证存在有效或非空录制数据."""
        return any(len(path) > 1 and path[0] == 'bag' for path in self.entries)


@contextmanager
def open_root(root: Path) -> Iterator[int]:
    """打开并固定当前用户拥有的记录根，不创建目录或跟随末级链接.

    只拒绝根目录、当前用户家目录及含父目录组件的路径，不推断整个业务授权范围。
    根目录祖先受信任；调用方仍须明确选择用途正确的运行记录目录。

    Args:
        root: 显式选定的记录目录。

    Yields:
        只在上下文中有效的只读目录描述符，调用方不得自行关闭。

    Raises:
        ValueError: 根过宽、含父目录组件或不属于当前用户。
        OSError: 无法安全打开目录或关闭描述符。
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
    """通过已打开目录验证记录有完成标记，不能替代在线进程存活检查.

    Args:
        descriptor: 调用方持有的运行目录描述符。
        name: 预期 run_id。
        entries: 已扫描文件身份，供不跟随链接的读取复核。

    Returns:
        保留文件为单硬链接普通文件、清单身份/schema 合法且 ended_at 可解析
        为日期时间时为 True；内容异常或身份不符为 False。

    Raises:
        OSError: 相对目录打开或读取失败，由上层保守跳过清理。
    """
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
    """盘点直接记录目录的元数据，并限量读取清单以核对完成标记.

    不读取录制/日志负载；链接本身可列入盘点，但不跟随。
    扫描受设备、目录身份、深度及二十万条目限制，不取得整树原子快照。

    Args:
        root_fd: 外层固定并持有的运行根目录描述符。
        name: 根下的精确运行 ID。

    Returns:
        有界盘点及保守完成标记；标记不证明进程存活状态或运行成功。

    Raises:
        ValueError: 身份、所有者、文件系统、遍历预算或文件类型不安全。
        OSError: 扫描、清单读取或描述符操作失败。
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
            """扫描同设备目录元数据并复核子目录身份，不读取负载或执行删除.

            允许目录、普通文件和链接本身，但不跟随链接。打开子目录后比对
            设备/inode 防止路径替换；这不是整棵目录的原子快照或挂载防护。
            先登记条目再递归，失败前的局部字典可能已有数据。

            Args:
                parent: 已固定父目录的借用描述符，不在此关闭。
                prefix: 相对组件元组，超过三十二层拒绝继续扫描。

            Raises:
                ValueError: 深度/二十万条目预算、文件类型、设备或目录身份不符。
                OSError: 不跟随链接的元数据读取、目录打开或关闭失败。
            """
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
    """按记录显式选择的根目录执行只读盘点，不删除任何内容.

    Args:
        run: 含根目录与运行 ID 的记录路径对象。

    Returns:
        不跟随链接且有遍历预算的元数据盘点。

    Raises:
        ValueError: 根目录、身份、所有者、文件系统、条目或预算不符合要求。
        OSError: 无法安全打开、扫描或关闭相关目录/清单。
    """
    with open_root(run.root) as descriptor:
        return inspect_at(descriptor, run.run_id)


def remove_at(root_fd: int, name: str, snapshot: Snapshot, bag_only: bool) -> int:
    """仅删除同一已完成记录中盘点过的负载，永久保留清单及摘要.

    调用方须提供可信、未篡改且刚重新盘点的 Snapshot，并持有外层维护租约。
    按由深到浅的顺序复核身份/类型，非目录还复核大小和修改时间再删除；
    检查与删除仍非原子，不抵御管理员改挂载，失败不撤销已完成删除。
    只删除链接本身，不跟随其目标；不删除运行目录本身。

    Args:
        root_fd: 外层固定并持有的根目录描述符。
        name: 已盘点记录的精确运行 ID。
        snapshot: 删除前重新核对的可信盘点，不应手工伪造或修改 entries。
        bag_only: 是否只处理真实 bag 目录及其后代。

    Returns:
        已删普通文件的逻辑字节和；硬链接或稀疏文件可能使其不同于实际回收空间。

    Raises:
        ValueError: 完成证据不足、bag 不是目录或目录/文件身份发生变化。
        OSError: 打开、复核或删除失败，此前项目可能已经删除。
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
