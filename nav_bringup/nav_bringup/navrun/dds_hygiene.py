"""Conservative Fast DDS orphan-file hygiene, separate from launch and ROS discovery.

No daemon is stopped and no transport setting is changed. Mapping observations
are advisory snapshots, not an atomic lock against external DDS participants;
callers must avoid concurrent participant startup during a cleanup window.
Tests use only private directories. The cleanup directory and observer are
mandatory arguments so importing or calling a default CLI cannot clean /dev/shm.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import re
import stat
from typing import Callable, Dict, FrozenSet, Tuple


_NAME = re.compile(r'^fastrtps_[0-9A-Za-z_]+$')
_MAPPED = re.compile(r'fastrtps_[0-9A-Za-z_]+')


@dataclass(frozen=True)
class MappingView:
    """One read-only observation of the process mapping set.

    Attributes:
        names: Shared-memory names mentioned by any readable process maps file.
        complete: Whether all observed process directories were readable and stable.
        issues: Reasons why the observation cannot establish absence of mappings.
    """

    names: FrozenSet[str]
    complete: bool
    issues: Tuple[str, ...] = ()


@dataclass(frozen=True)
class _FileState:
    name: str
    device: int
    inode: int
    uid: int
    mode: int
    links: int
    size: int
    mtime_ns: int
    ctime_ns: int


@dataclass(frozen=True)
class HygienePlan:
    """An immutable initial generation of candidate DDS orphan files.

    Attributes:
        directory: Explicit absolute directory inspected by the caller.
        candidates: Owned regular files eligible under the supplied mapping observation.
        retained: Name/reason pairs for files conservatively retained.
        owner_uid: User whose files alone may be removed.
    """

    directory: Path
    candidates: Tuple[_FileState, ...]
    retained: Tuple[Tuple[str, str], ...]
    owner_uid: int
    _entries: Tuple[_FileState, ...]
    _directory_identity: Tuple[int, int]


@dataclass(frozen=True)
class HygieneResult:
    """Auditable outcome without claiming removed files can be recovered.

    Attributes:
        planned: Initial candidate names, also returned unchanged for dry-run.
        removed: Names successfully unlinked by this operation.
        retained: Name/reason pairs for retained files.
        errors: Name/error pairs for failed observations or unlink attempts.
    """

    planned: Tuple[str, ...]
    removed: Tuple[str, ...]
    retained: Tuple[Tuple[str, str], ...]
    errors: Tuple[Tuple[str, str], ...]


def _pid_entries(proc_dir: Path) -> Dict[str, Tuple[int, int]]:
    """读取数字命名进程目录的设备号与 inode，供前后快照比对.

    使用 lstat 拒绝冒充进程目录的符号链接等非目录项；单次枚举不是原子快照，
    也不证明进程映射可读，完整性仍由外层读取和复查判断。

    Args:
        proc_dir: 待枚举的进程文件系统根目录，可为测试用私有目录。

    Returns:
        数字目录名到设备号、inode 的映射。

    Raises:
        OSError: 枚举或读取失败，或数字名称对应项不是实际目录。
    """
    result = {}
    for path in proc_dir.iterdir():
        if not path.name.isdecimal():
            continue
        info = path.lstat()
        if not stat.S_ISDIR(info.st_mode):
            raise OSError(f'non-directory process entry {path.name}')
        result[path.name] = (info.st_dev, info.st_ino)
    return result


def snapshot_mappings(proc_dir: Path = Path('/proc')) -> MappingView:
    """Read process maps without opening segments, invoking ROS or elevating privileges.

    Args:
        proc_dir: Process filesystem root; tests provide a private fake directory.

    Returns:
        Mapping names and a completeness flag. Unreadable maps, disappearing or
        newly observed process directories make absence evidence incomplete.
    """
    names, issues = set(), []
    try:
        before = _pid_entries(proc_dir)
    except OSError as error:
        return MappingView(frozenset(), False, (f'cannot list processes: {error}',))
    for pid in sorted(before):
        try:
            text = (proc_dir / pid / 'maps').read_text(encoding='utf-8', errors='replace')
            names.update(_MAPPED.findall(text))
        except OSError as error:
            issues.append(f'pid {pid}: cannot read maps: {error}')
    try:
        if _pid_entries(proc_dir) != before:
            issues.append('process directory identities changed during mapping observation')
    except OSError as error:
        issues.append(f'cannot recheck process set: {error}')
    return MappingView(frozenset(names), not issues, tuple(issues))


def _group(name: str) -> str:
    """去掉可选 _el 后缀以归并同一命名组，不作清理资格判断.

    Args:
        name: 共享内存候选文件名称。

    Returns:
        按既有后缀约定归并的组名；不是已验证的进程或映射身份。
    """
    return name[:-3] if name.endswith('_el') else name


def _state(path: Path) -> _FileState:
    """用 lstat 记录清理候选的文件身份和元数据，不跟随末级链接.

    只生成本次观察值，不证明文件可删除；类型、归属、映射与前后稳定性
    仍由规划和执行流程分别检查。

    Args:
        path: 待观察的单个文件路径。

    Returns:
        名称、设备/inode、用户/模式/链接数、大小与纳秒时间组成的快照。

    Raises:
        OSError: 文件已消失或元数据读取失败。
    """
    info = path.lstat()
    return _FileState(path.name, info.st_dev, info.st_ino, info.st_uid, info.st_mode,
                      info.st_nlink, info.st_size, info.st_mtime_ns, info.st_ctime_ns)


def _directory(directory: Path) -> Tuple[int, int]:
    """检查清理目录的显式路径与末级类型，返回待复核的目录身份.

    只做检查，不删除文件；不把根目录、用户主目录或含 .. 的路径作为目标。
    lstat 只检查末级不是符号链接，本函数不承诺逐级固定祖先目录。

    Args:
        directory: 调用方明确指定的绝对共享内存目录路径。

    Returns:
        本次读取的设备号与 inode，用于后续比对。

    Raises:
        ValueError: 路径不明确、过宽，或末级不是实际目录。
        OSError: 无法读取目录元数据。
    """
    if (not directory.is_absolute() or '..' in directory.parts
            or directory in (Path('/'), Path.home())):
        raise ValueError('hygiene requires an explicit normalized shared-memory directory')
    info = directory.lstat()
    if not stat.S_ISDIR(info.st_mode):
        raise ValueError('hygiene requires a real directory, not a symlink')
    return info.st_dev, info.st_ino


def plan_hygiene(
    directory: Path, view: MappingView, owner_uid: int | None = None,
) -> HygienePlan:
    """Inspect only Fast DDS names and preserve mapped, unsafe or unproven groups.

    Args:
        directory: Explicit absolute directory; no implicit deletion root exists.
        view: Mapping evidence; complete must be a real boolean.
        owner_uid: Allowed file owner, defaulting to the current real user id.

    Returns:
        An initial immutable file generation and retention reasons, without writes.

    Raises:
        ValueError: The directory, mapping evidence or owner id is unsafe or malformed.
        OSError: The directory or an entry cannot be inspected reliably.
    """
    if (type(view.complete) is not bool or not isinstance(view.names, frozenset)
            or any(not isinstance(name, str) for name in view.names)):
        raise ValueError('mapping evidence must contain frozen names and a boolean completeness')
    owner_uid = os.getuid() if owner_uid is None else owner_uid
    if type(owner_uid) is not int or owner_uid < 0:
        raise ValueError('owner_uid must be a nonnegative integer')
    identity = _directory(directory)
    entries = tuple(_state(path) for path in sorted(directory.iterdir())
                    if _NAME.fullmatch(path.name))
    mapped = {_group(name) for name in view.names}
    unsafe = {_group(entry.name) for entry in entries if (
        not stat.S_ISREG(entry.mode) or entry.uid != owner_uid or entry.links != 1)}
    candidates, retained = [], []
    for entry in entries:
        group = _group(entry.name)
        if not view.complete:
            retained.append((entry.name, 'incomplete process mapping observation'))
        elif group in mapped:
            retained.append((entry.name, 'segment or companion is mapped'))
        elif group in unsafe:
            retained.append((entry.name, 'unsafe file identity or other owner in group'))
        else:
            candidates.append(entry)
    if _directory(directory) != identity:
        raise ValueError('directory identity changed during planning')
    return HygienePlan(directory, tuple(candidates), tuple(retained), owner_uid, entries, identity)


def _recheck(
    original: HygienePlan, latest: HygienePlan, entry: _FileState, removed: set[str],
) -> str:
    """复核共享内存候选的目录身份、文件代次和同组伴随文件，不执行删除.

    Args:
        original: 初始只读清理计划。
        latest: 紧邻操作前重新观测的计划。
        entry: 当前待复核文件状态。
        removed: 本操作中已成功移除的文件名集合。

    Returns:
        不可继续的保留原因；空字符串只表示本次复核未发现保留理由。

    Raises:
        ValueError: 共享内存目录身份已变化，拒绝继续。
    """
    if latest._directory_identity != original._directory_identity:
        raise ValueError('shared-memory directory identity changed')
    group = _group(entry.name)
    expected = {item.name: item for item in original._entries
                if _group(item.name) == group and item.name not in removed}
    current = {item.name: item for item in latest._entries if _group(item.name) == group}
    if current != expected:
        return 'file generation or companion set changed'
    retained = dict(latest.retained)
    return retained.get(entry.name, '')


def cleanup_hygiene(
    plan: HygienePlan, observe: Callable[[], MappingView], dry_run: bool = False,
) -> HygieneResult:
    """Remove only initial candidates that pass a fresh mapping and identity recheck.

    Each unlink is preceded by a new complete mapping observation and group
    comparison. These checks reduce races but are not an atomic DDS-wide barrier;
    external participants must not start concurrently with the cleanup window.
    A partially removed pair or failed unlink is reported, never silently undone.

    Args:
        plan: Initial read-only decision with its fixed file generation.
        observe: Required fresh mapping reader; tests inject controlled observations.
        dry_run: Return planned names without reading maps or removing anything.

    Returns:
        Planned, removed, retained and failed names for recording.

    Raises:
        ValueError: Invalid dry-run flag, unsafe evidence or a replaced directory.
    """
    if type(dry_run) is not bool:
        raise ValueError('dry_run must be a boolean')
    planned = tuple(entry.name for entry in plan.candidates)
    if dry_run:
        return HygieneResult(planned, (), plan.retained, ())
    removed, retained, errors, blocked = [], list(plan.retained), [], set()
    for entry in plan.candidates:
        group = _group(entry.name)
        if group in blocked:
            retained.append((entry.name, 'group retained after earlier recheck or unlink failure'))
            continue
        try:
            latest = plan_hygiene(plan.directory, observe(), plan.owner_uid)
            reason = _recheck(plan, latest, entry, set(removed))
            if reason:
                retained.append((entry.name, reason))
                blocked.add(group)
                continue
            if _directory(plan.directory) != plan._directory_identity:
                raise ValueError('shared-memory directory identity changed before unlink')
            path = plan.directory / entry.name
            if _state(path) != entry:
                retained.append((entry.name, 'file identity changed before unlink'))
                blocked.add(group)
                continue
            path.unlink()
            removed.append(entry.name)
        except OSError as error:
            errors.append((entry.name, str(error)))
            retained.append((entry.name, 'observation or unlink failed'))
            blocked.add(group)
    return HygieneResult(planned, tuple(removed), tuple(retained), tuple(errors))
