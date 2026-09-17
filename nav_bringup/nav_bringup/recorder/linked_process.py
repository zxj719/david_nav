"""只读观察独立启动的进程，并为导航运行截取其日志.

定位栈由现场独立管理，本模块只读取 procfs、配置及匹配日志，不发送进程信号。
snapshot 选最小匹配 PID，记录命令行、启动时间、配置哈希和日志路径的大小；
collect_logs 根据路径大小差与修改时间截取尾部，并不证明日志由该进程写入。

写入位置由调用方提供的 destination 决定；模块本身不验证它位于运行目录内。
路径查询可能跟随符号链接，快照与复制不具备原子性，不能作为安全隔离或完整审计证据。
"""

from __future__ import annotations

from dataclasses import dataclass
import datetime
import glob
import os
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from nav_bringup.recorder import manifest

DEFAULT_CONFIG_FLAGS = ('--config',)
DEFAULT_MAX_COPY_BYTES = 20 * 1024 * 1024

_COPY_CHUNK_BYTES = 1 << 20


@dataclass(frozen=True)
class LinkSpec:
    """描述要观察的独立进程及其日志匹配规则.

    Attributes:
        name: 清单条目及预期 logs 子目录标签，不含斜杠且不是点目录。
        executable: 与进程 argv[0] 的文件名比较的字符串，不解析包装器。
        config_flags: 从命令行提取配置路径的选项名。
        log_globs: 日志 glob 模式；展开 ~，并将 {pid} 替换为所选 PID。
        max_copy_bytes: 每个日志最多复制的字节数，超出时保留尾部。
        proc_dir: procfs 根路径，测试可使用临时目录。
    """

    name: str
    executable: str
    config_flags: Tuple[str, ...] = DEFAULT_CONFIG_FLAGS
    log_globs: Tuple[str, ...] = ()
    max_copy_bytes: int = DEFAULT_MAX_COPY_BYTES
    proc_dir: Path = Path('/proc')

    def __post_init__(self) -> None:
        """检查标签、可执行名和复制上限的基本条件.

        不检查目标目录归属、模式指向的文件类型或路径可信性。

        Raises:
            ValueError: 标签为空、含斜杠、为点目录，或可执行名为空、上限非正。
        """
        if not self.name or '/' in self.name or self.name in ('.', '..'):
            raise ValueError(f'invalid link name {self.name!r}')
        if not self.executable:
            raise ValueError('executable must not be empty')
        if self.max_copy_bytes <= 0:
            raise ValueError('max_copy_bytes must be positive')


@dataclass(frozen=True)
class LinkedProcess:
    """保存一次关联进程观察结果，不代表一致的系统快照.

    冻结只限制字段重新赋值，不深度冻结 configs 和 log_sizes。

    Attributes:
        pid: 所有匹配进程中最小的 PID，不代表最早启动。
        other_pids: 其余匹配 PID，按数值升序排列。
        started_at: 带本地时区的 ISO 8601 秒级启动时间，未知为 None。
        command: 从 procfs 读取的命令行参数。
        configs: 配置路径到 SHA-256 的映射；文件不可用时可为 None。
        log_sizes: 匹配日志路径到观察到的文件字节数的映射。
    """

    pid: int
    other_pids: Tuple[int, ...]
    started_at: Optional[str]
    command: Tuple[str, ...]
    configs: Mapping[str, Optional[str]]
    log_sizes: Mapping[str, int]

    def to_dict(self) -> Dict[str, Any]:
        """复制字段容器，生成适合写入运行清单的数据.

        Returns:
            列表和字典组成的映射；不重新读取进程或校验字段。
        """
        return {
            'pid': self.pid,
            'other_pids': list(self.other_pids),
            'started_at': self.started_at,
            'command': list(self.command),
            'configs': dict(self.configs),
            'log_sizes': dict(self.log_sizes),
        }


@dataclass(frozen=True)
class CopiedLog:
    """描述一次成功返回的日志摘录写入.

    Attributes:
        source: 来源日志路径。
        destination: 实际写入目标路径，由调用方目录派生。
        copied_bytes: 实际复制字节数，来源并发缩短时可能少于计划。
        truncated: 是否因每文件字节上限丢弃开头，不表示完整性验证结果。
    """

    source: str
    destination: str
    copied_bytes: int
    truncated: bool


def find_processes(proc_dir: Path, executable: str) -> List[Tuple[int, List[str]]]:
    """按 procfs 命令行首参数的文件名查找进程.

    读取失败的单个 cmdline 被跳过；不核验 PID 是否在查询期间复用，
    也不解开 ros2 run 或 Python 等包装器。目录遍历异常仍可传播。

    Args:
        proc_dir: procfs 根目录；不存在或不是目录时返回空列表。
        executable: 与 argv[0] 的 basename 精确比较的名称。

    Returns:
        按 PID 升序排列的 (pid, 参数列表)，解码错误字节替换为占位字符。
    """
    found: List[Tuple[int, List[str]]] = []
    if not proc_dir.is_dir():
        return found
    for entry in proc_dir.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / 'cmdline').read_bytes()
        except OSError:
            continue
        arguments = [part.decode(errors='replace') for part in raw.split(b'\0') if part]
        if arguments and os.path.basename(arguments[0]) == executable:
            found.append((int(entry.name), arguments))
    return sorted(found)


def process_start_time(
    proc_dir: Path, pid: int, clock_ticks: Optional[int] = None,
) -> Optional[datetime.datetime]:
    """从系统启动时间和进程时钟滴答估算本地启动时刻.

    Args:
        proc_dir: procfs 根目录。
        pid: 要读取 stat 的进程号。
        clock_ticks: 每秒滴答数；None 或零时读取 SC_CLK_TCK，不校验负数。

    Returns:
        无时区的本地 datetime；文件读取或已处理的字段解析失败时为 None。
        系统时钟配置和时间转换异常不统一转为 None。
    """
    try:
        stat = (proc_dir / str(pid) / 'stat').read_text(encoding='utf-8', errors='replace')
        system = (proc_dir / 'stat').read_text(encoding='utf-8', errors='replace')
    except OSError:
        return None
    try:
        start_ticks = int(stat[stat.rindex(')') + 2:].split()[19])
        boot_time = next(
            int(line.split()[1]) for line in system.splitlines() if line.startswith('btime '))
    except (IndexError, StopIteration, ValueError):
        return None
    ticks = clock_ticks or os.sysconf('SC_CLK_TCK')
    return datetime.datetime.fromtimestamp(boot_time + start_ticks / ticks)


def config_paths(command: Sequence[str], flags: Sequence[str]) -> List[str]:
    """按给定选项提取配置路径，不校验路径或完整命令行语义.

    Args:
        command: 按原顺序提供的参数。
        flags: 接受 --flag value 与 --flag=value 两种形式的选项名。

    Returns:
        按参数和选项遍历顺序收集的值，保留重复、空值和类似选项的值；
        位于末尾且没有后继参数的独立选项被忽略。
    """
    paths = []
    for index, argument in enumerate(command):
        for flag in flags:
            if argument == flag and index + 1 < len(command):
                paths.append(command[index + 1])
            elif argument.startswith(flag + '='):
                paths.append(argument[len(flag) + 1:])
    return paths


def matching_logs(log_globs: Sequence[str], pid: Optional[int]) -> List[Path]:
    """展开日志模式并收集当前被识别为文件的路径.

    Args:
        log_globs: 展开用户目录和 {pid} 的 glob 模式，不启用递归 glob。
        pid: 用于替换的 PID；None 时跳过含 {pid} 的模式。

    Returns:
        按路径排序且去除相同路径的列表。isfile 可跟随符号链接，
        不按 inode 去重，也不证明文件归属于目标进程。
    """
    files = set()
    for pattern in log_globs:
        if '{pid}' in pattern:
            if pid is None:
                continue
            pattern = pattern.replace('{pid}', str(pid))
        for match in glob.glob(os.path.expanduser(pattern)):
            if os.path.isfile(match):
                files.add(Path(match))
    return sorted(files)


def snapshot(spec: LinkSpec, clock_ticks: Optional[int] = None) -> Optional[LinkedProcess]:
    """读取所选进程的命令行、启动时间、配置哈希与日志大小.

    各项依次读取，不锁定进程、不校验 PID 复用，不保证跨字段一致性。
    相对配置路径优先按进程 cwd 拼接；cwd 不可读时保留相对路径，
    后续哈希按记录器自身当前目录解析，不能当作配置归属证明。

    Args:
        spec: 进程匹配及日志观察规则。
        clock_ticks: 可替换的每秒滴答数，传给启动时间解析器。

    Returns:
        最小匹配 PID 的观察结果，附带其它匹配 PID；没有匹配进程为 None。
        日志 stat 失败的项被跳过，其它未处理异常可传播。
    """
    processes = find_processes(spec.proc_dir, spec.executable)
    if not processes:
        return None
    pid, command = processes[0]
    started = process_start_time(spec.proc_dir, pid, clock_ticks)
    working_dir = _working_directory(spec.proc_dir, pid)
    configs = {}
    for path in config_paths(command, spec.config_flags):
        if not os.path.isabs(path) and working_dir is not None:
            path = os.path.join(working_dir, path)
        configs[path] = manifest.sha256_file(path)
    sizes = {}
    for log in matching_logs(spec.log_globs, pid):
        try:
            sizes[str(log)] = log.stat().st_size
        except OSError:
            continue
    return LinkedProcess(
        pid=pid,
        other_pids=tuple(other for other, _ in processes[1:]),
        started_at=started.astimezone().isoformat(timespec='seconds') if started else None,
        command=tuple(command),
        configs=configs,
        log_sizes=sizes,
    )


def collect_logs(
    spec: LinkSpec,
    linked: Optional[LinkedProcess],
    destination: Path,
    since: float,
) -> List[CopiedLog]:
    """按路径、大小及修改时间截取运行期间可能新增的日志字节.

    已有路径变大时从旧大小开始，变小时从零开始，等长时直接跳过；
    这不是按 inode 跟踪轮转，同路径等长替换不会被发现。新增路径仅在
    mtime 不早于 since 时复制；每个文件最终只保留上限内的尾部。

    来源 stat 或复制的 OSError 被跳过；复制失败可能留下未列入返回值的
    部分目标，不回滚。创建目录等其它错误仍可传播，目标选名不具备并发排他性。

    Args:
        spec: 日志模式及单文件复制上限。
        linked: 启动时观察结果；None 时没有旧大小且跳过含 PID 的模式。
        destination: 调用方拥有的目标目录，有内容需复制时创建；不检查运行目录归属。
        since: 用于筛选新增路径的 Unix 秒级时间。

    Returns:
        成功完成复制调用的摘录列表，不保证来源稳定、字节完整或持久化。
    """
    sizes = linked.log_sizes if linked is not None else {}
    pid = linked.pid if linked is not None else None
    copies = []
    for path in matching_logs(spec.log_globs, pid):
        try:
            status = path.stat()
        except OSError:
            continue
        previous = sizes.get(str(path))
        if previous is None:
            if status.st_mtime < since:
                continue
            start = 0
        elif status.st_size == previous:
            continue
        else:
            start = previous if status.st_size > previous else 0
        truncated = status.st_size - start > spec.max_copy_bytes
        if truncated:
            start = status.st_size - spec.max_copy_bytes
        destination.mkdir(parents=True, exist_ok=True)
        target = _unique_target(destination, path)
        try:
            written = _copy_range(path, target, start, status.st_size)
        except OSError:
            continue
        copies.append(CopiedLog(str(path), str(target), written, truncated))
    return copies


def _working_directory(proc_dir: Path, pid: int) -> Optional[str]:
    try:
        return os.readlink(proc_dir / str(pid) / 'cwd')
    except OSError:
        return None


def _unique_target(destination: Path, source: Path) -> Path:
    """按原文件名、父目录前缀和计数后缀挑选未占用的拷贝目标名.

    这里只按 exists 的当前观察选名，不创建、锁定或原子保留路径，
    因此不能把返回值当作防并发覆盖或符号链接校验凭据。

    Args:
        destination: 本次运行拥有的日志摘录目标目录。
        source: 用于构造文件名和冲突前缀的源路径。

    Returns:
        当前检查未占用的候选路径；冲突计数后缀从 2 开始。
    """
    candidate = destination / source.name
    if not candidate.exists():
        return candidate
    candidate = destination / f'{source.parent.name}__{source.name}'
    counter = 1
    while candidate.exists():
        counter += 1
        candidate = destination / f'{source.parent.name}__{counter}__{source.name}'
    return candidate


def _copy_range(source: Path, target: Path, start: int, end: int) -> int:
    """按有界分块复制来源字节区间，允许来源提前结束而返回实际复制量.

    Args:
        source: 只读来源文件。
        target: 本次选定的摘录文件，wb 打开会截断现有目标。
        start: 来源起始字节偏移。
        end: 来源结束字节偏移，不包含该位置。

    Returns:
        实际写入字节数，可能小于 end-start；不保证来源并发修改时的快照一致性。

    Raises:
        OSError: 来源读取或目标写入失败，目标可能已部分写入。
    """
    written = 0
    with source.open('rb') as reader, target.open('wb') as writer:
        reader.seek(start)
        remaining = end - start
        while remaining > 0:
            chunk = reader.read(min(remaining, _COPY_CHUNK_BYTES))
            if not chunk:
                break
            writer.write(chunk)
            written += len(chunk)
            remaining -= len(chunk)
    return written
