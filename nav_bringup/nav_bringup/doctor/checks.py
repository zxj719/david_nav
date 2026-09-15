"""Read-only health checks for a navigation machine (``navrun doctor``).

Each check encodes a failure mode that cost debugging time before (refactor
plan §13.3) and reports ``OK``, ``WARN``, ``FAIL`` or ``SKIP`` with a hint on
how to fix it. Checks never modify files, processes or ROS state and start no
subprocesses.

This module holds the checks that need no running ROS graph; the checks of a
live graph are in :mod:`nav_bringup.doctor.live`.

Messages and hints are written in Chinese for the people operating the robot.
"""

from __future__ import annotations

from dataclasses import asdict
from dataclasses import dataclass
import enum
import os
from pathlib import Path
import re
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

import yaml

from nav_bringup.navrun.dds_hygiene import snapshot_mappings
from nav_bringup.recorder.manifest import sha256_file

_SHM_PREFIX = 'fastrtps_'
_INSTALL_PROGRAMS_RE = re.compile(
    r'install\(\s*PROGRAMS\s+(?P<source>\S+)\s+DESTINATION\s+lib/\$\{PROJECT_NAME\}'
    r'(?:\s+RENAME\s+(?P<name>\S+))?\s*\)',
)
_IGNORED_PROCESS_NAMES = frozenset({'pgrep', 'grep', 'pkill', 'ps'})


class Status(str, enum.Enum):
    """Outcome of a check."""

    OK = 'OK'
    SKIP = 'SKIP'
    WARN = 'WARN'
    FAIL = 'FAIL'


_SEVERITY_ORDER = (Status.OK, Status.SKIP, Status.WARN, Status.FAIL)


@dataclass(frozen=True)
class CheckResult:
    """Result of one check.

    Attributes:
        name: Stable check identifier, for example ``map_yaml_mode``.
        status: Outcome.
        message: What was found.
        hint: How to fix a warning or failure, or ``None``.
    """

    name: str
    status: Status
    message: str
    hint: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert the result to JSON-serializable data.

        Returns:
            A mapping with ``name``, ``status``, ``message`` and ``hint``.
        """
        data = asdict(self)
        data['status'] = self.status.value
        return data


@dataclass(frozen=True)
class DoctorContext:
    """Machine facts the checks inspect; every path is injectable for tests.

    Attributes:
        repo_dir: Source repository, for example ``~/ros2_ws/src``.
        install_dir: colcon install directory of that workspace.
        environ: Environment of the shell running the doctor.
        shm_dir: Shared memory directory.
        proc_dir: procfs mount point.
        serial_device: Chassis controller serial device.
        map_yamls: Map yaml files used by navigation.
        shm_orphan_warn_threshold: Orphan Fast DDS segment count that warns.
        ros_log_dir: ROS log directory, or ``None`` to skip its check.
        ros_log_warn_entries: Entry count of the ROS log directory that warns.
        ros_log_warn_bytes: Size of the ROS log directory that warns.
    """

    repo_dir: Path
    install_dir: Path
    environ: Mapping[str, str]
    shm_dir: Path = Path('/dev/shm')
    proc_dir: Path = Path('/proc')
    serial_device: Path = Path('/dev/wheeltec_controller')
    map_yamls: Sequence[Path] = ()
    shm_orphan_warn_threshold: int = 10
    ros_log_dir: Optional[Path] = None
    ros_log_warn_entries: int = 10000
    ros_log_warn_bytes: int = 1 << 30


def ros_log_directory(environ: Mapping[str, str]) -> Path:
    """Locate the ROS log directory the way ROS 2 logging does.

    Args:
        environ: Environment, typically ``os.environ``.

    Returns:
        ``$ROS_LOG_DIR``, else ``$ROS_HOME/log``, else ``~/.ros/log``.
    """
    if environ.get('ROS_LOG_DIR'):
        return Path(environ['ROS_LOG_DIR']).expanduser()
    if environ.get('ROS_HOME'):
        return Path(environ['ROS_HOME']).expanduser() / 'log'
    return Path('~/.ros/log').expanduser()


def default_context(environ: Mapping[str, str]) -> DoctorContext:
    """Build the context for this machine.

    Args:
        environ: Environment, typically ``os.environ``.

    Returns:
        A context for ``$NAVRUN_REPO`` (default ``~/ros2_ws/src``), its
        sibling ``install`` directory, ``david_nav/config/maps/*.yaml`` and
        the ROS log directory.
    """
    repo_dir = Path(environ.get('NAVRUN_REPO', '~/ros2_ws/src')).expanduser()
    maps_dir = repo_dir / 'david_nav' / 'config' / 'maps'
    return DoctorContext(
        repo_dir=repo_dir,
        install_dir=repo_dir.parent / 'install',
        environ=dict(environ),
        map_yamls=tuple(sorted(maps_dir.glob('*.yaml'))) if maps_dir.is_dir() else (),
        ros_log_dir=ros_log_directory(environ),
    )


def check_ros_environment(context: DoctorContext) -> CheckResult:
    """Check that the shell loaded the ROS and DDS environment.

    A shell without the DDS whitelist profile sees only part of the ROS graph,
    which once looked exactly like "the TF tree is empty".

    Args:
        context: Machine facts.

    Returns:
        ``FAIL`` without ``ROS_DISTRO``; ``WARN`` without ``ROS_DOMAIN_ID`` or
        an existing ``FASTRTPS_DEFAULT_PROFILES_FILE``; otherwise ``OK``.
    """
    name = 'ros_environment'
    env = context.environ
    if not env.get('ROS_DISTRO'):
        return CheckResult(
            name, Status.FAIL, '未加载 ROS 环境（ROS_DISTRO 为空）',
            'source /opt/ros/humble/setup.bash')
    problems = []
    if not env.get('ROS_DOMAIN_ID'):
        problems.append('ROS_DOMAIN_ID 未设置（默认 0）')
    profile = env.get('FASTRTPS_DEFAULT_PROFILES_FILE')
    if not profile:
        problems.append('FASTRTPS_DEFAULT_PROFILES_FILE 未设置')
    elif not Path(profile).is_file():
        problems.append(f'FASTRTPS_DEFAULT_PROFILES_FILE 指向的文件不存在：{profile}')
    if problems:
        return CheckResult(
            name, Status.WARN, '；'.join(problems),
            '手敲 ros2 命令前先加载 DDS 白名单：set -o allexport; '
            'source <david_nav>/config/ros_env.conf; set +o allexport; 再 ros2 daemon stop，'
            '否则可能只看见一部分节点')
    return CheckResult(
        name, Status.OK,
        f"ROS_DISTRO={env['ROS_DISTRO']} ROS_DOMAIN_ID={env['ROS_DOMAIN_ID']} "
        f'DDS profile={profile}')


def check_dds_shared_memory(context: DoctorContext) -> CheckResult:
    """Count orphan Fast DDS shared memory segments.

    Orphan segments accumulate when ROS processes are killed; enough of them
    once broke same-host discovery completely.

    Args:
        context: Machine facts.

    Returns:
        ``WARN`` when orphans reach the threshold or observations are incomplete,
        ``SKIP`` without the shared memory directory, otherwise ``OK``.
    """
    name = 'dds_shared_memory'
    if not context.shm_dir.is_dir():
        return CheckResult(name, Status.SKIP, f'{context.shm_dir} 不存在')
    try:
        segments = sorted(
            entry.name for entry in context.shm_dir.iterdir()
            if entry.name.startswith(_SHM_PREFIX))
    except OSError:
        return CheckResult(name, Status.WARN, '无法读取 Fast DDS 共享内存目录，无法判定孤儿段')
    view = snapshot_mappings(context.proc_dir)
    if segments and not view.complete:
        return CheckResult(
            name, Status.WARN,
            f'Fast DDS 共享内存段 {len(segments)} 个；进程映射观察不完整，无法判定孤儿段',
            '保留现有段；先核对进程可见性和映射读取权限，不停止定位或删除在用段')
    in_use = {_segment_base(segment) for segment in view.names}
    orphans = [segment for segment in segments if _segment_base(segment) not in in_use]
    message = f'Fast DDS 共享内存段 {len(segments)} 个，本次观察未映射的候选孤儿段 {len(orphans)} 个'
    if len(orphans) >= context.shm_orphan_warn_threshold:
        return CheckResult(
            name, Status.WARN, message,
            '孤儿段积压会影响同机发现。请在人工安排的维护窗口复查；'
            '不要为清理而启动整套导航，不停止定位或删除在用段')
    return CheckResult(name, Status.OK, message)


def check_map_yaml_modes(context: DoctorContext) -> CheckResult:
    """Check that navigation map yamls use ``mode: trinary``.

    With ``mode: scale`` the unknown cells of a lightning map (gray value 128)
    become lethal obstacles in the static costmap layer.

    Args:
        context: Machine facts.

    Returns:
        ``FAIL`` if any map yaml uses another mode, ``WARN`` if a file cannot
        be read, ``SKIP`` without map yamls, otherwise ``OK``.
    """
    name = 'map_yaml_mode'
    if not context.map_yamls:
        return CheckResult(name, Status.SKIP, '没有找到导航用的地图 yaml')
    wrong: List[str] = []
    unreadable: List[str] = []
    for path in context.map_yamls:
        try:
            data = yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}
        except (OSError, yaml.YAMLError) as error:
            unreadable.append(f'{path}: {error}')
            continue
        mode = data.get('mode', 'trinary') if isinstance(data, dict) else None
        if mode != 'trinary':
            wrong.append(f'{Path(path).name}: mode={mode}')
    if wrong:
        return CheckResult(
            name, Status.FAIL, '地图 yaml 不是 trinary 模式：' + '，'.join(wrong),
            '导航用的地图 yaml 必须是 mode: trinary（可另建 yaml 指向同一张 pgm），'
            '否则未知区域会被当成障碍')
    if unreadable:
        return CheckResult(name, Status.WARN, '无法读取：' + '，'.join(unreadable))
    return CheckResult(name, Status.OK, f'{len(context.map_yamls)} 个地图 yaml 均为 trinary')


def check_robot_state_publishers(context: DoctorContext) -> CheckResult:
    """Check that at most one robot_state_publisher runs.

    Two URDF publishers give ``base_link`` two parents and split the TF tree.

    Args:
        context: Machine facts.

    Returns:
        ``WARN`` with more than one process, otherwise ``OK``.
    """
    name = 'robot_state_publishers'
    found = find_processes(context.proc_dir, 'robot_state_publisher')
    pids = ', '.join(str(pid) for pid, _ in found)
    if len(found) > 1:
        return CheckResult(
            name, Status.WARN, f'{len(found)} 个 robot_state_publisher 在运行（pid {pids}）',
            '先关掉残留的 robot_state_publisher（上肢或 MoveIt 流程可能留下），否则 TF 会裂成两棵树')
    return CheckResult(name, Status.OK, f'{len(found)} 个 robot_state_publisher 在运行')


def parse_installed_programs(cmake_text: str) -> List[Tuple[str, str]]:
    """Extract ``install(PROGRAMS ...)`` entries from a CMakeLists.txt.

    Args:
        cmake_text: Contents of ``CMakeLists.txt``.

    Returns:
        ``(source_path, installed_name)`` pairs in file order; the installed
        name defaults to the source file name when no ``RENAME`` is given.
    """
    programs = []
    for match in _INSTALL_PROGRAMS_RE.finditer(cmake_text):
        source = match.group('source')
        programs.append((source, match.group('name') or Path(source).name))
    return programs


def check_stale_installed_programs(context: DoctorContext) -> CheckResult:
    """Compare copied executables in the install directory with their sources.

    ``install(PROGRAMS)`` copies scripts even with ``--symlink-install``, so an
    edited source keeps running the old copy until the next build.

    Args:
        context: Machine facts.

    Returns:
        ``WARN`` listing copies that differ from their sources, ``SKIP`` when
        the package or its install directory is missing, otherwise ``OK``.
    """
    name = 'stale_installed_programs'
    package_dir = context.repo_dir / 'david_nav'
    cmake = package_dir / 'CMakeLists.txt'
    lib_dir = context.install_dir / 'david_nav' / 'lib' / 'david_nav'
    if not cmake.is_file() or not lib_dir.is_dir():
        return CheckResult(name, Status.SKIP, '未找到 david_nav 源码或其 install 目录')
    stale = []
    compared = 0
    for source, installed_name in parse_installed_programs(cmake.read_text(encoding='utf-8')):
        installed = lib_dir / installed_name
        source_path = package_dir / source
        if installed.is_symlink() or not installed.is_file() or not source_path.is_file():
            continue
        compared += 1
        if sha256_file(installed) != sha256_file(source_path):
            stale.append(installed_name)
    if stale:
        return CheckResult(
            name, Status.WARN,
            f'install 目录里 {len(stale)} 个脚本与源码不一致：' + '，'.join(stale),
            '这些脚本是拷贝安装的：改了源码要重新 colcon build，'
            '否则 ros2 run david_nav <脚本> 跑的仍是旧版本')
    return CheckResult(name, Status.OK, f'{compared} 个拷贝安装的脚本与源码一致')


def check_serial_device(context: DoctorContext) -> CheckResult:
    """Check that the chassis controller serial device is present and accessible.

    The device is only inspected with ``os.access``; it is never opened.

    Args:
        context: Machine facts.

    Returns:
        ``WARN`` if the device is missing or not readable and writable,
        otherwise ``OK``.
    """
    name = 'serial_device'
    device = context.serial_device
    if not device.exists():
        return CheckResult(
            name, Status.WARN, f'{device} 不存在',
            '底盘控制板未连接或 udev 规则未生效（ls -l /dev/ttyACM*；'
            '规则见 third_party/wheeltec_original/scripts/wheeltec_udev.sh）')
    if not os.access(device, os.R_OK | os.W_OK):
        return CheckResult(
            name, Status.WARN, f'当前用户对 {device} 没有读写权限',
            '把用户加入 dialout 组，或检查 udev 规则里的 MODE')
    return CheckResult(name, Status.OK, f'{device} -> {os.path.realpath(device)} 可读写')


def check_ros_log_dir(context: DoctorContext) -> CheckResult:
    """Check how much the ROS log directory has accumulated.

    Every ros2 command and every ROS process writes a log file there, and
    nothing removes them; on David it once held about 40,000 files.

    Args:
        context: Machine facts.

    Returns:
        ``WARN`` at or above the entry count or size limit, ``SKIP`` without
        the directory, otherwise ``OK``.
    """
    name = 'ros_log_dir'
    directory = context.ros_log_dir
    if directory is None or not directory.is_dir():
        return CheckResult(name, Status.SKIP, f'ROS 日志目录不存在：{directory}')
    entries = 0
    total = 0
    with os.scandir(directory) as scan:
        for entry in scan:
            entries += 1
            try:
                if entry.is_dir(follow_symlinks=False):
                    total += _tree_size(entry.path)
                else:
                    total += entry.stat(follow_symlinks=False).st_size
            except OSError:
                continue
    message = f'{directory}：{entries} 个条目，{total / (1 << 20):.1f} MiB'
    if entries >= context.ros_log_warn_entries or total >= context.ros_log_warn_bytes:
        return CheckResult(
            name, Status.WARN, message,
            '每条 ros2 命令、每个 ROS 进程都会写日志且不会自动清理；确认无用后可删除旧日志，'
            '或给导航运行设置 ROS_LOG_DIR（运行记录会按保留策略清理）')
    return CheckResult(name, Status.OK, message)


ALL_CHECKS: Tuple[Callable[[DoctorContext], CheckResult], ...] = (
    check_ros_environment,
    check_dds_shared_memory,
    check_map_yaml_modes,
    check_robot_state_publishers,
    check_stale_installed_programs,
    check_serial_device,
    check_ros_log_dir,
)


def run_checks(
    context: Any,
    checks: Sequence[Callable[[Any], CheckResult]] = ALL_CHECKS,
) -> List[CheckResult]:
    """Run checks, turning an unexpected error in a check into a ``FAIL`` result.

    Args:
        context: Facts passed to every check, such as a :class:`DoctorContext`.
        checks: Checks to run, in order.

    Returns:
        One result per check.
    """
    results = []
    for check in checks:
        try:
            results.append(check(context))
        except Exception as error:  # noqa: BLE001 - isolate one broken check from the rest
            name = check.__name__[len('check_'):] if check.__name__.startswith('check_') else (
                check.__name__)
            results.append(CheckResult(name, Status.FAIL, f'检查本身出错：{error!r}'))
    return results


def worst_status(results: Sequence[CheckResult]) -> Status:
    """Return the most severe status among results.

    Args:
        results: Check results.

    Returns:
        ``FAIL`` > ``WARN`` > ``SKIP`` > ``OK``; ``OK`` for no results.
    """
    worst = Status.OK
    for result in results:
        if _SEVERITY_ORDER.index(result.status) > _SEVERITY_ORDER.index(worst):
            worst = result.status
    return worst


def format_report(results: Sequence[CheckResult]) -> str:
    """Format results as terminal text.

    Args:
        results: Check results.

    Returns:
        One line per check plus an indented hint line for each hint, and a
        final line with the counts per status.
    """
    lines = []
    for result in results:
        lines.append(f'[{result.status.value:^4}] {result.name}: {result.message}')
        if result.hint and result.status in (Status.WARN, Status.FAIL):
            lines.append(f'       提示：{result.hint}')
    counts = {status: sum(1 for r in results if r.status is status) for status in _SEVERITY_ORDER}
    lines.append(
        f"合计：OK {counts[Status.OK]} / WARN {counts[Status.WARN]} / "
        f"FAIL {counts[Status.FAIL]} / SKIP {counts[Status.SKIP]}")
    return '\n'.join(lines) + '\n'


def find_processes(proc_dir: Path, needle: str) -> List[Tuple[int, str]]:
    """Find processes whose command line contains a string.

    Args:
        proc_dir: procfs mount point.
        needle: Substring to look for in the command line.

    Returns:
        ``(pid, command_line)`` pairs sorted by pid, excluding search tools
        such as ``pgrep``.
    """
    found = []
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
        if not arguments or Path(arguments[0]).name in _IGNORED_PROCESS_NAMES:
            continue
        command_line = ' '.join(arguments)
        if needle in command_line:
            found.append((int(entry.name), command_line))
    return sorted(found)


def _segment_base(name: str) -> str:
    return name[:-len('_el')] if name.endswith('_el') else name


def _tree_size(path: str) -> int:
    total = 0
    for root, _, files in os.walk(path):
        for file_name in files:
            try:
                total += os.lstat(os.path.join(root, file_name)).st_size
            except OSError:
                continue
    return total
