"""执行不依赖 ROS 图的只读导航诊断，输出状态与人工复核提示.

检查读取环境、文件和进程观察，不启动子进程、不删除文件、不发信号或
修改 ROS 状态。实时图检查位于 doctor.live；这里的 OK 只表示已实现的
局部检查通过，不证明整机健康或拥有清理权限。注入的自定义检查由调用方负责。

历史观察原文保留在 docs/adr/0013-doctor-observation-history.md。
提示文本保持既有内容；不能将其中的维护建议视为自动执行授权。
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
    """表示诊断结果等级，不触发任何系统操作.

    Attributes:
        FAIL: 检查失败。
        OK: 检查通过。
        SKIP: 当前条件下未执行检查。
        WARN: 检查发现需注意的问题。
    """

    OK = 'OK'
    SKIP = 'SKIP'
    WARN = 'WARN'
    FAIL = 'FAIL'


_SEVERITY_ORDER = (Status.OK, Status.SKIP, Status.WARN, Status.FAIL)


@dataclass(frozen=True)
class CheckResult:
    """保存单项诊断结果，构造时不验证字段.

    Attributes:
        name: 稳定检查标识，例如 map_yaml_mode。
        status: Status 枚举结果。
        message: 观察到的情况。
        hint: 可选人工复核建议，不代表已执行修复。
    """

    name: str
    status: Status
    message: str
    hint: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        """将结果转为可序列化字段并展开状态枚举值.

        Returns:
            含 name、status、message、hint 的新映射。
        """
        data = asdict(self)
        data['status'] = self.status.value
        return data


@dataclass(frozen=True)
class DoctorContext:
    """保存只读诊断输入，各路径可注入临时夹具.

    字段只是一组参数，不代表已经采样的一致系统快照或可信路径证明。

    Attributes:
        repo_dir: 源码仓库根目录。
        install_dir: 对应工作空间安装目录。
        environ: 检查使用的环境映射，不在此验证导出状态。
        shm_dir: 共享内存目录。
        proc_dir: procfs 根目录。
        serial_device: 预期底盘串口路径，仅检查不打开。
        map_yamls: 要检查 mode 的地图 YAML 路径。
        shm_orphan_warn_threshold: 未映射候选条目数达到此值时告警。
        ros_log_dir: ROS 日志目录，None 时跳过。
        ros_log_warn_entries: 日志目录顶层条目数告警阈值。
        ros_log_warn_bytes: 估算日志字节数告警阈值。
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
    """按 ROS_LOG_DIR、ROS_HOME/log、默认目录的优先级选择日志路径.

    Args:
        environ: 用于读取两个 ROS 变量的环境映射。

    Returns:
        展开用户目录后的路径，不验证存在性；expanduser 使用当前进程环境，
        不读取传入映射里的 HOME 来替换真实用户目录。
    """
    if environ.get('ROS_LOG_DIR'):
        return Path(environ['ROS_LOG_DIR']).expanduser()
    if environ.get('ROS_HOME'):
        return Path(environ['ROS_HOME']).expanduser() / 'log'
    return Path('~/.ros/log').expanduser()


def default_context(environ: Mapping[str, str]) -> DoctorContext:
    """根据环境和仓库地图目录构造诊断输入，不执行诊断.

    Args:
        environ: 环境映射，结果保留其外层副本。

    Returns:
        NAVRUN_REPO 或默认 ~/ros2_ws/src、同级 install、仓库地图 YAML
        及日志目录组成的上下文；地图只按 *.yaml 发现，不证明正在被导航使用。
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
    """检查关键 ROS 环境变量与 DDS 配置文件路径存在性.

    不解析 XML、验证白名单内容、检查实际中间件或尝试 ROS 发现。
    环境隔离历史见 docs/adr/0013-doctor-observation-history.md。

    Args:
        context: 诊断输入。

    Returns:
        ROS_DISTRO 为空时 FAIL；domain 或 profile 缺失、profile 非文件时
        WARN；其余为 OK，不保证与正式进程使用同一传输配置。
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
    """对共享内存目录项与进程映射做只读候选计数.

    计数包含 fastrtps_ 前缀的全部目录项，_el 项归并判断占用但分别计数。
    有候选且映射观察不完整时不宣称孤儿数量；结果不是删除授权。
    相关历史见 docs/adr/0013-doctor-observation-history.md。

    Args:
        context: 目录、procfs 路径及告警阈值。

    Returns:
        目录不存在为 SKIP；目录不可读、有候选但映射不完整、
        或未映射条目数达到阈值为 WARN；其余为 OK。
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
    """检查地图 YAML 的 mode 字段，不读取图像或验证地图内容.

    缺省 mode 按 trinary 处理；YAML 假值先视为空映射。
    只核对配置文本，不能单凭此结果证明未知栅格或代价地图的运行行为。

    Args:
        context: 提供地图 YAML 路径的诊断输入。

    Returns:
        任一可读文件 mode 非 trinary 时优先 FAIL；仅有读取/解析错误时 WARN；
        没有路径为 SKIP，其余 OK。UTF-8 解码等未捕获异常由外层处理。
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
    """按进程命令行子串观察 robot_state_publisher 的匹配数量.

    不查询 ROS 图或 URDF，不证明重复 TF 发布者存在。包装器或命令行参数
    包含该名称也可能被计入，读取失败的进程可能漏计。

    Args:
        context: 提供 procfs 路径的诊断输入。

    Returns:
        多于一个匹配时 WARN，否则 OK；不停止任何匹配进程。
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
    """用受限正则提取单来源 install(PROGRAMS ...) 片段.

    不执行 CMake、不处理完整语法或剥除注释；多文件、复杂变量和其它目标可能遗漏。

    Args:
        cmake_text: CMakeLists.txt 原始文本。

    Returns:
        按文本顺序的 (来源路径, 安装名) 列表，无 RENAME 时用来源 basename。
    """
    programs = []
    for match in _INSTALL_PROGRAMS_RE.finditer(cmake_text):
        source = match.group('source')
        programs.append((source, match.group('name') or Path(source).name))
    return programs


def check_stale_installed_programs(context: DoctorContext) -> CheckResult:
    """比较 david_nav 中可识别的拷贝安装脚本与源码的哈希.

    只检查受限 CMake 正则识别的条目，跳过符号链接、缺失来源和缺失目标；
    OK 不证明整个 install 完整或可执行内容均为最新，零项比较也可为 OK。

    Args:
        context: 仓库及安装目录。

    Returns:
        缺少 CMake 文件或目标 lib 目录时 SKIP，有哈希不一致时 WARN，
        否则 OK。IO/解码异常可传播给外层 run_checks。
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
    """检查预期串口路径存在性和 os.access 权限，不打开设备.

    不验证字符设备类型、波特率、占用情况或下位机响应；普通文件也可能通过。

    Args:
        context: 含设备路径的诊断输入。

    Returns:
        路径缺失或不可同时读写时 WARN，其余 OK。
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
    """统计日志目录顶层条目数与尽力估算的递归字节数.

    子项读取错误被跳过，不代表完整容量统计；无跨文件快照。
    目录遍历错误可能传播。历史数量观察见 docs/adr/0013-doctor-observation-history.md。

    Args:
        context: 日志目录及条目/字节阈值。

    Returns:
        目录未提供或不是目录时 SKIP；任一统计达到阈值时 WARN；其余 OK。
        不删除、截断或改变日志位置。
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
    """依次执行诊断，并将普通异常转为 FAIL 后继续.

    不捕获 BaseException；异常处理依赖检查对象有 __name__ 属性。
    对注入检查不强制只读、副作用隔离或结果类型校验。

    Args:
        context: 原样传给各检查的上下文。
        checks: 按顺序调用的具名检查函数。

    Returns:
        每个成功返回或成功转换异常的检查对应一项结果。
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
    """按诊断枚举顺序选出最严重结果.

    Args:
        results: 含合法 Status 的检查结果序列。

    Returns:
        FAIL 高于 WARN、SKIP、OK；空序列返回 OK，不代表实际检查已执行。
    """
    worst = Status.OK
    for result in results:
        if _SEVERITY_ORDER.index(result.status) > _SEVERITY_ORDER.index(worst):
            worst = result.status
    return worst


def format_report(results: Sequence[CheckResult]) -> str:
    """按冻结的终端布局生成结果和状态计数.

    Args:
        results: 有效的 CheckResult 序列。

    Returns:
        每项一行，只为 WARN/FAIL 输出非空 hint，末尾附计数行及换行。
        消息与 hint 中的换行不做清理，不代表提示动作已执行。
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
    """按拼接命令行的子串查找 procfs 进程，不验证可执行文件身份.

    Args:
        proc_dir: procfs 根目录。
        needle: 在空格拼接命令行中查找的子串。

    Returns:
        按 PID 排序的 (pid, 命令行) 列表。忽略空参数、已知搜索工具及
        cmdline 读取失败项；不剥离其它包装器、不防 PID 复用。
        根目录非目录时返回空列表，目录遍历异常仍可传播。
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
    """按文件名去掉可选的 _el 后缀，归并诊断中的关联共享内存项.

    仅采用既有命名约定，不验证文件类型、映射或所有权，也不删除文件。

    Args:
        name: 待归并的共享内存条目名称。

    Returns:
        去掉末尾 _el 后的名字；其它名称保持不变。
    """
    return name[:-len('_el')] if name.endswith('_el') else name


def _tree_size(path: str) -> int:
    """尽力累计目录树中文件项的 lstat 长度，读不到的项不计入.

    这是诊断估算，不保证完整性，不按 inode 去重；符号链接计链接自身长度，
    而不是目标文件大小。os.walk 默认不递归目录符号链接。

    Args:
        path: 要观察的日志等目录路径，不在此修改文件。

    Returns:
        已成功读取条目的累计字节数；不可遍历的目录可能贡献零。
    """
    total = 0
    for root, _, files in os.walk(path):
        for file_name in files:
            try:
                total += os.lstat(os.path.join(root, file_name)).st_size
            except OSError:
                continue
    return total
