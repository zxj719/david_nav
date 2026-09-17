"""记录运行使用的代码、配置指纹和选定环境信息（PLAN §13.2）.

清单包含 git 查询、Python 分发版本、选定 ROS 环境、原始命令及输入文件哈希。
这些信息分次观察，不是宿主状态或代码树的原子快照，也不证明进程实际启动。
命令及调用方提供的数据不自动脱敏；存储位置和访问权限由外层负责。
"""

from __future__ import annotations

from dataclasses import asdict
from dataclasses import dataclass
import datetime
import hashlib
from importlib import metadata
from pathlib import Path
import platform
import socket
import subprocess
from typing import Any, Callable, Dict, Mapping, Optional, Sequence, Tuple, Union

import yaml

CommandRunner = Callable[[Sequence[str]], Tuple[int, str]]

ENVIRONMENT_KEYS = (
    'ROS_DISTRO',
    'ROS_DOMAIN_ID',
    'ROS_LOCALHOST_ONLY',
    'RMW_IMPLEMENTATION',
    'FASTRTPS_DEFAULT_PROFILES_FILE',
    'NAVRUN_ALIAS',
)
VERSIONED_DISTRIBUTIONS = ('websockets', 'PyYAML', 'psutil', 'numpy', 'pytest', 'flake8')
MANIFEST_SCHEMA_VERSION = 1

_HASH_CHUNK_BYTES = 1 << 20


def run_command(
    argv: Sequence[str],
    cwd: Optional[Union[str, Path]] = None,
    timeout_sec: float = 10.0,
) -> Tuple[int, str]:
    """同步执行参数列表并捕获输出，返回码非零不抛命令失败异常.

    不经 shell；stderr 被捕获但不返回。超时行为委托 subprocess.run，
    不提供整个子进程树的所有权回收或可抢占初始化期限。

    Args:
        argv: 程序名及参数序列。
        cwd: 工作目录，None 表示继承当前目录。
        timeout_sec: 传给 subprocess.run 的超时秒数。

    Returns:
        退出码和 stdout；找不到程序/目录为 (127, '')，超时为 (124, '')。

    Raises:
        OSError: 缺失以外的进程创建或执行系统错误。
        UnicodeError: 文本输出解码失败。
    """
    try:
        completed = subprocess.run(
            list(argv),
            cwd=cwd,
            capture_output=True,
            check=False,
            text=True,
            timeout=timeout_sec,
        )
    except FileNotFoundError:
        return 127, ''
    except subprocess.TimeoutExpired:
        return 124, ''
    return completed.returncode, completed.stdout


@dataclass(frozen=True)
class GitInfo:
    """表示分次命令采集的 git 状态，不是代码树一致性快照.

    Attributes:
        repository: 仓库根目录文本，不在仓库时为 None。
        branch: 分支名、分离状态的 HEAD 或 None。
        commit: 完整提交哈希或 None。
        dirty: 状态查询输出是否含非空行，不证明失败的查询对应干净仓库。
        changed_files: porcelain 输出非空行数，不等同所有变更文件的完整计数。
    """

    repository: Optional[str]
    branch: Optional[str]
    commit: Optional[str]
    dirty: bool
    changed_files: int


def collect_git_info(repo_dir: Union[str, Path], runner: CommandRunner = run_command) -> GitInfo:
    """通过注入命令端口采集仓库根、分支、提交及变更行数.

    仅仓库根查询的退出码决定是否按“非仓库”返回；其余查询沿用输出内容，
    不把非零退出独立标记为错误。因此空状态输出不能单独证明仓库干净。

    Args:
        repo_dir: 要查询的仓库内路径。
        runner: 可替换命令端口，不在此固定环境或快照。

    Returns:
        GitInfo；根查询失败时文本字段为 None、dirty 为 False、计数为零。
    """
    def git(*args: str) -> Tuple[int, str]:
        return runner(['git', '-C', str(repo_dir), *args])

    code, top_level = git('rev-parse', '--show-toplevel')
    if code != 0:
        return GitInfo(repository=None, branch=None, commit=None, dirty=False, changed_files=0)
    _, branch = git('rev-parse', '--abbrev-ref', 'HEAD')
    _, commit = git('rev-parse', 'HEAD')
    _, status = git('status', '--porcelain')
    changed = [line for line in status.splitlines() if line.strip()]
    return GitInfo(
        repository=top_level.strip() or None,
        branch=branch.strip() or None,
        commit=commit.strip() or None,
        dirty=bool(changed),
        changed_files=len(changed),
    )


def write_code_diff(
    repo_dir: Union[str, Path],
    destination: Union[str, Path],
    runner: CommandRunner = run_command,
) -> bool:
    """将状态行与 git diff HEAD 文本写入差异制品，不导出整个工作树.

    未跟踪文件只记录名称，不保存其内容；二进制差异或查询失败也不保证可重建。
    不检查命令退出码、不固定两次查询快照；目标写入非原子且可能覆盖原内容。

    Args:
        repo_dir: 仓库内路径。
        destination: 由外层选定的可写制品路径，父目录须已存在。
        runner: 可替换命令端口。

    Returns:
        状态存在非空行并完成写入时为 True，否则为 False 且不写目标。

    Raises:
        OSError: 目标无法写入或注入端口发生系统错误。
    """
    _, status = runner(['git', '-C', str(repo_dir), 'status', '--porcelain'])
    entries = [line for line in status.splitlines() if line.strip()]
    if not entries:
        return False
    _, diff = runner(['git', '-C', str(repo_dir), 'diff', 'HEAD'])
    lines = ['# git status --porcelain']
    lines.extend(f'# {entry}' for entry in entries)
    lines.extend(['', diff])
    Path(destination).write_text('\n'.join(lines), encoding='utf-8')
    return True


def sha256_file(path: Union[str, Path]) -> Optional[str]:
    """按块读取当前文件内容的 SHA-256，不锁文件或固定目录身份.

    普通路径判断会跟随符号链接；读期间文件可能改变，不能保证与其它制品同一快照。

    Args:
        path: 调用方已选择的输入文件。

    Returns:
        十六进制摘要；路径缺失或不是文件时为 None。

    Raises:
        OSError: 已存在文件无法打开或读取。
    """
    file_path = Path(path)
    if not file_path.is_file():
        return None
    digest = hashlib.sha256()
    with file_path.open('rb') as handle:
        for chunk in iter(lambda: handle.read(_HASH_CHUNK_BYTES), b''):
            digest.update(chunk)
    return digest.hexdigest()


def collect_versions(distributions: Sequence[str] = VERSIONED_DISTRIBUTIONS) -> Dict[str, str]:
    """读取当前解释器及已安装分发包的版本元数据，不导入目标包.

    Args:
        distributions: 要查询的分发名称序列。

    Returns:
        python 与分发名称到版本文本的映射，未安装项为 not installed；
        元数据版本不证明实际加载路径、ABI 或运行可用性。
    """
    versions = {'python': platform.python_version()}
    for name in distributions:
        try:
            versions[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            versions[name] = 'not installed'
    return versions


def collect_environment(
    environ: Mapping[str, str],
    keys: Sequence[str] = ENVIRONMENT_KEYS,
) -> Dict[str, Optional[str]]:
    """按显式键集合复制环境值，不自动记录全部环境或隐藏敏感值.

    Args:
        environ: 通常为 os.environ 的环境映射。
        keys: 要写入清单的变量名，外层须确保适合记录。

    Returns:
        每个指定键到原始值的字典，未设置时为 None。
    """
    return {key: environ.get(key) for key in keys}


def build_manifest(
    *,
    run_id: str,
    mode: str,
    command: Sequence[str],
    started_at: datetime.datetime,
    git: GitInfo,
    versions: Mapping[str, str],
    environment: Mapping[str, Optional[str]],
    files: Optional[Mapping[str, Union[str, Path]]] = None,
    robot: Optional[str] = None,
    site: Optional[str] = None,
    alias: Optional[str] = None,
    linked: Optional[Mapping[str, Any]] = None,
) -> Dict[str, Any]:
    """汇总启动清单并读取输入文件指纹，不写文件或启动命令.

    包含主机名、原始命令和调用方提供的环境；不自动脱敏或完整验证输入类型。
    外层映射复制但嵌套对象可能共享，文件哈希分次读取而非同一原子快照。

    Args:
        run_id: 运行目录名称。
        mode: 运行模式。
        command: 准备执行的命令行。
        started_at: 启动时间，无时区值按本地时间解释，文本省略微秒。
        git: 预先采集的 git 状态。
        versions: 预先采集的版本映射。
        environment: 选定的环境映射。
        files: 名称到输入文件路径的映射，缺失文件的摘要为 None。
        robot: 已知时填写的机器人档案名称。
        site: 已知时填写的场地档案名称。
        alias: 已知时填写的启动 alias。
        linked: 独立进程的关联观察，空值不写该字段，非空只复制外层映射。

    Returns:
        可交给 write_manifest 的清单，结束时间、时长、退出码和原因为 None。

    Raises:
        OSError: 某个可见输入文件无法读取。
    """
    fingerprints = {
        name: {'path': str(path), 'sha256': sha256_file(path)}
        for name, path in (files or {}).items()
    }
    record = {
        'schema_version': MANIFEST_SCHEMA_VERSION,
        'run_id': run_id,
        'mode': mode,
        'robot': robot,
        'site': site,
        'alias': alias,
        'host': socket.gethostname(),
        'command': list(command),
        'started_at': _iso(started_at),
        'ended_at': None,
        'duration_sec': None,
        'exit_code': None,
        'exit_reason': None,
        'git': asdict(git),
        'versions': dict(versions),
        'environment': dict(environment),
        'files': fingerprints,
    }
    if linked:
        record['linked'] = dict(linked)
    return record


def finalize_manifest(
    manifest: Mapping[str, Any],
    ended_at: datetime.datetime,
    exit_code: Optional[int],
    exit_reason: str,
) -> Dict[str, Any]:
    """浅复制清单并补结束时间与退出字段，不写原始外层映射.

    嵌套值仍共享；若开始时间为字符串则计算时长，否则保留已有时长字段。
    允许负时长，不验证进程已退出或所有资源已释放；由监督者决定何时完成。

    Args:
        manifest: 待补结束字段的清单。
        ended_at: 结束时间，无时区值按本地时间解释。
        exit_code: 被监督命令退出码，未知为 None。
        exit_reason: 监督者给出的简短退出原因。

    Returns:
        新的外层字典，结束文本到秒、时长按毫秒精度舍入。

    Raises:
        ValueError: 字符串开始时间不能解析。
        TypeError: 开始/结束时间的时区形式无法相减。
    """
    finalized = dict(manifest)
    ended = _aware(ended_at)
    finalized['ended_at'] = ended.isoformat(timespec='seconds')
    started = finalized.get('started_at')
    if isinstance(started, str):
        duration = ended - datetime.datetime.fromisoformat(started)
        finalized['duration_sec'] = round(duration.total_seconds(), 3)
    finalized['exit_code'] = exit_code
    finalized['exit_reason'] = exit_reason
    return finalized


def write_manifest(path: Union[str, Path], manifest: Mapping[str, Any]) -> None:
    """以 YAML 覆盖写入清单，不进行原子替换或 fsync.

    打开目标时即截断，序列化或写入失败可能留下部分内容；外层负责目录与所有权。

    Args:
        path: 目标文件，父目录须已存在。
        manifest: 要序列化的映射，不自动验证完整 schema。

    Raises:
        OSError: 文件打开、写入或关闭失败。
        yaml.YAMLError: 数据无法由安全 YAML 表示器序列化。
    """
    with Path(path).open('w', encoding='utf-8') as handle:
        yaml.safe_dump(dict(manifest), handle, allow_unicode=True, sort_keys=False)


def read_manifest(path: Union[str, Path]) -> Dict[str, Any]:
    """读取 YAML 并要求顶层为映射，不验证清单身份或完整 schema.

    读取没有大小/深度预算，会按普通路径跟随链接；安全记录浏览应使用受限读取入口。

    Args:
        path: 外层已选择并信任的清单文件。

    Returns:
        解析得到的原始字典，不补默认字段或核对 run_id。

    Raises:
        ValueError: 顶层不是映射。
        OSError: 无法打开或读取文件。
        UnicodeError: 文件不是有效 UTF-8。
        yaml.YAMLError: YAML 无法解析。
    """
    with Path(path).open('r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f'manifest is not a mapping: {path}')
    return data


def _aware(value: datetime.datetime) -> datetime.datetime:
    """为无时区时间补本地时区，已有时区时保留原对象.

    Args:
        value: 待用于运行清单的 datetime，无时区值按系统本地时间解释。

    Returns:
        带时区的 datetime，不统一转换到 UTC。
    """
    return value if value.tzinfo is not None else value.astimezone()


def _iso(value: datetime.datetime) -> str:
    """以秒精度生成带时区的清单时间文本，省略微秒.

    Args:
        value: 日历时间，无时区值先按本地时间补时区。

    Returns:
        带偏移信息的 ISO 时间字符串。
    """
    return _aware(value).isoformat(timespec='seconds')
