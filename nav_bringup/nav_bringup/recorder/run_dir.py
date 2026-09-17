"""定义导航运行记录的目录布局与标识，不启动导航进程.

每次运行在显式根目录下建立独立目录（PLAN §13.2）::

    <root>/
    ├── latest -> <run_id>
    └── <run_id>/
        ├── manifest.yaml
        ├── code.diff
        ├── events.jsonl
        ├── summary.md
        ├── params/inputs/  params/rendered/  params/effective/
        ├── logs/
        ├── metrics/
        └── bag/

标识为 <YYYYmmdd_HHMMSS>_<mode>_<short git sha>，按本地日历时间命名，
不编码时区或微秒；名称排序不能证明跨时区/时钟回拨时的真实先后。
这些是受信任根目录上的创建助手，不替代只读入口的描述符安全检查。
"""

from __future__ import annotations

from dataclasses import dataclass
import datetime
import os
from pathlib import Path
import re
from typing import List, Optional, Union

LATEST_LINK_NAME = 'latest'
RUN_ID_RE = re.compile(
    r'^(?P<stamp>\d{8}_\d{6})_(?P<mode>[a-z0-9_]+)_(?P<sha>[0-9a-f]{7}|nogit)$')

_MODE_RE = re.compile(r'^[a-z0-9_]+$')
_SUBDIRECTORIES = (
    'params/inputs',
    'params/rendered',
    'params/effective',
    'logs',
    'metrics',
    'bag',
)


def default_runs_root() -> Path:
    """解析默认记录根路径，不创建目录或验证用途.

    Returns:
        设置 NAV_RUNS_ROOT 时取其值，否则取 ~/nav_runs；展开用户目录符号。
    """
    return Path(os.environ.get('NAV_RUNS_ROOT', '~/nav_runs')).expanduser()


@dataclass(frozen=True)
class RunDirectory:
    """表示一条运行记录的路径集合，不在构造时创建或校验目录.

    Attributes:
        root: 存放运行记录的根目录，由调用方选择并信任。
        run_id: 此次记录目录名；直接构造不会自行验证名称。
    """

    root: Path
    run_id: str

    @property
    def path(self) -> Path:
        """Path: 本次记录目录路径，不证明目录存在或运行仍活跃."""
        return Path(self.root) / self.run_id

    @property
    def manifest(self) -> Path:
        """Path: 运行清单 manifest.yaml 的路径."""
        return self.path / 'manifest.yaml'

    @property
    def code_diff(self) -> Path:
        """Path: 版本差异记录 code.diff 的路径，不保证包含未跟踪文件内容."""
        return self.path / 'code.diff'

    @property
    def events(self) -> Path:
        """Path: 事件时间线 events.jsonl 的路径."""
        return self.path / 'events.jsonl'

    @property
    def summary(self) -> Path:
        """Path: 人工可读摘要 summary.md 的路径."""
        return self.path / 'summary.md'

    @property
    def logs(self) -> Path:
        """Path: 进程日志目录路径."""
        return self.path / 'logs'

    @property
    def metrics(self) -> Path:
        """Path: 资源等度量文件的目录路径."""
        return self.path / 'metrics'

    @property
    def bag(self) -> Path:
        """Path: 录制输出目录路径，不表示已经录制."""
        return self.path / 'bag'

    @property
    def params_inputs(self) -> Path:
        """Path: 原样保存的配置输入目录路径."""
        return self.path / 'params' / 'inputs'

    @property
    def params_rendered(self) -> Path:
        """Path: 为启动生成的参数文件目录路径."""
        return self.path / 'params' / 'rendered'

    @property
    def params_effective(self) -> Path:
        """Path: 运行节点参数转储目录路径，不表示转储已经成功."""
        return self.path / 'params' / 'effective'

    @property
    def started_at(self) -> datetime.datetime:
        """datetime.datetime: 从完整运行 ID 提取的不带时区本地日历时间.

        Raises:
            ValueError: ID 格式不完整匹配，或其日期时间字段不可解析。
        """
        match = RUN_ID_RE.fullmatch(self.run_id)
        if match is None:
            raise ValueError(f'not a run id: {self.run_id!r}')
        return datetime.datetime.strptime(match.group('stamp'), '%Y%m%d_%H%M%S')


def make_run_id(mode: str, git_sha: Optional[str], now: datetime.datetime) -> str:
    """按秒制日历时间、模式及提交短哈希构造运行 ID.

    不对时间做时区转换或碰撞消解；同一秒、模式、提交可能生成同名记录。

    Args:
        mode: 只含小写字母、数字或下划线的完整模式名。
        git_sha: 七至四十位小写十六进制提交哈希，未知时为 None。
        now: 用于名称的日历时间，只保留到秒的字段。

    Returns:
        <YYYYmmdd_HHMMSS>_<mode>_<前七位哈希或nogit> 形式的标识。

    Raises:
        ValueError: mode 或 git_sha 不能完整匹配允许格式。
    """
    if not _MODE_RE.fullmatch(mode):
        raise ValueError(f'invalid run mode {mode!r}')
    if git_sha is None:
        short_sha = 'nogit'
    elif re.fullmatch(r'[0-9a-f]{7,40}', git_sha):
        short_sha = git_sha[:7]
    else:
        raise ValueError(f'invalid git sha {git_sha!r}')
    return f"{now.strftime('%Y%m%d_%H%M%S')}_{mode}_{short_sha}"


def create_run_directory(root: Union[str, Path], run_id: str) -> RunDirectory:
    """创建运行目录树并更新 latest，不复用已有运行目录.

    先完整校验 ID 格式，再创建根及子目录；不在此校验日期字段的日历有效性。
    子目录或 latest 操作失败可能留下部分目录，不提供整次创建的事务回滚。
    根目录由调用方信任，本助手不实现逐级无链接打开或多创建者锁。

    Args:
        root: 显式记录根，不存在时递归创建。
        run_id: 完整匹配命名格式的运行 ID。

    Returns:
        已创建的运行路径对象。

    Raises:
        ValueError: run_id 不完整匹配允许格式。
        FileExistsError: 同名路径已存在或并发创建冲突。
        OSError: 根/子目录创建或 latest 更新失败。
    """
    if not RUN_ID_RE.fullmatch(run_id):
        raise ValueError(f'not a run id: {run_id!r}')
    run = RunDirectory(Path(root), run_id)
    run.root.mkdir(parents=True, exist_ok=True)
    if run.path.exists():
        raise FileExistsError(f'run directory already exists: {run.path}')
    run.path.mkdir()
    for subdirectory in _SUBDIRECTORIES:
        (run.path / subdirectory).mkdir(parents=True)
    update_latest_link(run)
    return run


def update_latest_link(run: RunDirectory) -> None:
    """通过临时相对链接与替换操作更新 latest，原子性仅限最终替换.

    使用同一 PID 命名临时路径，调用方应串行执行；不 fsync 或回滚前面的目录创建。
    本函数不验证 run_id 或目标是否存在，不应直接接收未经校验的外部路径。

    Args:
        run: 调用方已校验且持有的运行路径对象。

    Raises:
        OSError: 临时路径清理、建链或替换失败，临时链接可能仍在。
    """
    link = Path(run.root) / LATEST_LINK_NAME
    temporary = Path(run.root) / f'.{LATEST_LINK_NAME}.{os.getpid()}'
    if temporary.is_symlink() or temporary.exists():
        temporary.unlink()
    temporary.symlink_to(run.run_id)
    os.replace(temporary, link)


def list_runs(root: Union[str, Path]) -> List[RunDirectory]:
    """列出根下真实运行目录，按完整 ID 字典序从小到大返回.

    忽略符号链接和不匹配名称，不读取清单、验证日历日期或证明运行存活。
    没有目录扫描预算或固定描述符，不替代面向不可信记录的只读入口。

    Args:
        root: 显式根目录；缺失或不是目录时返回空列表。

    Returns:
        名称完整匹配的运行路径对象列表；排序不是跨时区的绝对时间排序。

    Raises:
        OSError: 目录枚举或元数据读取失败。
    """
    root_path = Path(root)
    if not root_path.is_dir():
        return []
    runs = [
        RunDirectory(root_path, entry.name)
        for entry in root_path.iterdir()
        if entry.is_dir() and not entry.is_symlink() and RUN_ID_RE.fullmatch(entry.name)
    ]
    return sorted(runs, key=lambda run: run.run_id)
