"""为可证明完成的运行记录规划保留策略，并显式应用清理计划.

按 ID 从新到旧计算数量和逻辑字节预算，超出时只保留清单及摘要；
保留运行的非空 bag 超过年龄阈值时可单独清理（PLAN §13.2）。
显式 protected_run_id 不裁剪，不完整/不安全记录保留，因此不是全磁盘硬上限。
完成标记不证明进程已经退出，调用方须处理活动运行保护和外层启动租约。
应用阶段另持每根维护锁，删除不可回滚；本模块导入不执行任何清理。
"""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
import datetime
import math
import os
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple, Union

from nav_bringup.recorder.run_dir import RUN_ID_RE
from nav_bringup.recorder.run_dir import RunDirectory

KEPT_FILES = ('manifest.yaml', 'summary.md')

_SECONDS_PER_DAY = 86400.0


@dataclass(frozen=True)
class RetentionPolicy:
    """表示完整运行制品的保留预算，不限制所有未完成或受保护记录.

    Attributes:
        max_runs: 从最新开始保留的完整记录数量上限。
        max_total_bytes: 已保留记录的逻辑字节预算，不是实际磁盘块数。
        bag_max_age_days: 从运行 ID 起始时间计算的 bag 年龄阈值，严格超出才清理。
    """

    max_runs: int = 30
    max_total_bytes: int = 20 * 1024 ** 3
    bag_max_age_days: float = 7.0

    def __post_init__(self) -> None:
        """在任何记录扫描前拒绝类型不符、负数或非有限预算.

        Raises:
            ValueError: 数量/字节不是非布尔整数，或天数不是有限非负数。
        """
        if (type(self.max_runs) is not int or self.max_runs < 0
                or type(self.max_total_bytes) is not int or self.max_total_bytes < 0
                or type(self.bag_max_age_days) not in (int, float)
                or not math.isfinite(self.bag_max_age_days) or self.bag_max_age_days < 0):
            raise ValueError('retention limits must be finite and nonnegative')


@dataclass(frozen=True)
class GcPlan:
    """表示待复核的清理计划，不是已经完成的删除或持有租约的凭证.

    冻结仅限制字段重赋值，内部列表与字典仍可变；调用方不应并发修改。

    Attributes:
        prune_runs: 拟裁剪为永久保留文件的运行。
        delete_bags: 保持其它制品时拟单独删除的 bag 路径。
        identities: 规划时目录设备号/inode，应用前重新核对。
        skipped: 因完成或安全遍历证据不足而保留的记录及原因。
    """

    prune_runs: List[RunDirectory] = field(default_factory=list)
    delete_bags: List[Path] = field(default_factory=list)
    identities: Dict[str, Tuple[int, int]] = field(default_factory=dict, compare=False, repr=False)
    skipped: Dict[str, str] = field(default_factory=dict, compare=False)


def directory_size(path: Union[str, Path]) -> int:
    """粗略累计目录树中文件条目的逻辑大小，沿用旧辅助接口.

    缺失或非目录根返回零；根路径判断可跟随链接，后代文件链接跳过，
    目录链接默认不递归。普通条目 stat 失败跳过，不提供固定描述符或原子快照。
    不额外验证每个非链接条目都是普通文件；默认 plan_gc 使用安全盘点替代本函数。

    Args:
        path: 调用方信任的目录路径。

    Returns:
        可统计条目的 st_size 之和，不是实际磁盘占用或安全删除依据。
    """
    root = Path(path)
    if not root.is_dir():
        return 0
    total = 0
    for directory, _, filenames in os.walk(root):
        for filename in filenames:
            file_path = Path(directory) / filename
            if file_path.is_symlink():
                continue
            try:
                total += file_path.stat().st_size
            except OSError:
                continue
    return total


def is_pruned(run: RunDirectory) -> bool:
    """只按直接子项名称判断是否仅剩永久保留项，不校验完成状态或类型.

    Args:
        run: 要只读枚举的运行路径对象。

    Returns:
        所有子项名字都在 KEPT_FILES 中为 True；空目录也为 True，
        不能据此证明清单/摘要存在或运行已完成。

    Raises:
        OSError: 运行目录无法枚举。
    """
    return all(entry.name in KEPT_FILES for entry in run.path.iterdir())


def plan_gc(
    runs: Sequence[RunDirectory],
    policy: RetentionPolicy,
    now: datetime.datetime,
    protected_run_id: Optional[str] = None,
    size_of: Callable[[Path], int] = directory_size,
) -> GcPlan:
    """按新到旧的记录顺序生成裁剪及过龄 bag 计划，不删除内容.

    每条先做安全盘点；不安全/未完成条目保留并记 skipped，已裁剪项不计预算。
    受保护 ID 仍计入已保留数量/字节，但不因超预算或年龄而删除。
    预算只统计保留候选，无法限制全盘占用；bag 年龄严格大于阈值才列入计划。

    Args:
        runs: 同一根下的运行列表，可无序，调用方应避免重复或同名冲突。
        policy: 要应用的已校验预算。
        now: 可与运行 ID 的无时区本地时间相减的当前时间。
        protected_run_id: 必须保留的运行，通常由外层显式传入当前 ID。
        size_of: 自定义尺寸端口；保持默认时直接使用安全盘点的字节数。

    Returns:
        未执行的计划；完成标记与目录身份只代表观察时刻，不是执行时授权。

    Raises:
        ValueError: 需要算年龄的运行 ID 日期无效，或自定义尺寸端口拒绝输入。
        TypeError: now 与起始时间不能相减。
        Exception: 自定义尺寸端口的其它异常，不在此隔离。
    """
    from nav_bringup.recorder.safe_retention import inspect_run

    plan = GcPlan()
    kept_count = 0
    kept_bytes = 0
    for run in sorted(runs, key=lambda item: item.run_id, reverse=True):
        try:
            snapshot = inspect_run(run)
        except (OSError, ValueError) as error:
            plan.skipped[run.run_id] = type(error).__name__
            continue
        if snapshot.pruned:
            continue
        protected = run.run_id == protected_run_id
        if not protected and not snapshot.complete:
            plan.skipped[run.run_id] = 'completion_unproven'
            continue
        size = snapshot.size if size_of is directory_size else size_of(run.path)
        over_budget = kept_count >= policy.max_runs or kept_bytes + size > policy.max_total_bytes
        if over_budget and not protected:
            plan.prune_runs.append(run)
            plan.identities[run.run_id] = snapshot.identity
            continue
        kept_count += 1
        kept_bytes += size
        if protected or not snapshot.has_bag:
            continue
        age_days = (now - run.started_at).total_seconds() / _SECONDS_PER_DAY
        if age_days > policy.bag_max_age_days:
            plan.delete_bags.append(run.bag)
            plan.identities[run.run_id] = snapshot.identity
    return plan


def apply_gc(plan: GcPlan, root: Union[str, Path]) -> int:
    """在每根维护租约内复核目标并执行增量删除，不回滚先前删除.

    先检查所有目标路径、盘点及可选的原目录身份，再逐项删除；manifest/summary 保留。
    同一运行若同时被列为 bag 与整记录裁剪，整记录裁剪优先。
    只取得维护锁，不替调用方证明运行不活跃或获取导航启动租约；空计划直接返回。

    Args:
        plan: 来自可信规划步骤的目标集合，调用期间不得并发修改。
        root: 显式根目录，各目标必须属于它的直接运行子目录。

    Returns:
        已删普通文件的逻辑字节数，不是实际回收磁盘块数。

    Raises:
        ValueError: 目标不安全、目录被替换或完成证据不足。
        OSError: 盘点或删除失败，前面已删项目不会恢复。
        LockBusy: 另一个收集器持有同根维护租约。
    """
    from nav_bringup.navrun.locking import RunLock
    from nav_bringup.recorder import safe_retention

    root_path = Path(root).expanduser().absolute()
    targets = {}
    for bag in plan.delete_bags:
        if Path(bag).name != 'bag':
            raise ValueError('invalid bag target')
        _require_run_under(Path(bag).parent, root_path)
        targets[Path(bag).parent.name] = True
    for run in plan.prune_runs:
        _require_run_under(run.path, root_path)
        targets[run.run_id] = False
    if not targets:
        return 0
    # The navigation lease and this per-root maintenance lease have different
    # scopes: wraps do not own the former, but all collectors serialize here.
    with safe_retention.open_root(root_path) as root_fd:
        # Linux procfs resolves this already-owned descriptor, not a root path
        # that another process might replace between inspection and locking.
        with RunLock(Path(f'/proc/self/fd/{root_fd}') / '.gc.lock'):
            snapshots = {}
            for name in targets:
                snapshot = safe_retention.inspect_at(root_fd, name)
                expected = plan.identities.get(name)
                if (not snapshot.complete or expected is not None
                        and snapshot.identity != expected):
                    raise ValueError('planned record changed or is not complete')
                snapshots[name] = snapshot
            removed = 0
            for name, bag_only in targets.items():
                removed += safe_retention.remove_at(root_fd, name, snapshots[name], bag_only)
            return removed


def _require_run_under(run_path: Path, root_path: Path) -> None:
    """按路径字面结构要求待清理运行是记录根的直接子目录.

    展开用户目录并转绝对路径，拒绝 .. 与非法运行 ID；不跟随符号链接
    验证真实目录身份，后续删除流程仍需完成类型和设备号/inode 复核。

    Args:
        run_path: 要检查的运行目录路径，不在本函数中删除。
        root_path: 调用方已规范化的绝对记录根。

    Raises:
        ValueError: 运行路径不在指定根下一级，或名称不符合运行 ID 格式。
    """
    path = run_path.expanduser().absolute()
    if ('..' in path.parts or path.parent != root_path
            or RUN_ID_RE.fullmatch(path.name) is None):
        raise ValueError('retention target is not directly under its selected root')
