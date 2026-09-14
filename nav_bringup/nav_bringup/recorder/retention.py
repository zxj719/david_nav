"""Retention policy for run directories.

Keeps disk usage bounded without losing the record of any run (refactor plan
§13.2). Beyond the newest ``max_runs`` complete runs, or once complete runs
exceed ``max_total_bytes`` counting from the newest, older runs are pruned
down to ``manifest.yaml`` and ``summary.md``. Rosbags of complete runs older
than ``bag_max_age_days`` are deleted. The current run is never touched.
"""

from __future__ import annotations

from dataclasses import dataclass
from dataclasses import field
import datetime
import os
from pathlib import Path
import shutil
from typing import Callable, List, Optional, Sequence, Union

from nav_bringup.recorder.run_dir import RUN_ID_RE
from nav_bringup.recorder.run_dir import RunDirectory

KEPT_FILES = ('manifest.yaml', 'summary.md')

_SECONDS_PER_DAY = 86400.0


@dataclass(frozen=True)
class RetentionPolicy:
    """Limits applied by :func:`plan_gc`.

    Attributes:
        max_runs: Number of newest runs kept complete.
        max_total_bytes: Size budget for complete runs, counted from the newest.
        bag_max_age_days: Age after which a complete run's ``bag/`` is deleted.
    """

    max_runs: int = 30
    max_total_bytes: int = 20 * 1024 ** 3
    bag_max_age_days: float = 7.0


@dataclass(frozen=True)
class GcPlan:
    """Deletions decided by :func:`plan_gc`.

    Attributes:
        prune_runs: Runs to reduce to :data:`KEPT_FILES`.
        delete_bags: ``bag/`` directories to delete from runs that stay complete.
    """

    prune_runs: List[RunDirectory] = field(default_factory=list)
    delete_bags: List[Path] = field(default_factory=list)


def directory_size(path: Union[str, Path]) -> int:
    """Sum the sizes of regular files under a directory.

    Args:
        path: Directory. A missing directory has size 0.

    Returns:
        Total size in bytes. Symbolic links are neither followed nor counted.
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
    """Tell whether a run has already been pruned.

    Args:
        run: Run to inspect.

    Returns:
        ``True`` if the run directory contains nothing but :data:`KEPT_FILES`.
    """
    return all(entry.name in KEPT_FILES for entry in run.path.iterdir())


def plan_gc(
    runs: Sequence[RunDirectory],
    policy: RetentionPolicy,
    now: datetime.datetime,
    protected_run_id: Optional[str] = None,
    size_of: Callable[[Path], int] = directory_size,
) -> GcPlan:
    """Decide which runs to prune and which bags to delete.

    Args:
        runs: Runs under one root, in any order.
        policy: Limits to apply.
        now: Current naive local time, comparable with run start times.
        protected_run_id: Run that must stay complete, typically the current one.
        size_of: Directory size function, replaceable in tests.

    Returns:
        The plan. Nothing is deleted here.
    """
    plan = GcPlan()
    kept_count = 0
    kept_bytes = 0
    for run in sorted(runs, key=lambda item: item.run_id, reverse=True):
        if is_pruned(run):
            continue
        protected = run.run_id == protected_run_id
        size = size_of(run.path)
        over_budget = kept_count >= policy.max_runs or kept_bytes + size > policy.max_total_bytes
        if over_budget and not protected:
            plan.prune_runs.append(run)
            continue
        kept_count += 1
        kept_bytes += size
        if protected or not run.bag.is_dir() or not any(run.bag.iterdir()):
            continue
        age_days = (now - run.started_at).total_seconds() / _SECONDS_PER_DAY
        if age_days > policy.bag_max_age_days:
            plan.delete_bags.append(run.bag)
    return plan


def apply_gc(plan: GcPlan, root: Union[str, Path]) -> int:
    """Execute a plan produced by :func:`plan_gc`.

    Args:
        plan: Deletions to perform.
        root: Runs root. Every path in the plan must belong to a run directory
            directly under it.

    Returns:
        Number of bytes freed.

    Raises:
        ValueError: If the plan references a path outside ``root``.
    """
    root_path = Path(root).resolve()
    for bag in plan.delete_bags:
        if Path(bag).name != 'bag':
            raise ValueError(f'not a bag directory: {bag}')
        _require_run_under(Path(bag).parent, root_path)
    for run in plan.prune_runs:
        _require_run_under(run.path, root_path)

    freed = 0
    for bag in plan.delete_bags:
        freed += directory_size(bag)
        shutil.rmtree(bag)
    for run in plan.prune_runs:
        for entry in run.path.iterdir():
            if entry.name in KEPT_FILES:
                continue
            if entry.is_dir() and not entry.is_symlink():
                freed += directory_size(entry)
                shutil.rmtree(entry)
            else:
                freed += entry.lstat().st_size
                entry.unlink()
    return freed


def _require_run_under(run_path: Path, root_path: Path) -> None:
    resolved = run_path.resolve()
    if resolved.parent != root_path or not RUN_ID_RE.match(resolved.name):
        raise ValueError(f'refusing to touch {run_path}: not a run directory of {root_path}')
