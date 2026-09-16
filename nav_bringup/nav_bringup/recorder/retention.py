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
    """Limits applied by :func:`plan_gc`.

    Attributes:
        max_runs: Number of newest runs kept complete.
        max_total_bytes: Size budget for complete runs, counted from the newest.
        bag_max_age_days: Age after which a complete run's ``bag/`` is deleted.
    """

    max_runs: int = 30
    max_total_bytes: int = 20 * 1024 ** 3
    bag_max_age_days: float = 7.0

    def __post_init__(self) -> None:
        """Reject malformed or negative limits before inspecting any record.

        Raises:
            ValueError: A limit has the wrong type or is negative/non-finite.
        """
        if (type(self.max_runs) is not int or self.max_runs < 0
                or type(self.max_total_bytes) is not int or self.max_total_bytes < 0
                or type(self.bag_max_age_days) not in (int, float)
                or not math.isfinite(self.bag_max_age_days) or self.bag_max_age_days < 0):
            raise ValueError('retention limits must be finite and nonnegative')


@dataclass(frozen=True)
class GcPlan:
    """Deletions decided by :func:`plan_gc`.

    Attributes:
        prune_runs: Runs to reduce to :data:`KEPT_FILES`.
        delete_bags: ``bag/`` directories to delete from runs that stay complete.
        identities: Planned device/inode pairs, rechecked before deletion.
        skipped: Records whose completion or safe traversal could not be proved.
    """

    prune_runs: List[RunDirectory] = field(default_factory=list)
    delete_bags: List[Path] = field(default_factory=list)
    identities: Dict[str, Tuple[int, int]] = field(default_factory=dict, compare=False, repr=False)
    skipped: Dict[str, str] = field(default_factory=dict, compare=False)


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
    """Execute a plan produced by :func:`plan_gc`.

    Args:
        plan: Deletions to perform.
        root: Runs root. Every path in the plan must belong to a run directory
            directly under it.

    Returns:
        Removed logical regular-file bytes, not physical disk blocks reclaimed.

    Raises:
        ValueError: A target is unsafe, replaced or no longer provably complete.
        OSError: Inspection or deletion fails. Earlier deletions are not rolled back.
        LockBusy: Another collector holds the per-root maintenance lease.
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
    path = run_path.expanduser().absolute()
    if ('..' in path.parts or path.parent != root_path
            or RUN_ID_RE.fullmatch(path.name) is None):
        raise ValueError('retention target is not directly under its selected root')
