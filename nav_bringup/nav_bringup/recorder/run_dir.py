"""Run directory layout for navigation runs.

Every navigation run gets a self-describing directory under a runs root::

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

A run id is ``<YYYYmmdd_HHMMSS>_<mode>_<short git sha>`` (refactor plan §13.2),
so lexical order is chronological order.
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
    """Return the default runs root.

    Returns:
        ``$NAV_RUNS_ROOT`` when set, otherwise ``~/nav_runs``.
    """
    return Path(os.environ.get('NAV_RUNS_ROOT', '~/nav_runs')).expanduser()


@dataclass(frozen=True)
class RunDirectory:
    """Paths inside one run directory.

    Attributes:
        root: Directory that contains all runs.
        run_id: Name of this run's directory.
    """

    root: Path
    run_id: str

    @property
    def path(self) -> Path:
        """Path: The run directory."""
        return Path(self.root) / self.run_id

    @property
    def manifest(self) -> Path:
        """Path: The run manifest, ``manifest.yaml``."""
        return self.path / 'manifest.yaml'

    @property
    def code_diff(self) -> Path:
        """Path: Uncommitted code changes, ``code.diff``."""
        return self.path / 'code.diff'

    @property
    def events(self) -> Path:
        """Path: The event timeline, ``events.jsonl``."""
        return self.path / 'events.jsonl'

    @property
    def summary(self) -> Path:
        """Path: The human readable summary, ``summary.md``."""
        return self.path / 'summary.md'

    @property
    def logs(self) -> Path:
        """Path: Process logs directory."""
        return self.path / 'logs'

    @property
    def metrics(self) -> Path:
        """Path: Metrics directory, for example ``resources.csv``."""
        return self.path / 'metrics'

    @property
    def bag(self) -> Path:
        """Path: Rosbag directory."""
        return self.path / 'bag'

    @property
    def params_inputs(self) -> Path:
        """Path: Configuration inputs copied verbatim."""
        return self.path / 'params' / 'inputs'

    @property
    def params_rendered(self) -> Path:
        """Path: Parameter files rendered for the launch."""
        return self.path / 'params' / 'rendered'

    @property
    def params_effective(self) -> Path:
        """Path: Parameters dumped from running nodes."""
        return self.path / 'params' / 'effective'

    @property
    def started_at(self) -> datetime.datetime:
        """datetime.datetime: Naive local start time parsed from the run id.

        Raises:
            ValueError: If the run id does not follow the run id format.
        """
        match = RUN_ID_RE.fullmatch(self.run_id)
        if match is None:
            raise ValueError(f'not a run id: {self.run_id!r}')
        return datetime.datetime.strptime(match.group('stamp'), '%Y%m%d_%H%M%S')


def make_run_id(mode: str, git_sha: Optional[str], now: datetime.datetime) -> str:
    """Build a run id.

    Args:
        mode: Run mode such as ``tour`` or ``precise``; lowercase letters,
            digits and underscores only.
        git_sha: Commit hash of the code being run, or ``None`` when unknown.
        now: Local start time of the run.

    Returns:
        ``<YYYYmmdd_HHMMSS>_<mode>_<first 7 characters of git_sha or nogit>``.

    Raises:
        ValueError: If ``mode`` or ``git_sha`` has an invalid format.
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
    """Create the run directory tree and point ``latest`` at it.

    Args:
        root: Runs root; created if missing.
        run_id: Name of the new run directory.

    Returns:
        The created run directory.

    Raises:
        ValueError: If ``run_id`` is not a valid run id.
        FileExistsError: If a run with the same id already exists.
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
    """Atomically point ``<root>/latest`` at a run.

    Args:
        run: The run that becomes the latest one.
    """
    link = Path(run.root) / LATEST_LINK_NAME
    temporary = Path(run.root) / f'.{LATEST_LINK_NAME}.{os.getpid()}'
    if temporary.is_symlink() or temporary.exists():
        temporary.unlink()
    temporary.symlink_to(run.run_id)
    os.replace(temporary, link)


def list_runs(root: Union[str, Path]) -> List[RunDirectory]:
    """List the run directories under a runs root, oldest first.

    Args:
        root: Runs root. A missing root yields an empty list.

    Returns:
        Runs whose directory names are valid run ids, sorted chronologically.
        The ``latest`` link and unrelated entries are ignored.
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
