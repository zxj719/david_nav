"""Run manifest: what code, configuration and environment a run used.

The manifest answers "what exactly was running" for every navigation run
(refactor plan §13.2): git state, package versions, ROS environment, the
command line, and content hashes of input files such as the map and the
point-of-interest file.
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
    """Run a command and capture its standard output.

    Args:
        argv: Program and arguments.
        cwd: Working directory, or ``None`` for the current directory.
        timeout_sec: Seconds before the command is abandoned.

    Returns:
        ``(return_code, stdout)``. A missing program yields ``(127, '')`` and a
        timeout yields ``(124, '')``.
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
    """Git state of the code a run uses.

    Attributes:
        repository: Repository top-level directory, or ``None`` outside git.
        branch: Current branch name, ``HEAD`` when detached, or ``None``.
        commit: Full commit hash, or ``None``.
        dirty: Whether tracked or untracked changes exist.
        changed_files: Number of entries reported by ``git status --porcelain``.
    """

    repository: Optional[str]
    branch: Optional[str]
    commit: Optional[str]
    dirty: bool
    changed_files: int


def collect_git_info(repo_dir: Union[str, Path], runner: CommandRunner = run_command) -> GitInfo:
    """Collect the git state of a directory.

    Args:
        repo_dir: Any directory inside the repository.
        runner: Command runner, replaceable in tests.

    Returns:
        The git state. All fields are empty when the directory is not inside a
        repository.
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
    """Write the uncommitted changes of a repository to a file.

    The file lists ``git status --porcelain`` as comments followed by
    ``git diff HEAD``, so the exact code of a dirty run can be reconstructed.

    Args:
        repo_dir: Any directory inside the repository.
        destination: File to write.
        runner: Command runner, replaceable in tests.

    Returns:
        ``True`` if the repository had changes and the file was written.
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
    """Hash a file.

    Args:
        path: File to hash.

    Returns:
        The hex SHA-256 digest, or ``None`` if the file does not exist.
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
    """Report the Python version and installed package versions.

    Args:
        distributions: Distribution names to look up.

    Returns:
        Mapping of ``python`` and each distribution name to its version string,
        or ``not installed``.
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
    """Pick the environment variables that affect a ROS run.

    Args:
        environ: Environment mapping, typically ``os.environ``.
        keys: Variable names to record.

    Returns:
        Mapping of each key to its value, or ``None`` when unset.
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
    """Assemble the manifest of a run that is starting.

    Args:
        run_id: Run directory name.
        mode: Run mode.
        command: Command line being run.
        started_at: Start time; naive values are taken as local time.
        git: Git state of the code.
        versions: Output of :func:`collect_versions`.
        environment: Output of :func:`collect_environment`.
        files: Named input files to fingerprint, for example ``map_yaml``.
        robot: Robot profile name, if known.
        site: Site profile name, if known.
        alias: Shell alias that started the run, if known.
        linked: State of separately started processes linked to the run, by
            name (see :mod:`nav_bringup.recorder.linked_process`); left out of
            the manifest when empty.

    Returns:
        A manifest ready for :func:`write_manifest`. ``ended_at``,
        ``duration_sec``, ``exit_code`` and ``exit_reason`` stay ``None``
        until :func:`finalize_manifest`.
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
    """Return a copy of a manifest completed with the end of the run.

    Args:
        manifest: Manifest produced by :func:`build_manifest`.
        ended_at: End time; naive values are taken as local time.
        exit_code: Exit status of the supervised command, or ``None`` if unknown.
        exit_reason: Short reason, for example ``ctrl_c`` or ``nav2_exited``.

    Returns:
        The finalized manifest; the input is not modified.
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
    """Write a manifest as YAML.

    Args:
        path: Destination file.
        manifest: Manifest to write.
    """
    with Path(path).open('w', encoding='utf-8') as handle:
        yaml.safe_dump(dict(manifest), handle, allow_unicode=True, sort_keys=False)


def read_manifest(path: Union[str, Path]) -> Dict[str, Any]:
    """Read a manifest written by :func:`write_manifest`.

    Args:
        path: Manifest file.

    Returns:
        The manifest mapping.

    Raises:
        ValueError: If the file does not contain a YAML mapping.
    """
    with Path(path).open('r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f'manifest is not a mapping: {path}')
    return data


def _aware(value: datetime.datetime) -> datetime.datetime:
    return value if value.tzinfo is not None else value.astimezone()


def _iso(value: datetime.datetime) -> str:
    return _aware(value).isoformat(timespec='seconds')
