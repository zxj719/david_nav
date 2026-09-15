"""Link a separately started process, such as localization, to a run.

Localization is started on its own (alias 3 on David) and keeps running
across navigation runs, so a run cannot own it. A run record must still say
which localization was running, with which configuration, and what it logged
during the run (refactor plan §13.2). This module answers that read-only:

* :func:`snapshot` finds the process by executable name in procfs at run
  start and records its pid, start time, command line, the SHA-256 of the
  configuration files named on its command line, and the sizes of its log
  files;
* :func:`collect_logs` copies at run end what those log files gained during
  the run, and whole log files written during the run, into the run
  directory, keeping at most the last ``max_copy_bytes`` of each file.

Processes are never signaled, and nothing outside the run directory is
written.
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
    """A separately started process to link to runs.

    Attributes:
        name: Label of the manifest entry and of the ``logs/<name>/`` directory.
        executable: Executable file name, compared with the base name of the
            first command line argument of each process.
        config_flags: Flags whose value names a configuration file to hash.
        log_globs: Glob patterns of its log files; ``{pid}`` is replaced with
            the process id and ``~`` is expanded.
        max_copy_bytes: Most bytes copied per log file; the end is kept.
        proc_dir: procfs mount point.
    """

    name: str
    executable: str
    config_flags: Tuple[str, ...] = DEFAULT_CONFIG_FLAGS
    log_globs: Tuple[str, ...] = ()
    max_copy_bytes: int = DEFAULT_MAX_COPY_BYTES
    proc_dir: Path = Path('/proc')

    def __post_init__(self) -> None:
        """Validate the specification.

        Raises:
            ValueError: If the name is not a single path component, the
                executable is empty, or the copy limit is not positive.
        """
        if not self.name or '/' in self.name or self.name in ('.', '..'):
            raise ValueError(f'invalid link name {self.name!r}')
        if not self.executable:
            raise ValueError('executable must not be empty')
        if self.max_copy_bytes <= 0:
            raise ValueError('max_copy_bytes must be positive')


@dataclass(frozen=True)
class LinkedProcess:
    """State of a linked process at run start.

    Attributes:
        pid: Process id; the lowest one if several instances run.
        other_pids: Ids of further instances.
        started_at: Local start time in ISO 8601, or ``None`` if unknown.
        command: Command line.
        configs: SHA-256 per configuration file path; ``None`` for a missing file.
        log_sizes: Size in bytes of each matching log file.
    """

    pid: int
    other_pids: Tuple[int, ...]
    started_at: Optional[str]
    command: Tuple[str, ...]
    configs: Mapping[str, Optional[str]]
    log_sizes: Mapping[str, int]

    def to_dict(self) -> Dict[str, Any]:
        """Convert the state to manifest data.

        Returns:
            A YAML-serializable mapping.
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
    """A log excerpt copied into a run directory.

    Attributes:
        source: Original log file.
        destination: Copy inside the run directory.
        copied_bytes: Bytes written.
        truncated: Whether the beginning was dropped to respect the limit.
    """

    source: str
    destination: str
    copied_bytes: int
    truncated: bool


def find_processes(proc_dir: Path, executable: str) -> List[Tuple[int, List[str]]]:
    """Find processes by executable name.

    Args:
        proc_dir: procfs mount point.
        executable: File name compared with the base name of the first
            command line argument, so wrappers such as ``ros2 run`` do not match.

    Returns:
        ``(pid, command line)`` pairs sorted by pid.
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
    """Compute when a process started.

    Args:
        proc_dir: procfs mount point.
        pid: Process id.
        clock_ticks: Kernel clock ticks per second; defaults to ``SC_CLK_TCK``.

    Returns:
        The naive local start time, or ``None`` if procfs cannot tell.
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
    """Extract configuration file paths from a command line.

    Args:
        command: Command line arguments.
        flags: Flags whose value is a path, as ``--flag value`` or ``--flag=value``.

    Returns:
        The paths in command line order.
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
    """Expand log file patterns.

    Args:
        log_globs: Glob patterns; ``{pid}`` is replaced and ``~`` is expanded.
        pid: Process id, or ``None`` to skip the patterns that contain ``{pid}``.

    Returns:
        Existing files, sorted and without duplicates.
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
    """Record the state of a linked process at run start.

    Args:
        spec: What to link.
        clock_ticks: Kernel clock ticks per second, replaceable in tests.

    Returns:
        The process state, or ``None`` if no process matches.
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
    """Copy what the linked log files gained during a run.

    A file that existed at the snapshot contributes the bytes appended since
    then, or its whole content if it shrank (rotated or truncated). Any other
    matching file contributes its whole content if it was modified at or
    after ``since``. Only the last ``spec.max_copy_bytes`` of each are kept.

    Args:
        spec: What was linked.
        linked: Snapshot taken at run start, or ``None`` if the process was not
            running then.
        destination: Directory of the copies; created when something is copied.
        since: Run start in seconds since the epoch.

    Returns:
        The copies that were written.
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
