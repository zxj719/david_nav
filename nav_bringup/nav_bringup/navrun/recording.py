"""Plan profile-owned SQLite bag recording without starting processes or writing files."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
from typing import Any, Mapping, Tuple


@dataclass(frozen=True)
class RecordingPlan:
    """Exact recording command and its explicit selection.

    Attributes:
        mode: Requested light, full or off policy.
        enabled: Whether a recorder should actually be started after readiness checks.
        argv: Program and arguments, empty when disabled.
        output: Run-local output directory that rosbag2 will create.
        topics: Explicit selected topics, with duplicate entries removed.
        regex: Additional full-mode topic selector, or empty.
        reason: Explanation when no recorder is planned.
    """

    mode: str
    enabled: bool
    argv: Tuple[str, ...]
    output: str
    topics: Tuple[str, ...]
    regex: str = ''
    reason: str = ''


def plan_recording(
    profile: Mapping[str, Any], mode: str, run_dir: Path, ros2: str = 'ros2',
) -> RecordingPlan:
    """Validate one recording selection and construct its shell-free invocation.

    Args:
        profile: Effective robot/mode profile, optionally containing record settings.
        mode: light, full or off; full requires an explicit topic list or regex.
        run_dir: Absolute owned run record; its bag parent already belongs to the recorder.
        ros2: ROS CLI executable resolved by the supervisor, or ros2 in a preview.

    Returns:
        Pure recording plan. Empty light lists are explicitly disabled; no fallback
        to all topics, automatic plugin installation or implicit MCAP is performed.

    Raises:
        ValueError: A mode, path, storage plugin, selector or option is invalid.
    """
    if mode not in ('light', 'full', 'off'):
        raise ValueError('record mode must be light, full or off')
    if not run_dir.is_absolute() or '..' in run_dir.parts or run_dir == Path('/'):
        raise ValueError('recording requires a normalized absolute run directory')
    if not isinstance(ros2, str) or not ros2:
        raise ValueError('ROS CLI executable must be nonempty')
    output = str(run_dir / 'bag/data')
    if mode == 'off':
        return RecordingPlan(mode, False, (), output, (), reason='recording explicitly disabled')
    values = profile.get('record', {})
    if not isinstance(values, Mapping) or values.get('storage', 'sqlite3') != 'sqlite3':
        raise ValueError('record settings require sqlite3 storage')
    cache = values.get('max_cache_size_bytes', 104857600)
    if type(cache) is not int or cache < 0:
        raise ValueError('record.max_cache_size_bytes must be a nonnegative integer')
    selected = values.get(mode + '_topics', [])
    if not isinstance(selected, list) or not all(
            isinstance(topic, str) and re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_/]*', topic)
            and '//' not in topic and not topic.endswith('/') for topic in selected):
        raise ValueError('record topic selection must be a list of absolute ROS topic names')
    topics = tuple(dict.fromkeys(selected))
    regex = ''
    extra = []
    if mode == 'full':
        regex = values.get('full_regex', '')
        if not isinstance(regex, str):
            raise ValueError('record.full_regex must be a string')
        try:
            re.compile(regex)
        except re.error as error:
            raise ValueError('record.full_regex is invalid') from error
        for key, flag in (('full_include_unpublished', '--include-unpublished-topics'),
                          ('full_include_hidden', '--include-hidden-topics')):
            enabled = values.get(key, False)
            if type(enabled) is not bool:
                raise ValueError(f'record.{key} must be boolean')
            if enabled:
                extra.append(flag)
        compression = values.get('full_compression', 'none')
        if compression not in ('none', 'zstd'):
            raise ValueError('full compression must be none or zstd')
        if compression == 'zstd':
            extra += ['--compression-mode', 'file', '--compression-format', 'zstd']
        if not topics and not regex:
            raise ValueError('full recording requires an explicit topic or regex selection')
    elif not topics:
        return RecordingPlan(mode, False, (), output, (), reason='empty light topic selection')
    argv = [ros2, 'bag', 'record', '--storage', 'sqlite3',
            '--max-cache-size', str(cache), '--output', output, *extra, *topics]
    if regex:
        argv += ['-e', regex]
    return RecordingPlan(mode, True, tuple(argv), output, topics, regex)
