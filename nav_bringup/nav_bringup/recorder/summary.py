"""Human readable ``summary.md`` for a finished run.

The summary is written for the people operating the robot, so headings and
labels are in Chinese; values come straight from the manifest, the event
timeline and the resource samples (refactor plan §13.2).
"""

from __future__ import annotations

from collections import Counter
import datetime
import json
from pathlib import Path
from typing import Any, List, Mapping, Optional, Sequence, Union

from nav_bringup.recorder.events import Event
from nav_bringup.recorder.resources import ResourceSummary

RECOVERY_EVENT_TYPES = ('escape_triggered', 'recovery_triggered')
MAX_LISTED_EVENTS = 20
MAX_LISTED_PROCESSES = 10

_MAX_DATA_CHARS = 160


def render_summary(
    manifest: Mapping[str, Any],
    events: Sequence[Event],
    resources: ResourceSummary,
    skipped_event_lines: int = 0,
) -> str:
    """Render the Markdown summary of a run.

    Args:
        manifest: Finalized run manifest.
        events: Events of the run in file order.
        resources: Aggregated resource usage.
        skipped_event_lines: Lines of ``events.jsonl`` that could not be parsed.

    Returns:
        Markdown text ending with a newline.
    """
    git = _mapping(manifest.get('git'))
    environment = _mapping(manifest.get('environment'))
    commit = git.get('commit')
    commit_text = '-'
    if commit:
        commit_text = str(commit)[:7] + ('（有未提交修改）' if git.get('dirty') else '')

    lines: List[str] = [
        f"# 运行摘要：{_text(manifest.get('run_id'))}",
        '',
        '| 项 | 值 |',
        '|---|---|',
        f"| 模式 | {_text(manifest.get('mode'))} |",
        f"| 机器人 / 场地 | {_text(manifest.get('robot'))} / {_text(manifest.get('site'))} |",
        f"| 启动方式 | {_text(manifest.get('alias'))} |",
        f"| 主机 | {_text(manifest.get('host'))} |",
        f"| 开始 | {_text(manifest.get('started_at'))} |",
        f"| 结束 | {_text(manifest.get('ended_at'))} |",
        f"| 时长 | {_duration(manifest.get('duration_sec'))} |",
        f"| 退出 | {_text(manifest.get('exit_reason'))}（退出码 {_text(manifest.get('exit_code'))}） |",
        f'| 代码 | {commit_text} |',
        f"| ROS_DOMAIN_ID | {_text(environment.get('ROS_DOMAIN_ID'))} |",
        '',
    ]
    lines.extend(_event_counts(events, skipped_event_lines))
    recoveries = [event for event in events if event.event_type in RECOVERY_EVENT_TYPES]
    problems = [
        event for event in events
        if event.severity != 'info' and event.event_type not in RECOVERY_EVENT_TYPES
    ]
    lines.extend(_listed_events('脱困与恢复触发', recoveries))
    lines.extend(_listed_events('告警与错误', problems))
    lines.extend(_resources(resources))
    return '\n'.join(lines).rstrip('\n') + '\n'


def write_summary(path: Union[str, Path], text: str) -> None:
    """Write a rendered summary.

    Args:
        path: Destination, normally the run's ``summary.md``.
        text: Output of :func:`render_summary`.
    """
    Path(path).write_text(text, encoding='utf-8')


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _text(value: Any) -> str:
    return '-' if value is None or value == '' else str(value)


def _duration(seconds: Any) -> str:
    if not isinstance(seconds, (int, float)):
        return '-'
    total = int(round(seconds))
    hours, remainder = divmod(total, 3600)
    minutes, secs = divmod(remainder, 60)
    return f'{hours}:{minutes:02d}:{secs:02d}'


def _clock_time(stamp: float) -> str:
    return datetime.datetime.fromtimestamp(stamp).strftime('%H:%M:%S')


def _event_counts(events: Sequence[Event], skipped: int) -> List[str]:
    lines = ['## 事件统计', '']
    if not events:
        lines.append('无事件。')
    else:
        counts = Counter((event.event_type, event.severity) for event in events)
        lines.extend(['| 类型 | 级别 | 次数 |', '|---|---|---|'])
        for (event_type, severity), count in sorted(
                counts.items(), key=lambda item: (-item[1], item[0])):
            lines.append(f'| {event_type} | {severity} | {count} |')
    if skipped:
        lines.extend(['', f'另有 {skipped} 行无法解析，见 events.jsonl。'])
    lines.append('')
    return lines


def _listed_events(title: str, events: Sequence[Event]) -> List[str]:
    lines = [f'## {title}（{len(events)} 次）', '']
    if not events:
        lines.append('无。')
    for event in events[:MAX_LISTED_EVENTS]:
        data = json.dumps(dict(event.data), ensure_ascii=False)
        if len(data) > _MAX_DATA_CHARS:
            data = data[:_MAX_DATA_CHARS] + '…'
        lines.append(
            f'- {_clock_time(event.stamp)} `{event.event_type}` [{event.source}] {data}')
    if len(events) > MAX_LISTED_EVENTS:
        lines.append(f'- ……另有 {len(events) - MAX_LISTED_EVENTS} 条，见 events.jsonl')
    lines.append('')
    return lines


def _resources(resources: ResourceSummary) -> List[str]:
    lines = ['## 资源占用', '']
    system: Optional[Any] = resources.system
    if system is None and not resources.processes:
        lines.extend(['无资源采样。', ''])
        return lines
    if system is not None:
        lines.append(
            f'整机 CPU 平均 {system.cpu_avg:.1f}% / 峰值 {system.cpu_peak:.1f}%，'
            f'已用内存峰值 {system.rss_peak_mb:.0f} MB（{system.samples} 次采样）。')
        lines.append('')
    if resources.processes:
        lines.extend([
            '进程 CPU 以单核 100% 计：',
            '',
            '| 进程 | 平均 CPU% | 峰值 CPU% | 峰值内存 MB |',
            '|---|---|---|---|',
        ])
        for stats in resources.processes[:MAX_LISTED_PROCESSES]:
            lines.append(
                f'| {stats.name} | {stats.cpu_avg:.1f} | {stats.cpu_peak:.1f} | '
                f'{stats.rss_peak_mb:.1f} |')
        lines.append('')
    return lines
