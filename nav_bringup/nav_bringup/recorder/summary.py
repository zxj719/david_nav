"""为一次已结束的运行生成可读的 ``summary.md``.

这份摘要写给现场操作机器人的人，因此标题与标签都是中文；数值全部直接取自清单、
事件时间线与资源采样（方案 §13.2），本模块不做任何推断或补全——
摘要里没写的，就是这次运行没有记录到的。
"""

from __future__ import annotations

from collections import Counter
import datetime
import json
from pathlib import Path
from typing import Any, List, Mapping, Optional, Sequence, Union

from nav_bringup.recorder.events import Event
from nav_bringup.recorder.resources import ResourceSummary
from nav_bringup.recorder.topic_stats import TopicsSummary

RECOVERY_EVENT_TYPES = ('escape_triggered', 'recovery_triggered')
MAX_LISTED_EVENTS = 20
MAX_LISTED_PROCESSES = 10

_MAX_DATA_CHARS = 160


def render_summary(
    manifest: Mapping[str, Any],
    events: Sequence[Event],
    resources: ResourceSummary,
    skipped_event_lines: int = 0,
    topics: Optional[TopicsSummary] = None,
) -> str:
    """渲染一次运行的 Markdown 摘要.

    纯函数：只读入参，不读文件、不写文件。

    Args:
        manifest: 已收尾的运行清单。
        events: 按文件顺序排列的本次运行事件。
        resources: 聚合后的资源占用。
        skipped_event_lines: ``events.jsonl`` 中解析失败的行；会如实计入摘要，
            不静默丢弃。
        topics: 话题频率与 TF 延迟的聚合结果；未开监控时可缺省。

    Returns:
        以换行结尾的 Markdown 文本。
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
    lines.extend(_listed_events('脱困与恢复触发', recoveries, by_node=True))
    lines.extend(_listed_events('告警与错误', problems))
    lines.extend(_topics(topics))
    lines.extend(_resources(resources))
    return '\n'.join(lines).rstrip('\n') + '\n'


def write_summary(path: Union[str, Path], text: str) -> None:
    """写出已渲染的摘要.

    直接覆盖目标路径，不做备份。

    Args:
        path: 目标路径，通常是该次运行的 ``summary.md``。
        text: :func:`render_summary` 的输出。
    """
    Path(path).write_text(text, encoding='utf-8')


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _text(value: Any) -> str:
    return '-' if value is None or value == '' else str(value)


def _duration(seconds: Any) -> str:
    """把秒数舍入到整数秒并格式化为时长，非数值返回占位符.

    沿用 Python round 的半偶数舍入，不另行限制负数或布尔值。

    Args:
        seconds: 记录中的秒数；期望为可舍入的有限数值。

    Returns:
        小时:两位分钟:两位秒文本，或非数值输入对应的横线。

    Raises:
        ValueError: NaN 等值不能转换为整数秒。
        OverflowError: 无穷等值不能转换为整数秒。
    """
    if not isinstance(seconds, (int, float)):
        return '-'
    total = int(round(seconds))
    hours, remainder = divmod(total, 3600)
    minutes, secs = divmod(remainder, 60)
    return f'{hours}:{minutes:02d}:{secs:02d}'


def _clock_time(stamp: float) -> str:
    """按系统本地时区显示墙钟时刻，超出平台日期范围时保留数值说明.

    Args:
        stamp: Unix 时间戳，单位秒。

    Returns:
        HH:MM:SS 文本；日期转换失败时返回原数值和超范围标记。
    """
    try:
        return datetime.datetime.fromtimestamp(stamp).strftime('%H:%M:%S')
    except (OverflowError, OSError, ValueError):
        return f'{stamp:g}（超出日期范围）'


def _event_counts(events: Sequence[Event], skipped: int) -> List[str]:
    """生成按次数降序排列的事件计数段，保留无法解析行的提示.

    Args:
        events: 已解析事件序列。
        skipped: 上游跳过的坏行数量。

    Returns:
        Markdown 行列表，包含类型/等级计数和原始日志查阅提示。
    """
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


def _listed_events(title: str, events: Sequence[Event], by_node: bool = False) -> List[str]:
    """生成有数量和负载长度上限的事件明细，避免摘要无限增长.

    Args:
        title: 摘要小节标题。
        events: 已筛选、保持输入顺序的事件。
        by_node: 是否先附按节点统计的次数排行。

    Returns:
        Markdown 行列表；截断只影响展示，完整事件仍留原始日志。
    """
    lines = [f'## {title}（{len(events)} 次）', '']
    if not events:
        lines.append('无。')
    elif by_node:
        counts = Counter(str(event.data.get('node', '-')) for event in events)
        ranked = sorted(counts.items(), key=lambda item: (-item[1], item[0]))
        lines.extend(['按节点：' + '，'.join(f'{name} {count} 次' for name, count in ranked), ''])
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
    """把现有资源汇总渲染为整机与进程占用段，不重新采样.

    Args:
        resources: 已计算的系统和进程 CPU/内存统计。

    Returns:
        Markdown 行列表；进程 CPU 以单核 100% 标示，只展示有限数量进程。
        空样本明确写无采样，不以零值冒充实际测量。
    """
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


def _topics(topics: Optional[TopicsSummary]) -> List[str]:
    """渲染话题频率与 TF 延迟统计，显式保留缺样和无发布者提示.

    Args:
        topics: 已有话题/变换汇总或 None；频率为 Hz，TF 延迟字段为毫秒。

    Returns:
        Markdown 行列表；无观测时写明无采样，不推断正式 ROS 栈状态。
    """
    lines = ['## 话题频率与 TF 延迟', '']
    if topics is None or (not topics.topics and not topics.transforms):
        lines.extend(['无话题采样。', ''])
        return lines
    if topics.topics:
        lines.extend([
            '| 话题 | 平均 Hz | 最低 Hz | 无消息周期 | 最多发布者 |',
            '|---|---|---|---|---|',
        ])
        for stats in topics.topics:
            publishers = '-' if stats.publishers_max is None else str(stats.publishers_max)
            lines.append(
                f'| {stats.name} | {stats.rate_avg_hz:.1f} | {stats.rate_min_hz:.1f} | '
                f'{stats.silent_samples}/{stats.samples} | {publishers} |')
        lines.append('')
        orphans = [stats.name for stats in topics.topics if stats.publishers_max == 0]
        if orphans:
            lines.extend(['⚠ 整个运行期间没有发布者的话题：' + '，'.join(orphans), ''])
    if topics.transforms:
        lines.extend([
            '| 变换 | P50 ms | P95 ms | 最大 ms | 不可用周期 |',
            '|---|---|---|---|---|',
        ])
        for stats in topics.transforms:
            lines.append(
                f'| {stats.name} | {_milliseconds(stats.lag_p50_ms)} | '
                f'{_milliseconds(stats.lag_p95_ms)} | {_milliseconds(stats.lag_max_ms)} | '
                f'{stats.missing_samples}/{stats.samples} |')
        lines.append('')
    return lines


def _milliseconds(value: Optional[float]) -> str:
    return '-' if value is None else f'{value:.1f}'
