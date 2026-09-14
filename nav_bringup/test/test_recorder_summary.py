"""Tests for the human readable run summary."""

import datetime

from nav_bringup.recorder import events
from nav_bringup.recorder import resources
from nav_bringup.recorder import summary


def _stamp(hour, minute, second):
    return datetime.datetime(2026, 9, 14, hour, minute, second).timestamp()


MANIFEST = {
    'run_id': '20260914_211342_tour_fe1ac78',
    'mode': 'tour',
    'robot': 'david',
    'site': 'daolan',
    'alias': '4',
    'host': 'ubuntu',
    'started_at': '2026-09-14T21:13:42+08:00',
    'ended_at': '2026-09-14T22:18:45+08:00',
    'duration_sec': 3903.4,
    'exit_code': 130,
    'exit_reason': 'ctrl_c',
    'git': {'commit': 'fe1ac78d0c4b9a2e5f1d3c7b8a6e4f2d1c0b9a87', 'dirty': True},
    'environment': {'ROS_DOMAIN_ID': '0'},
}


def test_render_summary_full_run():
    """Header table, event counts, recoveries, problems and resources are rendered."""
    run_events = [
        events.Event(_stamp(21, 13, 50), 'navrun', 'run_started'),
        events.Event(_stamp(21, 20, 1), 'bt', 'escape_triggered', 'warn',
                     {'node': 'RobotEmbeddedInObstacle', 'x': 14.2}),
        events.Event(_stamp(21, 30, 0), 'executor', 'task_failed', 'error',
                     {'reason': '目标点不存在'}),
    ]
    usage = resources.ResourceSummary(
        system=resources.ProcessStats(resources.SYSTEM_ROW_NAME, 10, 21.5, 48.0, 8123.0),
        processes=[resources.ProcessStats('controller_server', 10, 35.2, 60.0, 81.3)],
    )

    text = summary.render_summary(MANIFEST, run_events, usage, skipped_event_lines=2)

    assert text.startswith('# 运行摘要：20260914_211342_tour_fe1ac78\n')
    assert '| 时长 | 1:05:03 |' in text
    assert '| 退出 | ctrl_c（退出码 130） |' in text
    assert '| 代码 | fe1ac78（有未提交修改） |' in text
    assert '| ROS_DOMAIN_ID | 0 |' in text
    assert '| escape_triggered | warn | 1 |' in text
    assert '另有 2 行无法解析，见 events.jsonl。' in text
    assert '## 脱困与恢复触发（1 次）\n\n按节点：RobotEmbeddedInObstacle 1 次\n' in text
    assert ('- 21:20:01 `escape_triggered` [bt] '
            '{"node": "RobotEmbeddedInObstacle", "x": 14.2}') in text
    assert '## 告警与错误（1 次）' in text
    assert '{"reason": "目标点不存在"}' in text
    assert '整机 CPU 平均 21.5% / 峰值 48.0%，已用内存峰值 8123 MB（10 次采样）。' in text
    assert '| controller_server | 35.2 | 60.0 | 81.3 |' in text
    assert text.endswith('\n')


def test_render_summary_empty_run_uses_placeholders():
    """A run without events or samples still renders every section."""
    text = summary.render_summary({'run_id': 'x'}, [], resources.ResourceSummary())

    assert '| 时长 | - |' in text
    assert '无事件。' in text
    assert '## 脱困与恢复触发（0 次）\n\n无。' in text
    assert '无资源采样。' in text


def test_long_event_lists_are_truncated():
    """Only the first events are listed, with a pointer to the full file."""
    many = [
        events.Event(_stamp(21, 0, index), 'bt', 'recovery_triggered', 'warn', {'index': index})
        for index in range(summary.MAX_LISTED_EVENTS + 3)
    ]

    text = summary.render_summary({'run_id': 'x'}, many, resources.ResourceSummary())

    assert f'## 脱困与恢复触发（{summary.MAX_LISTED_EVENTS + 3} 次）' in text
    assert '- ……另有 3 条，见 events.jsonl' in text


def test_write_summary(tmp_path):
    """The rendered text is written as UTF-8."""
    path = tmp_path / 'summary.md'

    summary.write_summary(path, '# 运行摘要\n')

    assert path.read_text(encoding='utf-8') == '# 运行摘要\n'
