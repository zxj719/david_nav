"""Tests for the behavior tree log watcher that need no ROS graph."""

import datetime
import io
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

from nav_bringup.navrun import cli
from nav_bringup.recorder import bt_log_watcher
from nav_bringup.recorder import bt_triggers
from nav_bringup.recorder import events


def _entry(node_name, previous, current, sec, nanosec=0):
    return SimpleNamespace(
        node_name=node_name,
        previous_status=previous,
        current_status=current,
        timestamp=SimpleNamespace(sec=sec, nanosec=nanosec),
    )


def test_status_changes_from_msg():
    """Message entries become status changes with float stamps."""
    msg = SimpleNamespace(event_log=[
        _entry('Spin', 'IDLE', 'RUNNING', 100, 250_000_000),
        _entry('Spin', 'RUNNING', 'SUCCESS', 101),
    ])

    assert bt_log_watcher.status_changes_from_msg(msg) == [
        bt_triggers.StatusChange('Spin', 'IDLE', 'RUNNING', 100.25),
        bt_triggers.StatusChange('Spin', 'RUNNING', 'SUCCESS', 101.0),
    ]


def test_reporter_writes_events_and_lines(tmp_path):
    """Each trigger becomes one event and one line in every sink."""
    event_log = events.EventLog(tmp_path / 'events.jsonl', clock=lambda: 500.0)
    terminal = io.StringIO()
    navrun_log = io.StringIO()
    wall_time = datetime.datetime(2026, 9, 14, 21, 40, 3).timestamp()
    reporter = bt_log_watcher.TriggerReporter(
        bt_triggers.TriggerDetector(),
        event_log,
        [terminal, navrun_log],
        pose_provider=lambda: bt_triggers.Pose2D(1.0, 2.0, 0.5),
        task_provider=lambda: 'req_7',
        clock=lambda: wall_time,
    )

    triggers = reporter.handle([
        bt_triggers.StatusChange('RobotEmbeddedInObstacle', 'IDLE', 'SUCCESS', 10.0),
        bt_triggers.StatusChange('FollowPath', 'IDLE', 'RUNNING', 10.1),
    ])

    recorded, skipped = events.read_events(tmp_path / 'events.jsonl')
    assert len(triggers) == 1
    assert skipped == 0
    assert [(e.source, e.event_type, e.severity) for e in recorded] == [
        ('bt_watch', 'escape_triggered', 'warn')]
    assert recorded[0].data['pose'] == {'frame': 'map', 'x': 1.0, 'y': 2.0, 'yaw': 0.5}
    assert recorded[0].data['task_id'] == 'req_7'
    expected = ('[21:40:03] ⚠ 脱困触发：RobotEmbeddedInObstacle 确认嵌入'
                '（第 1 次，task=req_7，位姿 x=1.00 y=2.00 yaw=0.50）\n')
    assert terminal.getvalue() == expected
    assert navrun_log.getvalue() == expected


def test_reporter_without_event_log_only_prints():
    """Without a run directory lines are still printed."""
    terminal = io.StringIO()
    reporter = bt_log_watcher.TriggerReporter(bt_triggers.TriggerDetector(), None, [terminal])

    reporter.handle([bt_triggers.StatusChange('BackUp', 'IDLE', 'RUNNING', 3.0)])

    assert 'BackUp 开始（第 1 次）' in terminal.getvalue()


def test_importing_watcher_does_not_import_rclpy():
    """The watcher module stays importable without a ROS environment."""
    code = (
        'import sys\n'
        'import nav_bringup.recorder.bt_log_watcher\n'
        'import nav_bringup.navrun.cli\n'
        'sys.exit(1 if "rclpy" in sys.modules else 0)\n'
    )

    assert subprocess.run([sys.executable, '-c', code], check=False).returncode == 0


def test_cli_watch_bt_uses_navrun_run_dir(tmp_path, monkeypatch):
    """Without --run-dir the run directory exported by navrun wrap is used."""
    calls = []

    def fake_spin(run, **kwargs):
        calls.append((run, kwargs))
        return 0

    monkeypatch.setattr(bt_log_watcher, 'spin', fake_spin)
    run_path = tmp_path / '20260914_214003_tour_30f56d7'
    monkeypatch.setenv('NAVRUN_RUN_DIR', str(run_path))

    assert cli.main(['watch-bt', '--base-frame', 'base_link', '--min-interval', '1.5']) == 0
    run, kwargs = calls[0]
    assert Path(run.path) == run_path
    assert kwargs == {
        'topic': '/behavior_tree_log',
        'map_frame': 'map',
        'base_frame': 'base_link',
        'min_interval_sec': 1.5,
    }

    monkeypatch.delenv('NAVRUN_RUN_DIR')
    assert cli.main(['watch-bt']) == 0
    assert calls[1][0] is None
    assert calls[1][1]['base_frame'] == 'base_footprint'
