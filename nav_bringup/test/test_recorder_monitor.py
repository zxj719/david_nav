"""Tests for the run monitor that need no ROS graph."""

from pathlib import Path
import subprocess
import sys

import pytest

from nav_bringup.navrun import cli
from nav_bringup.recorder import monitor


def test_resolve_message_type():
    """The first advertised type is used; unknown or typeless topics give None."""
    listing = [
        ('/odom', ['nav_msgs/msg/Odometry']),
        ('/empty', []),
        ('/cmd_vel', ['geometry_msgs/msg/Twist', 'other_msgs/msg/Twist']),
    ]

    assert monitor.resolve_message_type('/cmd_vel', listing) == 'geometry_msgs/msg/Twist'
    assert monitor.resolve_message_type('/empty', listing) is None
    assert monitor.resolve_message_type('/scan', listing) is None


def test_monitor_options_validation():
    """Empty frames, non-positive periods and negative intervals are rejected."""
    with pytest.raises(ValueError):
        monitor.MonitorOptions(base_frame='')
    with pytest.raises(ValueError):
        monitor.MonitorOptions(base_frame='robot_base', period_sec=0.0)
    with pytest.raises(ValueError):
        monitor.MonitorOptions(base_frame='robot_base', min_trigger_interval_sec=-1.0)


def test_cli_monitor_builds_options(tmp_path, monkeypatch):
    """Arguments become monitor options; the run directory follows NAVRUN_RUN_DIR."""
    calls = []
    monkeypatch.setattr(monitor, 'spin', lambda run, options: calls.append((run, options)) or 0)
    run_path = tmp_path / '20260915_031500_tour_8991976'
    monkeypatch.setenv('NAVRUN_RUN_DIR', str(run_path))

    assert cli.main([
        'monitor', '--base-frame', 'robot_base',
        '--topic', '/cmd_vel', '--topic', '/odom', '--period', '2.5',
    ]) == 0
    run, options = calls[0]
    assert Path(run.path) == run_path
    assert options == monitor.MonitorOptions(
        base_frame='robot_base', topics=('/cmd_vel', '/odom'), period_sec=2.5)

    monkeypatch.delenv('NAVRUN_RUN_DIR')
    assert cli.main(['monitor', '--base-frame', 'robot_base', '--no-bt']) == 0
    assert calls[1][0] is None
    assert calls[1][1].bt_topic is None


@pytest.mark.parametrize('argv', [
    ['monitor'],
    ['monitor', '--base-frame', 'robot_base', '--period', '0'],
])
def test_cli_monitor_rejects_invalid_arguments(argv, monkeypatch):
    """The base frame is required and options are validated."""
    monkeypatch.setattr(monitor, 'spin', lambda run, options: 0)

    with pytest.raises(SystemExit) as raised:
        cli.main(argv)

    assert raised.value.code == 2


def test_importing_monitor_does_not_import_rclpy():
    """The monitor and the command line stay importable without ROS."""
    code = (
        'import sys\n'
        'import nav_bringup.recorder.monitor\n'
        'import nav_bringup.navrun.cli\n'
        'sys.exit(1 if "rclpy" in sys.modules else 0)\n'
    )

    assert subprocess.run([sys.executable, '-c', code], check=False).returncode == 0
