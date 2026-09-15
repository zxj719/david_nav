"""Tests for the run monitor that need no ROS graph."""

from pathlib import Path
import subprocess
import sys

import pytest

from nav_bringup.navrun import cli
from nav_bringup.recorder import monitor
from nav_bringup.recorder import run_dir


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


def test_action_status_topic():
    """Readiness is detected on the status topic that only servers publish."""
    assert monitor.action_status_topic('/navigate_to_pose') == '/navigate_to_pose/_action/status'
    assert monitor.action_status_topic('/ns/dock/') == '/ns/dock/_action/status'


def test_dump_params_command(tmp_path):
    """The parameter dump targets the run directory."""
    run = run_dir.RunDirectory(tmp_path, '20260915_031500_tour_c991ad0')

    assert monitor.dump_params_command(run, python='python3') == [
        'python3', '-m', 'nav_bringup.navrun.cli', 'dump-params', '--run-dir', str(run.path)]


def test_monitor_options_validation():
    """Empty frames, bad periods and intervals, and dumps without readiness are rejected."""
    with pytest.raises(ValueError):
        monitor.MonitorOptions(base_frame='')
    with pytest.raises(ValueError):
        monitor.MonitorOptions(base_frame='robot_base', period_sec=0.0)
    with pytest.raises(ValueError):
        monitor.MonitorOptions(base_frame='robot_base', min_trigger_interval_sec=-1.0)
    with pytest.raises(ValueError):
        monitor.MonitorOptions(base_frame='robot_base', dump_params_on_ready=True)


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

    assert cli.main([
        'monitor', '--base-frame', 'robot_base',
        '--ready-action', '/navigate_to_pose', '--dump-params-on-ready',
    ]) == 0
    assert (calls[2][1].ready_action, calls[2][1].dump_params_on_ready) == (
        '/navigate_to_pose', True)


@pytest.mark.parametrize('argv', [
    ['monitor'],
    ['monitor', '--base-frame', 'robot_base', '--period', '0'],
    ['monitor', '--base-frame', 'robot_base', '--dump-params-on-ready'],
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
