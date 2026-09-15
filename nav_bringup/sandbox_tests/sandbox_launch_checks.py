"""Check actual launch restart/exit ownership without nodes, topics or hardware."""

from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

from ament_index_python.packages import get_package_prefix
import pytest
import yaml

from nav_bringup.launching.graph import GraphOptions


@pytest.mark.parametrize('scenario', ['restart', 'shutdown', 'critical'])
def test_launch_owned_processes(sessions, tmp_path: Path, scenario: str) -> None:
    """Only the fixture's process lineage starts and exits during each scenario."""
    fixture = Path(__file__).with_name('launch_fixture.py')
    process = sessions.start(f'launch_{scenario}', [sys.executable, str(fixture), scenario])
    code = process.wait(timeout=15)
    log = (tmp_path / 'process_logs' / f'launch_{scenario}.log').read_text()
    assert code == 0, log
    assert f'ownership {scenario}:' in log and 'passed=True' in log


def test_ready_gate_times_out_without_executing_command(sessions, tmp_path: Path) -> None:
    """The installed gate cannot execute its child when no action server appears."""
    executable = Path(get_package_prefix('nav_bringup')) / 'lib/nav_bringup/nav_ready_exec'
    process = sessions.start('gate_timeout', [
        str(executable), '--timeout', '0.2', '--action', '/absent_gate_fixture', '--',
        sys.executable, '-c', 'print("UNEXPECTED_CHILD")'])
    code = process.wait(timeout=10)
    log = (tmp_path / 'process_logs/gate_timeout.log').read_text()
    assert code == 1, log
    assert 'executor was not started' in log and 'UNEXPECTED_CHILD' not in log


def test_ready_gate_discovers_fake_server_without_sending_a_goal(sessions, tmp_path: Path) -> None:
    """Only discovery occurs before replacing the gate with a harmless print process."""
    source = Path(__file__).resolve().parents[2]
    server_script = source / 'david_nav/sandbox_tests/fake_nav2_servers.py'
    goal_log = tmp_path / 'goals.log'
    server = sessions.start('gate_server', [
        sys.executable, str(server_script), '--log', str(goal_log), '--x', '0',
        '--y', '0', '--yaw', '0', '--map-mode', 'none'])
    executable = Path(get_package_prefix('nav_bringup')) / 'lib/nav_bringup/nav_ready_exec'
    gate = sessions.start('gate_ready', [
        str(executable), '--timeout', '10', '--', sys.executable,
        '-c', 'print("harmless_executor_started")'])
    code = gate.wait(timeout=15)
    log = (tmp_path / 'process_logs/gate_ready.log').read_text()
    assert code == 0 and 'harmless_executor_started' in log, log
    assert server.poll() is None
    assert not goal_log.exists(), 'readiness checking must never submit a goal'


@pytest.mark.parametrize('entry', ['nav_bringup', 'chassis_adapter'])
def test_installed_launch_entry_runs_only_a_zero_motion_fake_chassis(
        sessions, tmp_path: Path, entry: str) -> None:
    """Exercise both installed launch entries with an adapter and zero-motion driver."""
    from nav_msgs.msg import Odometry
    import rclpy

    root = tmp_path / 'prepared'
    rendered = root / 'params/rendered'
    rendered.mkdir(parents=True)
    options = GraphOptions(
        'chassis', str(root), driver_package='nav_sim', driver_executable='fake_chassis',
        chassis_command_topic='/entry_fixture/command', chassis_odom_topic='/entry_fixture/odom')
    values = asdict(options)
    values.pop('run_dir')
    (root / 'launch.json').write_text(json.dumps({
        'schema_version': 1, 'robot': 'example', 'site': 'offline', 'options': values}))
    (rendered / 'driver.yaml').write_text(yaml.safe_dump({'wheeltec_robot': {'ros__parameters': {
        'odom_frame': 'entry_encoder', 'body_frame': 'entry_body',
        'cmd_topic': '/entry_fixture/command', 'odom_topic': '/entry_fixture/odom'}}}))
    (rendered / 'adapter.yaml').write_text(yaml.safe_dump({
        'chassis_adapter': {'ros__parameters': {
            'input_cmd_vel_topic': '/entry_fixture/no_input',
            'output_cmd_vel_topic': '/entry_fixture/command'}}}))
    filename = 'nav.launch.py' if entry == 'nav_bringup' else 'chassis.launch.py'
    process = sessions.start('installed_entry', [
        'ros2', 'launch', entry, filename, f'run_dir:={root}', 'robot:=example'])
    rclpy.init(args=[])
    observer = rclpy.create_node('launch_entry_observer')
    received = []
    subscription = observer.create_subscription(
        Odometry, '/entry_fixture/odom', received.append, 10)
    deadline = time.monotonic() + 10.0
    try:
        while not received and time.monotonic() < deadline and process.poll() is None:
            rclpy.spin_once(observer, timeout_sec=0.1)
        log = (tmp_path / 'process_logs/installed_entry.log').read_text()
        assert received, log
        assert received[0].header.frame_id == 'entry_encoder'
        assert received[0].pose.pose.position.x == 0.0
        assert received[0].twist.twist.linear.x == 0.0
        assert received[0].twist.twist.angular.z == 0.0
    finally:
        observer.destroy_subscription(subscription)
        observer.destroy_node()
        rclpy.shutdown()
    assert sessions.stop(process) == 0
