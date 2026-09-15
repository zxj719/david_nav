"""Domain-99 startup/stop checks: fake chassis and zero-only velocity publications."""

import os
import json
from pathlib import Path
import signal
import sys
import time

from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
import rclpy
import pytest
import yaml

from conftest import require_sandbox
from nav_bringup.navrun.locking import RunLock
from nav_bringup.navrun.owned_process import OwnedProcess
from nav_bringup.recorder.events import read_events
from nav_bringup.recorder.manifest import read_manifest


def _zero(message):
    return all(value == 0.0 for vector in (message.linear, message.angular)
               for value in (vector.x, vector.y, vector.z))


def test_bound_zero_helper_publishes_only_zero_and_exits(tmp_path):
    """The installed zero module uses real DDS but accepts no nonzero value."""
    require_sandbox()
    rclpy.init(args=[])
    node = rclpy.create_node('sandbox_zero_receiver')
    received = []
    node.create_subscription(Twist, '/sandbox_shutdown_zero', received.append, 10)
    process = None
    try:
        process = OwnedProcess(
            (sys.executable, '-m', 'nav_bringup.navrun.zero_command',
             '--topic', '/sandbox_shutdown_zero'), os.environ, tmp_path / 'zero.log')
        deadline = time.monotonic() + 10
        while (process.poll() is None or len(received) < 5) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert process.poll() == 0, (tmp_path / 'zero.log').read_text()
        assert len(received) == 5 and all(_zero(message) for message in received)
    finally:
        if process is not None:
            process.close(2)
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('backend', ['python', 'cpp'])
def test_recorded_chassis_cli_owns_launch_and_stops_with_zero(tmp_path, backend):
    """Real CLI, launch, adapter, fake driver and recorder finish without real host work."""
    require_sandbox()
    root = tmp_path / 'private_run'
    rclpy.init(args=[])
    node = rclpy.create_node('sandbox_recorded_chassis_receiver')
    odometry, commands = [], []
    diagnostics = []
    node.create_subscription(Odometry, '/demo/wheel_odom', odometry.append, 10)
    node.create_subscription(Twist, '/demo/command', commands.append, 10)
    node.create_subscription(DiagnosticArray, '/diagnostics',
                             lambda message: diagnostics.extend(message.status), 10)
    mode_file = tmp_path / 'chassis_mode.yaml'
    mode_data = {'schema_version': 1, 'mode': 'chassis',
                 'profile': {'chassis': {'adapter': {'implementation': backend}}}}
    mode_file.write_text(yaml.safe_dump(mode_data))
    process = OwnedProcess(
        [sys.executable, str(Path(__file__).with_name('recorded_chassis_fixture.py')),
         '--root', str(root), '--mode-file', str(mode_file)], os.environ, tmp_path / 'wrapper.log')
    try:
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            if odometry and any(status.name == '/chassis_adapter' for status in diagnostics):
                break
            assert process.poll() is None, (tmp_path / 'wrapper.log').read_text()
            rclpy.spin_once(node, timeout_sec=0.1)
        assert odometry, (tmp_path / 'wrapper.log').read_text()
        assert any(status.name == '/chassis_adapter' for status in diagnostics)
        os.kill(process.pid, signal.SIGINT)
        deadline = time.monotonic() + 25
        while process.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert process.poll() == 130, (tmp_path / 'wrapper.log').read_text()
        assert commands and all(_zero(message) for message in commands)
        run = (root / 'runs/latest').resolve()
        assert json.loads((run / 'launch.json').read_text())['options'][
            'adapter_implementation'] == backend
        assert read_manifest(run / 'manifest.yaml')['exit_code'] == 130
        assert (run / 'summary.md').is_file()
        assert (run / 'metrics/resources.csv').is_file()
        events = read_events(run / 'events.jsonl')[0]
        assert sum(event.event_type == 'stop_zero_sent' for event in events) == 2
        assert all(not Path('/proc', str(event.data['pid'])).exists()
                   for event in events if event.event_type == 'process_started')
        with RunLock(root / 'lease'):
            pass
    finally:
        process.close(30)
        node.destroy_node()
        rclpy.shutdown()
