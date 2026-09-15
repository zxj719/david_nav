"""Zero-only installed convenience launch with a test-owned controlling terminal."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import termios
import time

from geometry_msgs.msg import Twist
import pytest
import rclpy
import yaml

from conftest import require_sandbox
from nav_bringup.navrun.locking import RunLock
from nav_bringup.recorder.events import read_events
from nav_bringup.recorder.manifest import read_manifest


@pytest.mark.parametrize('finish', ['quit', 'interrupt', 'ctrl_c', 'conflict', 'missing_tty'])
def test_installed_keyboard_launch_owns_terminal_and_propagates_failure(tmp_path, finish):
    """Actual launch delegates real up with private host hooks and never motion keys."""
    require_sandbox()
    master, slave = os.openpty()
    original = termios.tcgetattr(slave)
    root = tmp_path / 'entry'
    process = None
    rclpy.init(args=[])
    node = rclpy.create_node('keyboard_launch_observer')
    received = []
    node.create_subscription(Twist, '/demo/command', received.append, 10)
    if finish == 'conflict':
        node.create_publisher(Twist, '/demo/command', 10)
    output = (tmp_path / 'wrapper.log').open('wb')
    try:
        helper = Path(__file__).with_name('keyboard_launch_fixture.py')
        command = [sys.executable, str(helper), '--root', str(root)]
        if finish != 'missing_tty':
            command.extend(['--tty-fd', str(slave)])
        process = subprocess.Popen(
            command, stdout=output, stderr=subprocess.STDOUT, start_new_session=True,
            pass_fds=(slave,) if finish != 'missing_tty' else ())
        deadline = time.monotonic() + 25
        while not received and process.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if finish in ('quit', 'interrupt', 'ctrl_c'):
            assert received, (tmp_path / 'wrapper.log').read_text()
            if finish == 'quit':
                os.write(master, b' q')
            elif finish == 'interrupt':
                process.send_signal(signal.SIGINT)
            else:
                # Like real Ctrl-C, the private controlling TTY signals its
                # foreground group, including launch and its navrun child.
                os.write(master, b'\x03')
        else:
            assert not received
        deadline = time.monotonic() + 30
        while process.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert process.poll() == (0 if finish == 'quit' else 1), (
            tmp_path / 'wrapper.log').read_text()
        assert termios.tcgetattr(slave) == original
        assert all(message == Twist() for message in received)
        run = (root / 'runs/latest').resolve()
        assert read_manifest(run / 'manifest.yaml')['exit_code'] == {
            'quit': 0, 'interrupt': 130, 'ctrl_c': 130, 'conflict': 2, 'missing_tty': 2}[finish]
        options = yaml.safe_load((run / 'params/inputs/teleop_options.yaml').read_bytes())
        assert options == {'layout': 'uiojkl', 'linear_speed': 0.1, 'angular_speed': 0.2}
        parameters = yaml.safe_load((run / 'params/rendered/teleop.yaml').read_bytes())
        for key, value in options.items():
            assert parameters['chassis_teleop']['ros__parameters'][key] == value
        events = read_events(run / 'events.jsonl')[0]
        started = [event for event in events if event.event_type == 'process_started']
        assert all(not Path('/proc', str(event.data['pid'])).exists() for event in started)
        if finish == 'missing_tty':
            assert not started
        else:
            assert any(event.event_type == 'terminal_restored' for event in events)
        if finish == 'conflict':
            assert [event.data['name'] for event in started] == ['keyboard']
        if finish not in ('quit', 'interrupt', 'ctrl_c'):
            assert not any(event.event_type == 'stop_zero_sent' for event in events)
        with RunLock(root / 'lease'):
            pass
    finally:
        if process is not None and process.poll() is None:
            # Signal the private foreground group as a last-resort test cleanup;
            # do not terminate launch and close its TTY while navrun is draining.
            if finish != 'missing_tty':
                os.write(master, b'\x03')
            else:
                process.send_signal(signal.SIGINT)
            process.wait(timeout=75)
        output.close()
        node.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)
