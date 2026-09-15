"""Recorded combined teleop with fake chassis and private zero-only terminal input."""

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

from conftest import require_sandbox
from nav_bringup.navrun.locking import RunLock
from nav_bringup.recorder.events import read_events
from nav_bringup.recorder.manifest import read_manifest


@pytest.mark.parametrize('finish', ['quit', 'interrupt', 'keyboard_killed', 'conflict'])
def test_combined_teleop_records_and_restores_terminal(tmp_path, finish):
    """The real up path owns keyboard and launch separately, never physical hardware."""
    require_sandbox()
    master, slave = os.openpty()
    original = termios.tcgetattr(slave)
    root = tmp_path / 'combined'
    process = None
    rclpy.init(args=[])
    node = rclpy.create_node('combined_teleop_observer')
    received = []
    node.create_subscription(Twist, '/demo/command', received.append, 10)
    if finish == 'conflict':
        node.create_publisher(Twist, '/demo/command', 10)
    output = (tmp_path / 'wrapper.log').open('wb')
    try:
        helper = Path(__file__).with_name('recorded_chassis_fixture.py')
        process = subprocess.Popen(
            [sys.executable, str(helper), '--root', str(root), '--mode', 'teleop',
             '--tty', os.ttyname(slave)], stdout=output, stderr=subprocess.STDOUT,
            start_new_session=True)
        deadline = time.monotonic() + 20
        while not received and process.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        run = (root / 'runs/latest').resolve()
        if finish == 'conflict':
            assert not received
        else:
            assert received, (tmp_path / 'wrapper.log').read_text()
            if finish == 'quit':
                os.write(master, b' q')
            elif finish == 'interrupt':
                process.send_signal(signal.SIGINT)
            else:
                events = read_events(run / 'events.jsonl')[0]
                pid = next(event.data['pid'] for event in events
                           if event.event_type == 'process_started'
                           and event.data['name'] == 'keyboard')
                descriptor = os.pidfd_open(pid)
                try:
                    fields = Path('/proc', str(pid), 'stat').read_text().rsplit(')', 1)[1].split()
                    assert int(fields[1]) == process.pid
                    signal.pidfd_send_signal(descriptor, signal.SIGKILL)
                finally:
                    os.close(descriptor)
        deadline = time.monotonic() + 25
        while process.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        expected = {'quit': 0, 'interrupt': 130, 'keyboard_killed': 137, 'conflict': 2}[finish]
        assert process.poll() == expected, (tmp_path / 'wrapper.log').read_text()
        assert all(message == Twist() for message in received)
        assert termios.tcgetattr(slave) == original
        assert read_manifest(run / 'manifest.yaml')['exit_code'] == expected
        events = read_events(run / 'events.jsonl')[0]
        started = [event for event in events if event.event_type == 'process_started']
        assert all(not Path('/proc', str(event.data['pid'])).exists() for event in started)
        assert any(event.event_type == 'terminal_restored' for event in events)
        if finish == 'conflict':
            assert [event.data['name'] for event in started] == ['keyboard']
            assert not any(event.event_type == 'stop_zero_sent' for event in events)
        else:
            guard = next(i for i, event in enumerate(events)
                         if event.event_type == 'keyboard_guard_ready')
            launch = next(i for i, event in enumerate(events)
                          if event.event_type == 'process_started'
                          and event.data['name'] == 'launch')
            assert guard < launch
            assert sum(event.event_type == 'stop_zero_sent' for event in events) == 2
        with RunLock(root / 'lease'):
            pass
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            process.wait(timeout=30)
        output.close()
        node.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)
