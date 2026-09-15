"""Exercise keyboard lifecycle on owned PTYs with zero-only isolated ROS output."""

import os
from pathlib import Path
import signal
import sys
import termios
import time

from geometry_msgs.msg import Twist
import pytest
import rclpy

from nav_bringup.navrun.locking import RunLock


@pytest.fixture
def terminal():
    """Provide a private PTY, retaining its original attributes for restoration checks."""
    master, slave = os.openpty()
    original = termios.tcgetattr(slave)
    try:
        yield master, slave, original
    finally:
        os.close(master)
        os.close(slave)


def _start(sessions, tmp_path, tty):
    helper = Path(__file__).with_name('teleop_runtime_fixture.py')
    return sessions.start('keyboard', [
        sys.executable, str(helper), str(tmp_path / 'navrun.lock'), '--ros-args',
        '-p', f'tty_device:={tty}', '-p', 'cmd_vel_topic:=/sandbox_keyboard_output',
        '-p', 'guard_topics:=[/sandbox_keyboard_drive, /sandbox_keyboard_nav]'])


@pytest.mark.parametrize('exit_mode', ['q', 'space_q', 'int', 'term'])
def test_keyboard_exit_restores_terminal_and_releases_lock(
        sessions, tmp_path, terminal, exit_mode):
    """No motion key is sent; successful startup and every exit publish only zero."""
    master, slave, original = terminal
    rclpy.init(args=[])
    observer = rclpy.create_node('keyboard_zero_observer')
    received = []
    observer.create_subscription(Twist, '/sandbox_keyboard_output', received.append, 10)
    try:
        process = _start(sessions, tmp_path, os.ttyname(slave))
        deadline = time.monotonic() + 12.0
        while not received and time.monotonic() < deadline and process.poll() is None:
            rclpy.spin_once(observer, timeout_sec=0.1)
        assert received, (tmp_path / 'process_logs/keyboard.log').read_text()
        before = len(received)
        if exit_mode in ('int', 'term'):
            process.send_signal(signal.SIGINT if exit_mode == 'int' else signal.SIGTERM)
        else:
            os.write(master, b' q' if exit_mode == 'space_q' else b'q')
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and (process.poll() is None or len(received) <= before):
            rclpy.spin_once(observer, timeout_sec=0.1)
        assert process.wait(timeout=1) == 0
        assert len(received) > before
        assert all(message == Twist() for message in received)
        assert termios.tcgetattr(slave) == original
        with RunLock(tmp_path / 'navrun.lock'):
            pass
    finally:
        observer.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('reason', ['publisher', 'lock', 'tty'])
def test_keyboard_rejection_never_publishes(sessions, tmp_path, terminal, reason):
    """A graph conflict, busy private lock or absent TTY cannot stop another publisher."""
    _, slave, original = terminal
    rclpy.init(args=[])
    observer = rclpy.create_node('keyboard_conflict_observer')
    received = []
    observer.create_subscription(Twist, '/sandbox_keyboard_output', received.append, 10)
    lease = None
    try:
        if reason == 'publisher':
            # Discovery endpoint only: this publisher is never used to send a message.
            observer.create_publisher(Twist, '/sandbox_keyboard_drive', 10)
        if reason == 'lock':
            lease = RunLock(tmp_path / 'navrun.lock')
            lease.acquire()
        tty = str(tmp_path / 'absent_tty') if reason == 'tty' else os.ttyname(slave)
        process = _start(sessions, tmp_path, tty)
        deadline = time.monotonic() + 12.0
        while process.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.1)
        assert process.wait(timeout=1) == 2
        for _ in range(5):
            rclpy.spin_once(observer, timeout_sec=0.05)
        assert not received
        assert termios.tcgetattr(slave) == original
    finally:
        if lease is not None:
            lease.release()
        observer.destroy_node()
        rclpy.shutdown()
