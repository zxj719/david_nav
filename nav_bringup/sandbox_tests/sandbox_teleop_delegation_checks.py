"""Real inherited keyboard capabilities, only zero/quit input, in isolated domain 99."""

import os
from pathlib import Path
import socket
import sys
import termios
import time

from geometry_msgs.msg import Twist
import pytest
import rclpy

from conftest import require_sandbox
from nav_bringup.navrun.locking import LockBusy, RunLock
from nav_bringup.navrun.owned_process import OwnedProcess


@pytest.mark.parametrize('finish', ['quit', 'owner_lost', 'no_permit', 'conflict'])
def test_delegated_keyboard_preserves_lease_and_terminal(tmp_path, finish):
    """The child cannot arm before permission or unlock its parent's lease on exit."""
    require_sandbox()
    master, slave = os.openpty()
    before = termios.tcgetattr(slave)
    parent, child = socket.socketpair()
    parent.setblocking(False)
    process = None
    rclpy.init(args=[])
    node = rclpy.create_node('delegated_keyboard_observer')
    received = []
    node.create_subscription(Twist, '/sandbox_delegated_keyboard', received.append, 10)
    if finish == 'conflict':
        node.create_publisher(Twist, '/sandbox_delegated_drive', 10)
    path = tmp_path / 'lease'
    try:
        with RunLock(path) as lease:
            helper = Path(__file__).with_name('teleop_delegation_fixture.py')
            args = [sys.executable, str(helper), str(path),
                    '--lock-fd', str(lease.fileno()), '--control-fd', str(child.fileno()),
                    '--tty-fd', str(slave), '--ros-args',
                    '-p', 'cmd_vel_topic:=/sandbox_delegated_keyboard',
                    '-p', 'guard_topics:=[/sandbox_delegated_drive, /sandbox_delegated_nav]']
            process = OwnedProcess(args, os.environ, tmp_path / 'keyboard.log',
                                   pass_fds=(lease.fileno(), child.fileno(), slave))
            child.close()
            deadline = time.monotonic() + 10
            ready = False
            while process.poll() is None and not ready and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                try:
                    ready = parent.recv(1) == b'R'
                except BlockingIOError:
                    pass
            assert not received
            if finish == 'conflict':
                assert process.poll() == 2, (tmp_path / 'keyboard.log').read_text()
                assert not ready
            else:
                assert ready, (tmp_path / 'keyboard.log').read_text()
                for _ in range(4):
                    rclpy.spin_once(node, timeout_sec=0.05)
                assert not received, 'must not publish while waiting for the owner permit'
                if finish == 'no_permit':
                    parent.close()
                else:
                    parent.sendall(b'G')
                    deadline = time.monotonic() + 5
                    while not received and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.05)
                    assert received, (tmp_path / 'keyboard.log').read_text()
                    if finish == 'quit':
                        os.write(master, b' q')
                    else:
                        parent.close()
                deadline = time.monotonic() + 5
                while process.poll() is None and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                assert process.poll() == (0 if finish == 'quit' else 2), (
                    tmp_path / 'keyboard.log').read_text()
            process.close(1)
            assert all(message == Twist() for message in received)
            assert termios.tcgetattr(slave) == before
            with pytest.raises(LockBusy):
                RunLock(path).acquire()
        with RunLock(path):
            pass
    finally:
        if process is not None:
            process.close(1)
        node.destroy_node()
        rclpy.shutdown()
        parent.close()
        child.close()
        os.close(master)
        os.close(slave)
