"""Parent-side capabilities and terminal crash restoration use private PTYs only."""

import os
import socket
import termios
import tty

import pytest

from nav_bringup.navrun.keyboard_session import KeyboardSession
from nav_bringup.navrun.locking import LockBusy, RunLock


def test_session_delegates_and_restores_without_unlocking(tmp_path):
    """Parent cleanup repairs terminal state even when the child never restored it."""
    master, slave = os.openpty()
    original = termios.tcgetattr(slave)
    try:
        with RunLock(tmp_path / 'lease') as lease:
            with KeyboardSession(lease, os.ttyname(slave)) as session:
                argv, fds = session.command(('managed', '--ros-args'))
                assert argv[:2] == ('managed', '--ros-args')
                assert lease.fileno() in fds and len(fds) == 3
                control = int(argv[argv.index('--control-fd') + 1])
                with socket.socket(fileno=os.dup(control)) as child:
                    session.child_started()
                    assert not session.ready()
                    child.sendall(b'R')
                    assert session.ready()
                    session.permit()
                    assert child.recv(1) == b'G'
                    tty.setcbreak(slave)
                    session.revoke()
                    assert child.recv(1) == b''
            assert termios.tcgetattr(slave) == original
            with pytest.raises(LockBusy):
                RunLock(tmp_path / 'lease').acquire()
    finally:
        os.close(master)
        os.close(slave)


def test_missing_tty_cannot_create_channels(tmp_path):
    """A missing terminal fails before the keyboard or chassis could be spawned."""
    with RunLock(tmp_path / 'lease') as lease:
        with pytest.raises(OSError):
            with KeyboardSession(lease, str(tmp_path / 'missing')):
                pytest.fail('missing terminal')


def test_regular_file_is_not_an_interactive_terminal(tmp_path):
    """A profile typo cannot silently run a headless keyboard."""
    data = tmp_path / 'data'
    data.write_text('preserved')
    with RunLock(tmp_path / 'lease') as lease:
        with pytest.raises(ValueError, match='TTY'):
            with KeyboardSession(lease, str(data)):
                pytest.fail('not a terminal')
    assert data.read_text() == 'preserved'
