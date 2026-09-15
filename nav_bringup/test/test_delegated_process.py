"""Real exec/session boundaries retain only the explicitly delegated lock capability."""

import os
from pathlib import Path
import socket
import sys
import time

import pytest

from nav_bringup.navrun.locking import LockBusy, RunLock
from nav_bringup.navrun.owned_process import OwnedProcess


def test_child_exec_can_verify_without_unlocking_parent(tmp_path):
    """Kernel fdinfo ownership evidence remains valid after setsid and exec."""
    path = tmp_path / 'lease'
    parent, child = socket.socketpair()
    process = None
    code = (
        'import socket,sys\n'
        'from pathlib import Path\n'
        'from nav_bringup.navrun.delegated_lock import DelegatedLock\n'
        'with DelegatedLock(int(sys.argv[1]), Path(sys.argv[2])):\n'
        '    sock=socket.socket(fileno=int(sys.argv[3]))\n'
        '    sock.sendall(b"R")\n'
        '    assert sock.recv(1)==b"G"\n'
    )
    try:
        with RunLock(path) as lease:
            env = dict(os.environ)
            env['PYTHONPATH'] = str(Path(__file__).resolve().parents[1])
            process = OwnedProcess(
                [sys.executable, '-c', code, str(lease.fileno()), str(path), str(child.fileno())],
                env, tmp_path / 'child.log', pass_fds=(lease.fileno(), child.fileno()))
            child.close()
            parent.settimeout(5)
            assert parent.recv(1) == b'R'
            with pytest.raises(LockBusy):
                RunLock(path).acquire()
            parent.sendall(b'G')
            deadline = time.monotonic() + 5
            while process.poll() is None and time.monotonic() < deadline:
                time.sleep(0.01)
            assert process.poll() == 0, (tmp_path / 'child.log').read_text()
            assert process.close(0.2) == 0
            with pytest.raises(LockBusy):
                RunLock(path).acquire()
    finally:
        if process is not None:
            process.close(0.2)
        parent.close()
        child.close()
