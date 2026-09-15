"""Exercise real kernel locks only on private temporary files and owned children."""

import fcntl
import os
from pathlib import Path
import subprocess
import sys

import pytest

from nav_bringup.navrun.locking import LockBusy, RunLock


def test_competing_lock_preserves_owner_metadata(tmp_path: Path) -> None:
    """A second launch refuses immediately and cannot truncate the first one's record."""
    path = tmp_path / 'navrun.lock'
    with RunLock(path):
        before = path.read_bytes()
        with pytest.raises(LockBusy):
            RunLock(path).acquire()
        assert path.read_bytes() == before
        assert str(os.getpid()).encode() in before
    assert path.is_file()
    with RunLock(path):
        pass


def test_context_exception_releases_lock_without_unlinking(tmp_path: Path) -> None:
    """Failure unwinding preserves one inode while releasing kernel ownership."""
    path = tmp_path / 'navrun.lock'
    with pytest.raises(RuntimeError):
        with RunLock(path):
            inode = path.stat().st_ino
            raise RuntimeError('launch failed')
    with RunLock(path):
        assert path.stat().st_ino == inode


def test_release_is_idempotent_but_double_acquire_is_an_error(tmp_path: Path) -> None:
    """A reused supervisor cannot leak or accidentally replace its own descriptor."""
    lock = RunLock(tmp_path / 'lock')
    lock.release()
    lock.acquire()
    with pytest.raises(RuntimeError, match='already acquired'):
        lock.acquire()
    lock.release()
    lock.release()
    lock.acquire()
    lock.release()


@pytest.mark.parametrize('kind', ['symlink', 'hardlink', 'directory', 'fifo'])
def test_unsafe_lock_files_are_rejected_without_modification(tmp_path: Path, kind: str) -> None:
    """Shared temporary paths cannot redirect lock metadata into another file."""
    path = tmp_path / 'lock'
    original = tmp_path / 'original'
    original.write_bytes(b'user file')
    if kind == 'symlink':
        path.symlink_to(original)
    elif kind == 'hardlink':
        os.link(original, path)
    elif kind == 'directory':
        path.mkdir()
    else:
        os.mkfifo(path)
    with pytest.raises((ValueError, OSError)):
        RunLock(path).acquire()
    assert original.read_bytes() == b'user file'


def test_different_process_cannot_acquire_owned_lock(tmp_path: Path) -> None:
    """The exclusion works between processes, not merely between Python objects."""
    path = tmp_path / 'lock'
    code = (
        'import sys\n'
        'from pathlib import Path\n'
        'from nav_bringup.navrun.locking import LockBusy, RunLock\n'
        'try:\n'
        '    RunLock(Path(sys.argv[1])).acquire()\n'
        'except LockBusy:\n'
        '    sys.exit(23)\n'
        'sys.exit(0)\n')
    with RunLock(path):
        result = subprocess.run([sys.executable, '-c', code, str(path)], timeout=5, check=False)
        assert result.returncode == 23
    result = subprocess.run([sys.executable, '-c', code, str(path)], timeout=5, check=False)
    assert result.returncode == 0


def test_abrupt_exit_releases_kernel_lock_despite_stale_pid(tmp_path: Path) -> None:
    """The next run does not treat old metadata as a live lock owner."""
    path = tmp_path / 'lock'
    code = (
        'import os, sys\n'
        'from pathlib import Path\n'
        'from nav_bringup.navrun.locking import RunLock\n'
        'lock = RunLock(Path(sys.argv[1]))\nlock.acquire()\nos._exit(0)\n')
    result = subprocess.run([sys.executable, '-c', code, str(path)], timeout=5, check=False)
    assert result.returncode == 0
    before = path.read_bytes()
    with RunLock(path):
        assert path.read_bytes() != before


def test_lock_descriptor_is_not_inherited_through_exec(tmp_path: Path) -> None:
    """Launch children cannot keep a crashed supervisor's ownership alive after exec."""
    lock = RunLock(tmp_path / 'lock')
    with lock:
        assert not os.get_inheritable(lock.fileno())
        assert fcntl.fcntl(lock.fileno(), fcntl.F_GETFD) & fcntl.FD_CLOEXEC


def test_fork_child_release_does_not_unlock_parent(tmp_path: Path) -> None:
    """An inherited Python lock object must not issue LOCK_UN on the parent's lease."""
    path = tmp_path / 'lock'
    with RunLock(path) as lock:
        pid = os.fork()
        if pid == 0:
            try:
                lock.release()
            finally:
                os._exit(0)
        _, status = os.waitpid(pid, 0)
        assert os.waitstatus_to_exitcode(status) == 0
        with pytest.raises(LockBusy):
            RunLock(path).acquire()


def test_unacquired_lock_has_no_descriptor(tmp_path: Path) -> None:
    """Callers cannot pass an invalid descriptor to subprocess management."""
    with pytest.raises(RuntimeError, match='not acquired'):
        RunLock(tmp_path / 'lock').fileno()


def test_world_writable_lock_is_rejected(tmp_path: Path) -> None:
    """Another account must not be able to change diagnostic lock contents."""
    path = tmp_path / 'lock'
    path.write_bytes(b'old')
    path.chmod(0o666)
    with pytest.raises(ValueError):
        RunLock(path).acquire()
    assert path.read_bytes() == b'old'


def test_metadata_write_failure_releases_descriptor(tmp_path: Path, monkeypatch) -> None:
    """A disk failure cannot strand an invisible kernel lock in this process."""
    path = tmp_path / 'lock'
    original = os.write

    def fail(*_args):
        raise OSError('full filesystem')

    monkeypatch.setattr(os, 'write', fail)
    with pytest.raises(OSError, match='full filesystem'):
        RunLock(path).acquire()
    monkeypatch.setattr(os, 'write', original)
    with RunLock(path):
        pass
