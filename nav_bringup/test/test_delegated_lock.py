"""Kernel-backed lock delegation uses only private, test-owned lock files."""

import fcntl
import os

import pytest

from nav_bringup.navrun.delegated_lock import DelegatedLock
from nav_bringup.navrun.locking import LockBusy, RunLock


def test_borrowed_close_never_unlocks_the_parent(tmp_path):
    """A duplicate open-file description is authority, unlike diagnostic PID text."""
    path = tmp_path / 'lease'
    with RunLock(path) as parent:
        with DelegatedLock(parent.fileno(), path) as child:
            assert child.fileno() != parent.fileno()
            assert not os.get_inheritable(child.fileno())
            with pytest.raises(LockBusy):
                RunLock(path).acquire()
        with pytest.raises(LockBusy):
            RunLock(path).acquire()
    with RunLock(path):
        pass


def test_separate_open_of_locked_inode_is_not_delegation(tmp_path):
    """Knowing the lock path or PID cannot bypass the parent flock."""
    path = tmp_path / 'lease'
    with RunLock(path), path.open('rb') as different:
        with pytest.raises(ValueError, match='description'):
            with DelegatedLock(different.fileno(), path):
                pytest.fail('independent descriptor must be rejected')
        with pytest.raises(LockBusy):
            RunLock(path).acquire()


def test_unlocked_descriptor_cannot_be_promoted_to_authority(tmp_path):
    """Validation cannot acquire a previously unheld lease as a side effect."""
    path = tmp_path / 'lease'
    with RunLock(path):
        pass
    with path.open('rb') as source:
        with pytest.raises(ValueError, match='held'):
            with DelegatedLock(source.fileno(), path):
                pytest.fail('unheld lease must be rejected')
        with RunLock(path):
            pass


def test_wrong_inode_and_symlink_rejected_without_unlock(tmp_path):
    """A locked unrelated file and an alias cannot impersonate the selected lease."""
    expected, other = tmp_path / 'expected', tmp_path / 'other'
    with RunLock(expected), RunLock(other) as lease:
        for selected in (expected, tmp_path / 'alias'):
            if selected.name == 'alias':
                selected.symlink_to(other)
            with pytest.raises((ValueError, OSError)):
                with DelegatedLock(lease.fileno(), selected):
                    pytest.fail('wrong identity')
        with pytest.raises(LockBusy):
            RunLock(other).acquire()


@pytest.mark.parametrize('descriptor', [-1, True, '3', 1])
def test_invalid_descriptor_rejected_before_duplication(tmp_path, descriptor):
    """Standard streams and noninteger selectors cannot become lease capabilities."""
    with pytest.raises(ValueError):
        DelegatedLock(descriptor, tmp_path / 'lease')


def test_parent_release_invalidates_future_checks(tmp_path):
    """The child can detect an explicit parent unlock before it arms output."""
    path = tmp_path / 'lease'
    parent = RunLock(path)
    parent.acquire()
    with DelegatedLock(parent.fileno(), path) as child:
        parent.release()
        with pytest.raises(ValueError):
            child.verify()
    with RunLock(path):
        pass


def test_wrong_shared_mode_cannot_be_upgraded(tmp_path):
    """A shared reader lock is not exclusive ownership and remains shared on rejection."""
    path = tmp_path / 'lease'
    with RunLock(path):
        pass
    with path.open('rb') as source:
        fcntl.flock(source, fcntl.LOCK_SH)
        with pytest.raises(ValueError):
            with DelegatedLock(source.fileno(), path):
                pytest.fail('shared flock must not be upgraded')
        with path.open('rb') as peer:
            fcntl.flock(peer, fcntl.LOCK_SH | fcntl.LOCK_NB)
