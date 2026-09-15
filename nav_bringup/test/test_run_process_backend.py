"""Fake every OS/psutil signal operation; never target a real process."""

from contextlib import nullcontext
from dataclasses import replace
import os
import signal
from types import SimpleNamespace

import psutil
import pytest

from nav_bringup.navrun import process_backend
from nav_bringup.navrun.preflight import PreflightError, ProcessInfo


IDENTITY = ProcessInfo(987654, 1, 10.0, ('fake_executor',), 1000)


class _Process:
    def __init__(self, info=IDENTITY, failure=None):
        self.info = info
        self.pid = info.pid
        self.failure = failure

    def oneshot(self):
        return nullcontext()

    def ppid(self):
        if self.failure is not None:
            raise self.failure
        return self.info.ppid

    def create_time(self):
        return self.info.started_at

    def cmdline(self):
        return self.info.argv

    def uids(self):
        return SimpleNamespace(real=self.info.uid)


def _hooks(monkeypatch, info=IDENTITY):
    events = []
    monkeypatch.setattr(os, 'getuid', lambda: 1000)
    monkeypatch.setattr(os, 'pidfd_open', lambda pid, flags: events.append(('open', pid)) or 999)
    monkeypatch.setattr(os, 'close', lambda fd: events.append(('close', fd)))
    monkeypatch.setattr(psutil, 'Process', lambda pid: events.append(('inspect', pid)) or (
        _Process(info)))
    monkeypatch.setattr(signal, 'pidfd_send_signal',
                        lambda fd, sig, _info, flags: events.append(('signal', fd, sig)))
    monkeypatch.setattr(os, 'kill', lambda *_: pytest.fail('PID-only signaling is forbidden'))
    return events


def test_snapshot_is_sorted_and_omits_only_confirmed_exits(monkeypatch):
    """A process that disappeared during inspection is omitted without losing live peers."""
    exited = _Process(replace(IDENTITY, pid=2), failure=psutil.NoSuchProcess(2))
    monkeypatch.setattr(psutil, 'process_iter', lambda: iter([
        _Process(), exited, _Process(replace(IDENTITY, pid=5))]))
    assert [item.pid for item in process_backend.snapshot_processes()] == [5, 987654]


def test_unreadable_process_rejects_snapshot_instead_of_proving_absence(monkeypatch):
    """Access denial prevents a complete snapshot instead of hiding a possible participant."""
    monkeypatch.setattr(psutil, 'process_iter', lambda: iter([
        _Process(failure=psutil.AccessDenied(987654))]))
    with pytest.raises(PreflightError, match='complete process snapshot'):
        process_backend.snapshot_processes()


def test_pidfd_is_opened_before_inspection_and_always_closed(monkeypatch):
    """The kernel identity is pinned first and the descriptor is closed after sending."""
    events = _hooks(monkeypatch)
    assert process_backend.send_if_same(IDENTITY, signal.SIGTERM)
    assert events == [('open', 987654), ('inspect', 987654),
                      ('signal', 999, signal.SIGTERM), ('close', 999)]


@pytest.mark.parametrize('change', [
    {'started_at': 20.0}, {'uid': 2000}, {'argv': ('different_program',)},
])
def test_changed_identity_is_not_signaled(monkeypatch, change):
    """Start time, executable arguments and real user must all match the planned identity."""
    events = _hooks(monkeypatch, replace(IDENTITY, **change))
    assert not process_backend.send_if_same(IDENTITY, signal.SIGKILL)
    assert not any(event[0] == 'signal' for event in events)
    assert events[-1] == ('close', 999)


def test_reparenting_does_not_change_process_identity(monkeypatch):
    """A surviving child retains its identity when its launcher exits."""
    events = _hooks(monkeypatch, replace(IDENTITY, ppid=200))
    assert process_backend.send_if_same(IDENTITY, signal.SIGTERM)
    assert events[-2][0] == 'signal'


@pytest.mark.parametrize('operation', ['open', 'inspect', 'signal'])
def test_exit_races_never_signal_a_reused_numeric_pid(monkeypatch, operation):
    """Each exit race returns False and never falls back to numeric PID signaling."""
    events = _hooks(monkeypatch)

    def disappeared(*_args):
        if operation == 'inspect':
            raise psutil.NoSuchProcess(IDENTITY.pid)
        raise ProcessLookupError()

    if operation == 'open':
        monkeypatch.setattr(os, 'pidfd_open', disappeared)
    elif operation == 'inspect':
        monkeypatch.setattr(psutil, 'Process', disappeared)
    else:
        monkeypatch.setattr(signal, 'pidfd_send_signal', disappeared)
    assert not process_backend.send_if_same(IDENTITY, signal.SIGTERM)
    if operation != 'open':
        assert events[-1] == ('close', 999)


def test_unavailable_pidfd_does_not_fall_back_to_os_kill(monkeypatch):
    """Platforms without the required API fail before any signal operation."""
    _hooks(monkeypatch)
    monkeypatch.delattr(os, 'pidfd_open')
    with pytest.raises(PreflightError, match='pidfd support'):
        process_backend.send_if_same(IDENTITY, signal.SIGTERM)


def test_signal_permission_error_propagates_and_closes_descriptor(monkeypatch):
    """Permission failure remains visible and cannot leak a pinned descriptor."""
    events = _hooks(monkeypatch)

    def denied(*_args):
        raise PermissionError('not permitted')

    monkeypatch.setattr(signal, 'pidfd_send_signal', denied)
    with pytest.raises(PermissionError):
        process_backend.send_if_same(IDENTITY, signal.SIGTERM)
    assert events[-1] == ('close', 999)


@pytest.mark.parametrize('identity, signum', [
    (replace(IDENTITY, pid=1), signal.SIGTERM),
    (replace(IDENTITY, pid=os.getpid()), signal.SIGTERM),
    (replace(IDENTITY, uid=2000), signal.SIGTERM), (IDENTITY, signal.SIGUSR1),
])
def test_ineligible_direct_calls_fail_before_pidfd_open(monkeypatch, identity, signum):
    """The backend rejects self, init, other users and unapproved signals independently."""
    events = _hooks(monkeypatch)
    with pytest.raises(ValueError):
        process_backend.send_if_same(identity, signum)
    assert events == []
