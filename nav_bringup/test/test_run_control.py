"""Check cooperative local control using private leases and non-ROS owners only."""

import json
import os
import socket
import threading
import time

import pytest

from nav_bringup.navrun import cli
from nav_bringup.navrun.locking import RunLock


def test_status_missing_lease_does_not_create_it(tmp_path, capsys):
    """Absence of a managed owner is not a claim that all navigation is stopped."""
    path = tmp_path / 'lease'
    assert cli.main(['status', '--lock-path', str(path)]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['state'] == 'inactive'
    assert not path.exists()


def test_down_ignores_unlocked_stale_pid(tmp_path, capsys, monkeypatch):
    """Never signal a numeric PID from a stale diagnostic record."""
    path = tmp_path / 'lease'
    with RunLock(path):
        pass
    before = path.read_bytes()
    monkeypatch.setattr(os, 'kill', lambda *args: pytest.fail('OS signal forbidden'))
    assert cli.main(['down', '--lock-path', str(path)]) == 0
    assert json.loads(capsys.readouterr().out)['state'] == 'inactive'
    assert path.read_bytes() == before


@pytest.mark.parametrize('command', ['status', 'down'])
def test_legacy_busy_lease_without_control_is_unknown(tmp_path, capsys, command):
    """A busy old lock grants neither status evidence nor process-control authority."""
    path = tmp_path / 'lease'
    with RunLock(path):
        assert cli.main([command, '--lock-path', str(path), '--timeout', '0.1']) == 2
    assert json.loads(capsys.readouterr().out)['state'] == 'unknown'


@pytest.fixture
def owner(tmp_path):
    """Poll a private cooperative server in one owned test thread."""
    from nav_bringup.navrun.control_server import RunControl

    path = tmp_path / 'lease'
    stopped = threading.Event()
    finished = threading.Event()
    with RunLock(path) as lease, RunControl(lease, stopped.set) as server:
        server.identify('fixture_run', 'chassis')

        def poll():
            while not finished.wait(0.005):
                server.poll()

        thread = threading.Thread(target=poll)
        thread.start()
        try:
            yield path, stopped, server, lease
        finally:
            finished.set()
            thread.join(timeout=2)
            assert not thread.is_alive()


def test_status_proves_current_peer_without_requesting_stop(owner, capsys):
    """A response identifies the live cooperative owner, not merely a manifest."""
    path, stopped, _, _ = owner
    assert cli.main(['status', '--lock-path', str(path)]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['state'] == 'active'
    assert report['pid'] == os.getpid() and report['run_id'] == 'fixture_run'
    assert report['mode'] == 'chassis' and not stopped.is_set()


def test_down_requests_cleanup_but_does_not_claim_completion(owner, capsys, monkeypatch):
    """The server callback is cooperative; a held lease cannot mean cleanup completed."""
    path, stopped, _, _ = owner
    monkeypatch.setattr(os, 'kill', lambda *args: pytest.fail('OS signal forbidden'))
    assert cli.main(['down', '--lock-path', str(path), '--wait-timeout', '0.01']) == 3
    report = json.loads(capsys.readouterr().out)
    assert stopped.is_set() and report['state'] == 'stop_requested'
    assert report['cleanup_complete'] is False


@pytest.mark.parametrize('kind', ['symlink', 'hardlink', 'fifo', 'world_write'])
def test_unsafe_lease_rejected_without_modification(tmp_path, capsys, kind):
    """Control cannot consume aliases or special-file lock contents."""
    path = tmp_path / 'lease'
    other = tmp_path / 'other'
    other.write_text('{}')
    if kind == 'symlink':
        path.symlink_to(other)
    elif kind == 'hardlink':
        os.link(other, path)
    elif kind == 'fifo':
        os.mkfifo(path)
    else:
        path.write_text('{}')
        path.chmod(0o666)
    assert cli.main(['status', '--lock-path', str(path)]) == 2
    assert json.loads(capsys.readouterr().out)['state'] == 'unknown'
    assert other.read_text() == '{}'


@pytest.mark.parametrize('option,value', [
    ('--timeout', '0'), ('--timeout', 'nan'), ('--wait-timeout', '-1')])
def test_invalid_timeouts_rejected_before_contact(owner, capsys, option, value):
    """Bad bounds cannot accidentally turn into an indefinite or immediate stop request."""
    path, stopped, _, _ = owner
    assert cli.main(['down', '--lock-path', str(path), option, value]) == 2
    capsys.readouterr()
    assert not stopped.is_set()


def test_nonce_mismatch_never_requests_shutdown(owner, capsys):
    """Changing diagnostic metadata cannot redirect a request to the wrong incarnation."""
    path, stopped, _, lease = owner
    original = json.loads(path.read_text())['control']
    lease.annotate(dict(original, instance='0' * 32))
    assert cli.main(['down', '--lock-path', str(path), '--timeout', '0.1']) == 2
    capsys.readouterr()
    assert not stopped.is_set()


def test_peer_pid_mismatch_never_requests_shutdown(owner, capsys, monkeypatch):
    """Client verifies kernel peer identity before sending a mutating operation."""
    from nav_bringup.navrun import control_client

    path, stopped, _, _ = owner
    monkeypatch.setattr(control_client, '_peer', lambda sock: (os.getpid() + 1, os.getuid()))
    assert cli.main(['down', '--lock-path', str(path), '--timeout', '0.1']) == 2
    capsys.readouterr()
    assert not stopped.is_set()


def test_close_releases_abstract_endpoint_and_marks_normal_completion(tmp_path):
    """No socket file or persistent listener survives the scoped owner."""
    from nav_bringup.navrun.control_server import RunControl

    path = tmp_path / 'lease'
    with RunLock(path) as lease:
        with RunControl(lease, lambda: None):
            assert json.loads(path.read_text())['control']['finished'] is False
        assert json.loads(path.read_text())['control']['finished'] is True
    assert sorted(item.name for item in tmp_path.iterdir()) == ['lease']


def test_exceptional_owner_exit_does_not_mark_cleanup_completed(tmp_path):
    """Lease release after an exception must not be labelled successful finalization."""
    from nav_bringup.navrun.control_server import RunControl

    path = tmp_path / 'lease'
    with pytest.raises(RuntimeError), RunLock(path) as lease:
        with RunControl(lease, lambda: None):
            raise RuntimeError('fixture failure')
    assert json.loads(path.read_text())['control']['finished'] is False


def test_busy_unresponsive_owner_times_out_without_signal(tmp_path, capsys):
    """A live socket that is not polled grants no permission for a kill fallback."""
    from nav_bringup.navrun.control_server import RunControl

    stopped = []
    path = tmp_path / 'lease'
    with RunLock(path) as lease, RunControl(lease, lambda: stopped.append(True)):
        started = time.monotonic()
        assert cli.main(['down', '--lock-path', str(path), '--timeout', '0.05']) == 2
        assert time.monotonic() - started < 2.0
    capsys.readouterr()
    assert not stopped


@pytest.mark.parametrize('packet', [b'broken', b'{}', b'x' * 2048,
                                    b'{"op":"kill","instance":"wrong"}'])
def test_malformed_clients_do_not_disable_control(owner, capsys, packet):
    """Invalid or oversized packets are isolated before the stop callback."""
    path, stopped, _, _ = owner
    address = json.loads(path.read_text())['control']['address']
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as client:
        client.settimeout(1.0)
        client.connect('\0' + address)
        client.sendall(packet)
        assert client.recv(4096) == b''
    assert cli.main(['status', '--lock-path', str(path)]) == 0
    assert json.loads(capsys.readouterr().out)['state'] == 'active'
    assert not stopped.is_set()


def test_slow_client_does_not_block_other_status_requests(owner, capsys):
    """Accepted connections with no packet never block the existing observer thread."""
    path, stopped, _, _ = owner
    address = json.loads(path.read_text())['control']['address']
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as client:
        client.settimeout(1.0)
        client.connect('\0' + address)
        assert cli.main(['status', '--lock-path', str(path)]) == 0
    assert json.loads(capsys.readouterr().out)['state'] == 'active'
    assert not stopped.is_set()


@pytest.mark.parametrize('transition', ['crash', 'new_owner', 'replacement'])
def test_cleanup_wait_never_claims_completion_for_unproven_transitions(monkeypatch, transition):
    """A released or replaced lease alone is not proof of completed owned-child cleanup."""
    from nav_bringup.navrun import control_client

    response = {'instance': 'original', '_identity': (1, 2),
                'state': 'stop_requested', 'cleanup_complete': False}
    metadata = {'control': {'instance': 'original', 'finished': False}}
    identity = (1, 2)
    held = False
    if transition == 'new_owner':
        held = True
        metadata['control']['instance'] = 'new'
    elif transition == 'replacement':
        identity = (1, 3)
    monkeypatch.setattr(control_client, '_snapshot', lambda _: (held, metadata, identity))
    result = control_client._wait(None, response, 0)
    assert result['state'] == 'unknown' and result['cleanup_complete'] is False


def test_client_interrupt_never_escalates_to_process_signals(tmp_path, capsys, monkeypatch):
    """Canceling a client wait must not force or undo the owner's cleanup policy."""
    from nav_bringup.navrun import control_client

    def interrupt(*args):
        raise KeyboardInterrupt()

    monkeypatch.setattr(control_client, '_exchange', interrupt)
    assert cli.main(['down', '--lock-path', str(tmp_path / 'lease')]) == 130
    assert json.loads(capsys.readouterr().out)['state'] == 'unknown'


def test_run_lock_annotations_require_current_owner_and_path(tmp_path):
    """Neither a released lease nor a replaced inode may publish control discovery."""
    lease = RunLock(tmp_path / 'lease')
    lease.acquire()
    lease.path.rename(tmp_path / 'previous')
    lease.path.write_text('replacement')
    try:
        with pytest.raises(ValueError):
            lease.annotate({'instance': 'not-authorized'})
        assert lease.path.read_text() == 'replacement'
    finally:
        lease.release()
    with pytest.raises(RuntimeError):
        lease.annotate({})
