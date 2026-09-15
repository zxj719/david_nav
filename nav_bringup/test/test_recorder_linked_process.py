"""Tests for linking a separately started process to a run."""

import datetime
import os
from pathlib import Path
import time

import pytest

from nav_bringup.recorder import linked_process
from nav_bringup.recorder import manifest

BOOT_TIME = 1_000_000


def _proc(tmp_path):
    proc = tmp_path / 'proc'
    proc.mkdir()
    (proc / 'stat').write_text(f'cpu 1 2 3\nbtime {BOOT_TIME}\n', encoding='utf-8')
    return proc


def _fake_process(proc, pid, argv, start_ticks=500, cwd=None):
    directory = proc / str(pid)
    directory.mkdir()
    (directory / 'cmdline').write_bytes(b'\0'.join(arg.encode() for arg in argv) + b'\0')
    fields = ['S'] + ['0'] * 18 + [str(start_ticks), '0']
    (directory / 'stat').write_text(
        f'{pid} (odd ) name) ' + ' '.join(fields) + '\n', encoding='utf-8')
    if cwd is not None:
        (directory / 'cwd').symlink_to(cwd)


def test_find_processes_ignores_wrappers(tmp_path):
    """Only processes whose first argument is the executable match."""
    proc = _proc(tmp_path)
    _fake_process(proc, 30, [
        '/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'run', 'loc', 'fake_loc'])
    _fake_process(proc, 31, ['/opt/loc/lib/fake_loc', '--config', '/x.yaml'])
    _fake_process(proc, 12, ['fake_loc'])

    assert linked_process.find_processes(proc, 'fake_loc') == [
        (12, ['fake_loc']),
        (31, ['/opt/loc/lib/fake_loc', '--config', '/x.yaml']),
    ]


def test_process_start_time(tmp_path):
    """Start ticks after boot become local time; unknown processes give None."""
    proc = _proc(tmp_path)
    _fake_process(proc, 40, ['fake_loc'], start_ticks=250)

    assert linked_process.process_start_time(proc, 40, clock_ticks=100) == (
        datetime.datetime.fromtimestamp(BOOT_TIME + 2.5))
    assert linked_process.process_start_time(proc, 41, clock_ticks=100) is None


def test_config_paths():
    """Both flag forms are recognized; a trailing flag without a value is ignored."""
    command = ['loc', '--config', 'a.yaml', '--config=b.yaml', '--other', 'c.yaml', '--config']

    assert linked_process.config_paths(command, ['--config']) == ['a.yaml', 'b.yaml']


@pytest.mark.parametrize('fields', [
    {'name': '', 'executable': 'fake_loc'},
    {'name': 'a/b', 'executable': 'fake_loc'},
    {'name': 'loc', 'executable': ''},
    {'name': 'loc', 'executable': 'fake_loc', 'max_copy_bytes': 0},
])
def test_link_spec_validation(fields):
    """Names are single path components; executable and copy limit must be set."""
    with pytest.raises(ValueError):
        linked_process.LinkSpec(**fields)


def test_snapshot_and_collect_logs(tmp_path):
    """The snapshot records the process; collection copies only what the run added."""
    proc = _proc(tmp_path)
    config_dir = tmp_path / 'config'
    config_dir.mkdir()
    (config_dir / 'loc.yaml').write_text('map: site\n', encoding='utf-8')
    _fake_process(proc, 4242, ['/opt/loc/fake_loc', '--config', 'loc.yaml'], cwd=config_dir)
    _fake_process(proc, 4300, ['/opt/loc/fake_loc'])
    ros_log = tmp_path / 'ros_log'
    launch_log = tmp_path / 'launch_log'
    ros_log.mkdir()
    launch_log.mkdir()
    (ros_log / 'fake_loc_4242_1.log').write_text('boot\n', encoding='utf-8')
    (ros_log / 'fake_loc_9999_1.log').write_text('other process\n', encoding='utf-8')
    (launch_log / 'imu.out').write_text('imu\n', encoding='utf-8')
    (launch_log / 'lidar.out').write_text('lidar started long ago\n', encoding='utf-8')
    spec = linked_process.LinkSpec(
        'loc', 'fake_loc',
        log_globs=(str(ros_log / 'fake_loc_{pid}_*.log'), str(launch_log / '*.out')),
        proc_dir=proc)

    snapshot = linked_process.snapshot(spec, clock_ticks=100)

    config = str(config_dir / 'loc.yaml')
    started = datetime.datetime.fromtimestamp(BOOT_TIME + 5).astimezone()
    assert (snapshot.pid, snapshot.other_pids) == (4242, (4300,))
    assert snapshot.started_at == started.isoformat(timespec='seconds')
    assert snapshot.configs == {config: manifest.sha256_file(config)}
    assert sorted(Path(path).name for path in snapshot.log_sizes) == [
        'fake_loc_4242_1.log', 'imu.out', 'lidar.out']

    since = time.time() - 1.0
    with (ros_log / 'fake_loc_4242_1.log').open('a', encoding='utf-8') as handle:
        handle.write('during run\n')
    (launch_log / 'lidar.out').write_text('restarted\n', encoding='utf-8')
    (launch_log / 'new.out').write_text('created during run\n', encoding='utf-8')
    stale = launch_log / 'stale.out'
    stale.write_text('old\n', encoding='utf-8')
    os.utime(stale, (since - 100, since - 100))
    destination = tmp_path / 'run' / 'logs' / 'loc'

    copies = linked_process.collect_logs(spec, snapshot, destination, since)

    assert sorted(Path(item.destination).name for item in copies) == [
        'fake_loc_4242_1.log', 'lidar.out', 'new.out']
    assert (destination / 'fake_loc_4242_1.log').read_text(encoding='utf-8') == 'during run\n'
    assert (destination / 'lidar.out').read_text(encoding='utf-8') == 'restarted\n'
    assert (destination / 'new.out').read_text(encoding='utf-8') == 'created during run\n'
    assert not any(item.truncated for item in copies)


def test_collect_logs_keeps_the_end_of_large_files(tmp_path):
    """Only the last max_copy_bytes of a file are copied."""
    logs = tmp_path / 'logs'
    logs.mkdir()
    (logs / 'big.out').write_bytes(b'a' * 2000 + b'b' * 1000)
    spec = linked_process.LinkSpec(
        'loc', 'fake_loc', log_globs=(str(logs / '*.out'),), max_copy_bytes=1000,
        proc_dir=tmp_path / 'no_proc')

    copies = linked_process.collect_logs(spec, None, tmp_path / 'copy', since=0.0)

    assert [(item.copied_bytes, item.truncated) for item in copies] == [(1000, True)]
    assert (tmp_path / 'copy' / 'big.out').read_bytes() == b'b' * 1000


def test_without_process_pid_patterns_are_skipped(tmp_path):
    """Without a running process, {pid} patterns are skipped and recent files still copied."""
    logs = tmp_path / 'logs'
    logs.mkdir()
    (logs / 'fake_loc_1_1.log').write_text('pid log\n', encoding='utf-8')
    (logs / 'launch.out').write_text('launch\n', encoding='utf-8')
    spec = linked_process.LinkSpec(
        'loc', 'fake_loc',
        log_globs=(str(logs / 'fake_loc_{pid}_*.log'), str(logs / '*.out')),
        proc_dir=tmp_path / 'no_proc')

    assert linked_process.snapshot(spec) is None
    copies = linked_process.collect_logs(spec, None, tmp_path / 'copy', since=0.0)

    assert [Path(item.destination).name for item in copies] == ['launch.out']


def test_colliding_log_names_get_a_directory_prefix(tmp_path):
    """Files with the same name in different directories do not overwrite each other."""
    for directory in ('first', 'second'):
        (tmp_path / directory).mkdir()
        (tmp_path / directory / 'node.log').write_text(directory + '\n', encoding='utf-8')
    spec = linked_process.LinkSpec(
        'loc', 'fake_loc',
        log_globs=(str(tmp_path / 'first' / '*.log'), str(tmp_path / 'second' / '*.log')),
        proc_dir=tmp_path / 'no_proc')

    copies = linked_process.collect_logs(spec, None, tmp_path / 'copy', since=0.0)

    assert sorted(Path(item.destination).name for item in copies) == [
        'node.log', 'second__node.log']
