"""Tests for the read-only doctor checks."""

import json
import os
from pathlib import Path

import pytest

from nav_bringup.doctor import checks
from nav_bringup.navrun import cli


def _context(tmp_path, **overrides):
    values = {
        'repo_dir': tmp_path / 'src',
        'install_dir': tmp_path / 'install',
        'environ': {},
        'shm_dir': tmp_path / 'shm',
        'proc_dir': tmp_path / 'proc',
        'serial_device': tmp_path / 'wheeltec_controller',
    }
    values.update(overrides)
    return checks.DoctorContext(**values)


def _process(proc_dir, pid, argv, maps=''):
    directory = proc_dir / str(pid)
    directory.mkdir(parents=True)
    (directory / 'cmdline').write_bytes(b'\0'.join(arg.encode() for arg in argv) + b'\0')
    (directory / 'maps').write_text(maps, encoding='utf-8')


def test_ros_environment_fails_without_ros(tmp_path):
    """A shell without ROS is a failure."""
    result = checks.check_ros_environment(_context(tmp_path))

    assert result.status is checks.Status.FAIL


def test_ros_environment_warns_without_dds_profile(tmp_path):
    """Missing domain id and DDS profile are warnings with a hint."""
    result = checks.check_ros_environment(_context(tmp_path, environ={'ROS_DISTRO': 'humble'}))

    assert result.status is checks.Status.WARN
    assert 'ROS_DOMAIN_ID' in result.message
    assert 'FASTRTPS_DEFAULT_PROFILES_FILE' in result.message
    assert 'ros_env.conf' in result.hint


def test_ros_environment_warns_for_missing_profile_file(tmp_path):
    """A profile variable pointing at nothing is a warning."""
    environ = {
        'ROS_DISTRO': 'humble', 'ROS_DOMAIN_ID': '0',
        'FASTRTPS_DEFAULT_PROFILES_FILE': str(tmp_path / 'missing.xml'),
    }

    result = checks.check_ros_environment(_context(tmp_path, environ=environ))

    assert result.status is checks.Status.WARN
    assert '不存在' in result.message


def test_ros_environment_ok(tmp_path):
    """A fully loaded environment passes."""
    profile = tmp_path / 'fastdds.xml'
    profile.write_text('<profiles/>', encoding='utf-8')
    environ = {
        'ROS_DISTRO': 'humble',
        'ROS_DOMAIN_ID': '0',
        'FASTRTPS_DEFAULT_PROFILES_FILE': str(profile),
    }

    assert checks.check_ros_environment(_context(tmp_path, environ=environ)).status is (
        checks.Status.OK)


def test_dds_shared_memory_counts_orphans(tmp_path):
    """Segments not mapped by any process are orphans; _el files follow their base."""
    shm = tmp_path / 'shm'
    shm.mkdir()
    for name in ('fastrtps_aaa', 'fastrtps_aaa_el', 'fastrtps_bbb', 'fastrtps_bbb_el', 'other'):
        (shm / name).write_bytes(b'')
    _process(tmp_path / 'proc', 10, ['/usr/bin/talker'], maps='7f00 rw-s /dev/shm/fastrtps_aaa\n')

    warn = checks.check_dds_shared_memory(_context(tmp_path, shm_orphan_warn_threshold=2))
    ok = checks.check_dds_shared_memory(_context(tmp_path, shm_orphan_warn_threshold=3))

    assert warn.status is checks.Status.WARN
    assert '共享内存段 4 个' in warn.message
    assert '孤儿段 2 个' in warn.message
    assert ok.status is checks.Status.OK


def test_dds_shared_memory_skips_without_directory(tmp_path):
    """Machines without /dev/shm skip the check."""
    assert checks.check_dds_shared_memory(_context(tmp_path)).status is checks.Status.SKIP


def test_map_yaml_modes(tmp_path):
    """Trinary or unspecified modes pass; scale fails; unreadable files warn."""
    trinary = tmp_path / 'nav.yaml'
    trinary.write_text('image: map.pgm\nmode: trinary\n', encoding='utf-8')
    default = tmp_path / 'default.yaml'
    default.write_text('image: map.pgm\n', encoding='utf-8')
    scale = tmp_path / 'lightning.yaml'
    scale.write_text('image: map.pgm\nmode: scale\n', encoding='utf-8')
    broken = tmp_path / 'broken.yaml'
    broken.write_text('mode: [unclosed\n', encoding='utf-8')

    ok = checks.check_map_yaml_modes(_context(tmp_path, map_yamls=[trinary, default]))
    fail = checks.check_map_yaml_modes(_context(tmp_path, map_yamls=[trinary, scale]))
    warn = checks.check_map_yaml_modes(_context(tmp_path, map_yamls=[trinary, broken]))
    skip = checks.check_map_yaml_modes(_context(tmp_path))

    assert ok.status is checks.Status.OK
    assert fail.status is checks.Status.FAIL
    assert 'lightning.yaml: mode=scale' in fail.message
    assert warn.status is checks.Status.WARN
    assert skip.status is checks.Status.SKIP


def test_robot_state_publishers(tmp_path):
    """More than one publisher warns; search tools are not counted."""
    proc = tmp_path / 'proc'
    _process(proc, 20, ['/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher'])
    _process(proc, 21, ['pgrep', '-f', 'robot_state_publisher'])

    assert checks.check_robot_state_publishers(_context(tmp_path)).status is checks.Status.OK

    _process(proc, 22, ['/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher'])
    result = checks.check_robot_state_publishers(_context(tmp_path))

    assert result.status is checks.Status.WARN
    assert 'pid 20, 22' in result.message


def test_parse_installed_programs():
    """install(PROGRAMS) blocks with and without RENAME are recognized."""
    cmake = '''
install(PROGRAMS
  david_nav/dock_navigator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dock_navigator
)
install(PROGRAMS navigate.sh DESTINATION lib/${PROJECT_NAME})
install(TARGETS something DESTINATION lib/${PROJECT_NAME})
'''

    assert checks.parse_installed_programs(cmake) == [
        ('david_nav/dock_navigator.py', 'dock_navigator'),
        ('navigate.sh', 'navigate.sh'),
    ]


def test_stale_installed_programs(tmp_path):
    """Copies that differ from their sources are reported; symlinks are ignored."""
    package = tmp_path / 'src' / 'david_nav'
    (package / 'david_nav').mkdir(parents=True)
    (package / 'CMakeLists.txt').write_text(
        'install(PROGRAMS david_nav/a.py DESTINATION lib/${PROJECT_NAME} RENAME a)\n'
        'install(PROGRAMS david_nav/b.py DESTINATION lib/${PROJECT_NAME} RENAME b)\n'
        'install(PROGRAMS david_nav/c.py DESTINATION lib/${PROJECT_NAME} RENAME c)\n',
        encoding='utf-8')
    for name in ('a', 'b', 'c'):
        source = package / 'david_nav' / f'{name}.py'
        source.write_text(f'print("{name} new")\n', encoding='utf-8')
    lib = tmp_path / 'install' / 'david_nav' / 'lib' / 'david_nav'
    lib.mkdir(parents=True)
    (lib / 'a').write_text('print("a new")\n', encoding='utf-8')
    (lib / 'b').write_text('print("b old")\n', encoding='utf-8')
    (lib / 'c').symlink_to(package / 'david_nav' / 'c.py')

    result = checks.check_stale_installed_programs(_context(tmp_path))

    assert result.status is checks.Status.WARN
    assert result.message.endswith('：b')


def test_stale_installed_programs_skips_without_install(tmp_path):
    """Without a build there is nothing to compare."""
    assert checks.check_stale_installed_programs(_context(tmp_path)).status is checks.Status.SKIP


def test_serial_device(tmp_path):
    """Missing or inaccessible devices warn; an accessible one passes."""
    device = tmp_path / 'wheeltec_controller'

    assert checks.check_serial_device(_context(tmp_path)).status is checks.Status.WARN

    device.write_bytes(b'')
    assert checks.check_serial_device(_context(tmp_path)).status is checks.Status.OK


@pytest.mark.skipif(hasattr(os, 'geteuid') and os.geteuid() == 0, reason='root ignores modes')
def test_serial_device_without_permissions(tmp_path):
    """A device the user cannot open for reading and writing warns."""
    device = tmp_path / 'wheeltec_controller'
    device.write_bytes(b'')
    device.chmod(0o000)
    try:
        result = checks.check_serial_device(_context(tmp_path))
    finally:
        device.chmod(0o600)

    assert result.status is checks.Status.WARN
    assert '读写权限' in result.message


def test_ros_log_dir(tmp_path):
    """Many or large ROS logs warn; a missing directory is skipped."""
    logs = tmp_path / 'ros_log'
    (logs / 'launch_dir').mkdir(parents=True)
    (logs / 'launch_dir' / 'launch.log').write_bytes(b'x' * 100)
    for index in range(3):
        (logs / f'python3_{index}_1.log').write_bytes(b'y' * 10)

    warn = checks.check_ros_log_dir(_context(tmp_path, ros_log_dir=logs, ros_log_warn_entries=4))
    ok = checks.check_ros_log_dir(_context(tmp_path, ros_log_dir=logs, ros_log_warn_entries=5))
    too_big = checks.check_ros_log_dir(
        _context(tmp_path, ros_log_dir=logs, ros_log_warn_bytes=130))
    skip = checks.check_ros_log_dir(_context(tmp_path, ros_log_dir=tmp_path / 'missing'))

    assert warn.status is checks.Status.WARN
    assert '4 个条目' in warn.message
    assert ok.status is checks.Status.OK
    assert too_big.status is checks.Status.WARN
    assert skip.status is checks.Status.SKIP


def test_ros_log_directory():
    """ROS_LOG_DIR wins over ROS_HOME/log, which wins over ~/.ros/log."""
    assert checks.ros_log_directory({'ROS_LOG_DIR': '/tmp/a', 'ROS_HOME': '/tmp/b'}) == (
        Path('/tmp/a'))
    assert checks.ros_log_directory({'ROS_HOME': '/tmp/b'}) == Path('/tmp/b/log')
    assert checks.ros_log_directory({}) == Path('~/.ros/log').expanduser()


def test_run_checks_isolates_broken_checks(tmp_path):
    """A check that raises becomes a FAIL result without stopping the others."""
    def check_explodes(context):
        raise RuntimeError('boom')

    results = checks.run_checks(
        _context(tmp_path), [check_explodes, checks.check_dds_shared_memory])

    assert [(r.name, r.status) for r in results] == [
        ('explodes', checks.Status.FAIL), ('dds_shared_memory', checks.Status.SKIP)]
    assert 'boom' in results[0].message


def test_worst_status_and_report():
    """The report lists every result, hints for problems, and totals."""
    results = [
        checks.CheckResult('a', checks.Status.OK, 'fine', 'unused hint'),
        checks.CheckResult('b', checks.Status.WARN, 'hmm', '做点什么'),
        checks.CheckResult('c', checks.Status.SKIP, 'n/a'),
    ]

    report = checks.format_report(results)

    assert checks.worst_status(results) is checks.Status.WARN
    assert checks.worst_status([]) is checks.Status.OK
    assert report.splitlines() == [
        '[ OK ] a: fine',
        '[WARN] b: hmm',
        '       提示：做点什么',
        '[SKIP] c: n/a',
        '合计：OK 1 / WARN 1 / FAIL 0 / SKIP 1',
    ]


def test_default_context_uses_navrun_repo(tmp_path):
    """The repository, map yamls and ROS log directory follow the environment."""
    maps = tmp_path / 'src' / 'david_nav' / 'config' / 'maps'
    maps.mkdir(parents=True)
    (maps / 'site.yaml').write_text('mode: trinary\n', encoding='utf-8')

    context = checks.default_context(
        {'NAVRUN_REPO': str(tmp_path / 'src'), 'ROS_LOG_DIR': str(tmp_path / 'ros_log')})

    assert context.install_dir == tmp_path / 'install'
    assert context.map_yamls == (maps / 'site.yaml',)
    assert context.ros_log_dir == tmp_path / 'ros_log'


def test_cli_doctor_json_output(tmp_path, monkeypatch, capsys):
    """The doctor subcommand with --json prints one object per check and exits 0 or 1."""
    monkeypatch.setenv('NAVRUN_REPO', str(tmp_path / 'src'))
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))

    code = cli.main(['doctor', '--json'])
    payload = json.loads(capsys.readouterr().out)

    assert code in (0, 1)
    assert [item['name'] for item in payload] == [
        'ros_environment', 'dds_shared_memory', 'map_yaml_mode', 'robot_state_publishers',
        'stale_installed_programs', 'serial_device', 'ros_log_dir']
    assert all(item['status'] in ('OK', 'WARN', 'FAIL', 'SKIP') for item in payload)
    assert Path(tmp_path / 'src').exists() is False
