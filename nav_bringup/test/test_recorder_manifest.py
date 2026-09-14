"""Tests for the run manifest."""

import datetime
import sys

import pytest

from nav_bringup.recorder import manifest

REPO = '/repo'
COMMIT = 'fe1ac78d0c4b9a2e5f1d3c7b8a6e4f2d1c0b9a87'


def _fake_runner(responses):
    """Return a runner answering ``git -C REPO <args>`` from a table."""
    calls = []

    def run(argv):
        calls.append(list(argv))
        assert argv[:3] == ['git', '-C', REPO]
        return responses.get(tuple(argv[3:]), (1, ''))

    return run, calls


def _git_responses(status=''):
    return {
        ('rev-parse', '--show-toplevel'): (0, '/repo\n'),
        ('rev-parse', '--abbrev-ref', 'HEAD'): (0, 'refactor/nav-system\n'),
        ('rev-parse', 'HEAD'): (0, COMMIT + '\n'),
        ('status', '--porcelain'): (0, status),
        ('diff', 'HEAD'): (0, 'diff --git a/x b/x\n'),
    }


def test_collect_git_info_clean_repository():
    """A clean checkout reports branch, commit and no changes."""
    runner, _ = _fake_runner(_git_responses())

    info = manifest.collect_git_info(REPO, runner)

    assert info == manifest.GitInfo('/repo', 'refactor/nav-system', COMMIT, False, 0)


def test_collect_git_info_dirty_repository_counts_changes():
    """Uncommitted changes mark the run as dirty."""
    runner, _ = _fake_runner(_git_responses(' M a.py\n?? b.py\n'))

    info = manifest.collect_git_info(REPO, runner)

    assert info.dirty is True
    assert info.changed_files == 2


def test_collect_git_info_outside_repository():
    """Outside git every field is empty."""
    runner, _ = _fake_runner({})

    assert manifest.collect_git_info(REPO, runner) == manifest.GitInfo(None, None, None, False, 0)


def test_write_code_diff_records_status_and_diff(tmp_path):
    """A dirty tree produces a reconstructable diff file."""
    runner, _ = _fake_runner(_git_responses(' M a.py\n'))
    destination = tmp_path / 'code.diff'

    assert manifest.write_code_diff(REPO, destination, runner) is True
    assert destination.read_text(encoding='utf-8') == (
        '# git status --porcelain\n#  M a.py\n\ndiff --git a/x b/x\n')


def test_write_code_diff_skips_clean_tree(tmp_path):
    """A clean tree writes no file."""
    runner, calls = _fake_runner(_git_responses())
    destination = tmp_path / 'code.diff'

    assert manifest.write_code_diff(REPO, destination, runner) is False
    assert not destination.exists()
    assert ['git', '-C', REPO, 'diff', 'HEAD'] not in calls


def test_sha256_file_and_missing_file(tmp_path):
    """Files are fingerprinted; missing files have no hash."""
    target = tmp_path / 'map.yaml'
    target.write_bytes(b'abc')

    assert manifest.sha256_file(target) == (
        'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad')
    assert manifest.sha256_file(tmp_path / 'missing.yaml') is None


def test_collect_environment_picks_known_keys_only():
    """Only ROS-relevant variables are recorded, unset ones as None."""
    environment = manifest.collect_environment(
        {'ROS_DOMAIN_ID': '0', 'HOME': '/home/agx'}, keys=('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION'))

    assert environment == {'ROS_DOMAIN_ID': '0', 'RMW_IMPLEMENTATION': None}


def test_collect_versions_reports_missing_distributions():
    """Unknown distributions are reported instead of raising."""
    versions = manifest.collect_versions(('pytest', 'surely-not-installed-dist'))

    assert versions['python'] == '.'.join(str(part) for part in sys.version_info[:3])
    assert versions['pytest'] != 'not installed'
    assert versions['surely-not-installed-dist'] == 'not installed'


def test_run_command_captures_output_and_missing_program():
    """Commands report their exit code and stdout; missing programs yield 127."""
    code, output = manifest.run_command([sys.executable, '-c', 'print("ok")'])

    assert (code, output) == (0, 'ok\n')
    assert manifest.run_command(['surely-not-a-program-name']) == (127, '')


def test_manifest_round_trip_with_finalization(tmp_path):
    """A manifest survives YAML round trips and records duration and exit."""
    tz = datetime.timezone(datetime.timedelta(hours=8))
    map_yaml = tmp_path / 'map.yaml'
    map_yaml.write_bytes(b'abc')
    started = datetime.datetime(2026, 9, 14, 21, 13, 42, tzinfo=tz)
    runner, _ = _fake_runner(_git_responses())

    built = manifest.build_manifest(
        run_id='20260914_211342_tour_fe1ac78',
        mode='tour',
        command=['start_navigation_headless.sh', '--no-bag'],
        started_at=started,
        git=manifest.collect_git_info(REPO, runner),
        versions={'python': '3.10.12'},
        environment={'ROS_DOMAIN_ID': '0'},
        files={'map_yaml': map_yaml},
        robot='david',
        site='展厅',
        alias='4',
    )
    finalized = manifest.finalize_manifest(
        built, started + datetime.timedelta(minutes=5, seconds=3), 130, 'ctrl_c')
    path = tmp_path / 'manifest.yaml'
    manifest.write_manifest(path, finalized)
    loaded = manifest.read_manifest(path)

    assert built['ended_at'] is None
    assert loaded['started_at'] == '2026-09-14T21:13:42+08:00'
    assert loaded['ended_at'] == '2026-09-14T21:18:45+08:00'
    assert loaded['duration_sec'] == 303.0
    assert loaded['exit_code'] == 130
    assert loaded['exit_reason'] == 'ctrl_c'
    assert loaded['site'] == '展厅'
    assert loaded['git']['commit'] == COMMIT
    assert loaded['files']['map_yaml']['sha256'].startswith('ba7816bf')
    assert '展厅' in path.read_text(encoding='utf-8')


def test_read_manifest_rejects_non_mapping(tmp_path):
    """A manifest must be a YAML mapping."""
    path = tmp_path / 'manifest.yaml'
    path.write_text('- just\n- a list\n', encoding='utf-8')

    with pytest.raises(ValueError):
        manifest.read_manifest(path)
