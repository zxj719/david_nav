"""Exercise record inspection through the CLI, with no ROS or process control."""

import json
import os

import pytest
import yaml

from nav_bringup.navrun import cli

RUN = '20260916_010203_tour_abcdef0'


@pytest.fixture
def record(tmp_path):
    """Create only synthetic data, including fields that default output must omit."""
    run = tmp_path / RUN
    run.mkdir()
    (tmp_path / 'latest').symlink_to(RUN)
    (run / 'manifest.yaml').write_text(yaml.safe_dump({
        'schema_version': 1, 'run_id': RUN, 'mode': 'tour', 'robot': 'example',
        'ended_at': None, 'command': ['secret-command'], 'environment': {'SECRET': 'value'},
    }))
    (run / 'summary.md').write_text('Synthetic summary\n')
    for name in ('inputs', 'rendered', 'effective'):
        (run / 'params' / name).mkdir(parents=True)
    (run / 'params/inputs/robot.yaml').write_text('speed: 1\nkeep: true\n')
    (run / 'params/inputs/mode.yaml').write_text(
        'schema_version: 1\nmode: tour\nprofile: {speed: 2}\nnav2_overrides: {}\n')
    (run / 'params/rendered/nav2.yaml').write_text('node: {ros__parameters: {gain: 2}}\n')
    (run / 'params/effective/node.yaml').write_text('/node: {ros__parameters: {gain: 3}}\n')
    (run / 'events.jsonl').write_text('{"stamp": 123, "event": "ready"}\n{"stamp":')
    return run


def _invoke(root, capsys, *args):
    status = cli.main([*args, '--runs-root', str(root)])
    captured = capsys.readouterr()
    return status, captured


def test_listing_missing_root_does_not_create_it(tmp_path, capsys):
    """An empty installation is a valid, completely read-only list."""
    root = tmp_path / 'missing'
    status, output = _invoke(root, capsys, 'ls')
    assert status == 0
    assert json.loads(output.out) == []
    assert not root.exists()


@pytest.mark.parametrize('selector', [RUN, 'latest'])
def test_show_summary_does_not_claim_liveness_or_expose_raw_manifest(record, capsys, selector):
    """An unfinished record is not proof of a living process."""
    status, output = _invoke(record.parent, capsys, 'show', selector)
    report = json.loads(output.out)
    assert status == 0
    assert report['run_id'] == RUN
    assert report['record_state'] == 'unfinished'
    assert report['summary'] == 'Synthetic summary\n'
    assert 'secret-command' not in output.out and 'SECRET' not in output.out
    assert 'running' not in output.out


def test_list_keeps_corrupt_record_visible_and_ignores_symlink_runs(record, capsys):
    """One incomplete record must not hide all the other runs."""
    other = record.parent / '20260916_010204_tour_nogit'
    other.mkdir()
    (other / 'manifest.yaml').write_text('not: [valid')
    (record.parent / '20260916_010205_tour_nogit').symlink_to(record, target_is_directory=True)
    status, output = _invoke(record.parent, capsys, 'ls')
    result = json.loads(output.out)
    assert status == 0 and len(result) == 2
    assert result[0]['run_id'] == other.name and result[0]['record_state'] == 'unreadable'
    assert result[1]['run_id'] == RUN


def test_params_exports_exact_saved_files_not_a_new_configuration(record, capsys):
    """Missing dumps remain missing; rendered files are never labelled effective."""
    status, output = _invoke(record.parent, capsys, 'show', RUN, '--params')
    result = json.loads(output.out)
    assert status == 0
    assert result['params']['effective']['node.yaml'] == (
        record / 'params/effective/node.yaml').read_text()
    assert result['params']['rendered']['nav2.yaml'] == (
        record / 'params/rendered/nav2.yaml').read_text()
    assert 'effective' not in result['params']['inputs']
    assert result['snapshot_atomic'] is False


def test_diff_reports_mode_profile_changes_and_explicit_nav2_overrides(record, capsys):
    """Tuning differences are computed from the frozen inputs, not today's profile."""
    status, output = _invoke(record.parent, capsys, 'show', RUN, '--diff')
    result = json.loads(output.out)['diff']
    assert status == 0
    assert '-speed: 1' in result['profile_unified']
    assert '+speed: 2' in result['profile_unified']
    assert result['nav2_overrides'] == {}
    assert result['runtime_comparison'] == 'not_computed'


def test_events_preserves_complete_lines_and_marks_pending_tail(record, capsys):
    """A concurrent writer's last incomplete line is not a corrupt complete event."""
    status, output = _invoke(record.parent, capsys, 'show', RUN, '--events')
    result = json.loads(output.out)['events']
    assert status == 0
    assert result['jsonl'] == '{"stamp": 123, "event": "ready"}\n'
    assert result['pending_tail'] is True


@pytest.mark.parametrize('selector', ['../outside', '/tmp/outside', '.', 'latest/../outside'])
def test_invalid_selectors_rejected(record, capsys, selector):
    """Only an exact run ID or the special latest selector is accepted."""
    status, output = _invoke(record.parent, capsys, 'show', selector)
    assert status == 2 and not output.out


@pytest.mark.parametrize('target', ['../outside', '/tmp/outside', './' + RUN])
def test_latest_requires_direct_relative_run_id(record, capsys, target):
    """A latest link cannot authorize traversal outside the selected root."""
    link = record.parent / 'latest'
    link.unlink()
    link.symlink_to(target)
    status, output = _invoke(record.parent, capsys, 'show', 'latest')
    assert status == 2 and not output.out


@pytest.mark.parametrize('relative,flag', [
    ('manifest.yaml', None), ('summary.md', None), ('events.jsonl', '--events'),
    ('params/effective/node.yaml', '--params'), ('params/inputs/mode.yaml', '--diff'),
])
@pytest.mark.parametrize('kind', ['symlink', 'fifo', 'hardlink'])
def test_special_files_rejected_without_reading_them(record, capsys, relative, flag, kind):
    """Neither a symlink, hardlink nor a FIFO may become a record input."""
    target = record / relative
    target.unlink()
    outside = record.parent / 'outside'
    outside.write_text('DO_NOT_DISCLOSE')
    if kind == 'symlink':
        target.symlink_to(outside)
    elif kind == 'hardlink':
        os.link(outside, target)
    else:
        os.mkfifo(target)
    status, output = _invoke(record.parent, capsys, 'show', RUN, *([flag] if flag else []))
    assert status == 2 and not output.out
    assert 'DO_NOT_DISCLOSE' not in output.err


def test_parameter_directory_symlink_rejected(record, capsys):
    """Every directory component is checked, not just the leaf file."""
    effective = record / 'params/effective'
    effective.rename(record / 'saved')
    effective.symlink_to(record / 'saved', target_is_directory=True)
    status, output = _invoke(record.parent, capsys, 'show', RUN, '--params')
    assert status == 2 and not output.out


def test_oversized_file_is_rejected_before_read(record, capsys, monkeypatch):
    """Inspection has a hard byte budget, including regular files that keep growing."""
    from nav_bringup.navrun import record_reader

    monkeypatch.setattr(record_reader, 'MAX_FILE_BYTES', 16)
    status, output = _invoke(record.parent, capsys, 'show', RUN)
    assert status == 2 and not output.out


def test_manifest_identity_mismatch_is_an_error(record, capsys):
    """A valid YAML document from a different run is not authoritative here."""
    (record / 'manifest.yaml').write_text('schema_version: 1\nrun_id: another\n')
    status, output = _invoke(record.parent, capsys, 'show', RUN)
    assert status == 2 and not output.out


def test_recursive_yaml_is_rejected_for_diff(record, capsys):
    """Malformed aliases must not hang recursive profile merging."""
    (record / 'params/inputs/robot.yaml').write_text('recursive: &r {again: *r}\n')
    status, output = _invoke(record.parent, capsys, 'show', RUN, '--diff')
    assert status == 2 and not output.out


def test_missing_optional_artifacts_stay_unavailable(record, capsys):
    """Retention-pruned historical records are still useful, without invented dumps."""
    (record / 'summary.md').unlink()
    (record / 'events.jsonl').unlink()
    (record / 'params/inputs/robot.yaml').unlink()
    (record / 'params/effective/node.yaml').unlink()
    status, output = _invoke(record.parent, capsys, 'show', RUN,
                             '--params', '--events', '--diff')
    result = json.loads(output.out)
    assert status == 0 and result['summary'] is None
    assert result['events'] == {'available': False, 'jsonl': '', 'pending_tail': False}
    assert result['diff']['available'] is False
    assert result['params']['effective'] == {}


def test_environment_default_and_no_file_changes(record, capsys, monkeypatch):
    """The standard runs-root environment works without adding lock or output files."""
    monkeypatch.setenv('NAV_RUNS_ROOT', str(record.parent))
    before = {str(path): path.read_bytes() for path in record.rglob('*') if path.is_file()}
    assert cli.main(['show', 'latest', '--params', '--diff', '--events']) == 0
    capsys.readouterr()
    after = {str(path): path.read_bytes() for path in record.rglob('*') if path.is_file()}
    assert before == after
    assert sorted(path.name for path in record.parent.iterdir()) == [RUN, 'latest']


def test_total_byte_budget_covers_multiple_files(record, capsys, monkeypatch):
    """Several individually small files cannot bypass the aggregate read limit."""
    from nav_bringup.navrun import record_reader

    manifest_size = (record / 'manifest.yaml').stat().st_size
    monkeypatch.setattr(record_reader, 'MAX_TOTAL_BYTES', manifest_size + 1)
    status, output = _invoke(record.parent, capsys, 'show', RUN)
    assert status == 2 and not output.out


def test_tree_entry_budget_is_shared(record, capsys, monkeypatch):
    """Directory counts accumulate across inputs, rendered and effective trees."""
    from nav_bringup.navrun import record_reader

    monkeypatch.setattr(record_reader, 'MAX_ENTRIES', 2)
    status, output = _invoke(record.parent, capsys, 'show', RUN, '--params')
    assert status == 2 and not output.out


@pytest.mark.parametrize('target', ['root', 'run'])
def test_symlink_root_or_run_is_rejected(record, capsys, target):
    """The only permitted link is the validated latest selector, not run directories."""
    if target == 'root':
        root = record.parent / 'redirect'
        root.symlink_to(record.parent, target_is_directory=True)
    else:
        record.rename(record.parent / 'saved')
        record.symlink_to(record.parent / 'saved', target_is_directory=True)
        root = record.parent
    status, output = _invoke(root, capsys, 'show', RUN)
    assert status == 2 and not output.out


def test_open_record_is_pinned_when_directory_name_is_replaced(record):
    """Replacing the name after open cannot redirect reads to another run."""
    from nav_bringup.navrun.record_reader import open_record

    with open_record(record.parent, RUN) as reader:
        record.rename(record.parent / 'saved')
        record.mkdir()
        (record / 'summary.md').write_text('wrong generation')
        assert reader.read('summary.md') == 'Synthetic summary\n'


def test_file_growing_during_read_is_rejected(record, monkeypatch):
    """A stat-size precheck is insufficient: bounded reads detect concurrent growth."""
    from nav_bringup.navrun import record_reader

    monkeypatch.setattr(record_reader, 'MAX_FILE_BYTES', 32)
    original = os.read

    def grow(descriptor, count):
        with (record / 'summary.md').open('a') as handle:
            handle.write('x' * 64)
        return original(descriptor, count)

    monkeypatch.setattr(record_reader.os, 'read', grow)
    with record_reader.open_record(record.parent, RUN) as reader:
        with pytest.raises(ValueError, match='budget'):
            reader.read('summary.md')
