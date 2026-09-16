"""Exercise retention only on test-owned records; never use the operator's run root."""

import datetime
import json
import os

import pytest
import yaml

from nav_bringup.navrun import cli
from nav_bringup.navrun.locking import RunLock
from nav_bringup.recorder import retention, run_dir, safe_retention

NOW = datetime.datetime(2026, 9, 20, 12)


def _record(root, day=10, complete=True):
    record = run_dir.create_run_directory(root, f'202609{day:02d}_120000_tour_abcdef0')
    record.manifest.write_text(yaml.safe_dump({
        'schema_version': 1, 'run_id': record.run_id,
        'ended_at': '2026-09-19T12:00:00' if complete else None}))
    record.summary.write_text('permanent summary')
    (record.logs / 'payload').write_bytes(b'x' * 100)
    (record.bag / 'data.db3').write_bytes(b'b' * 50)
    return record


def _call(root, capsys, *options):
    status = cli.main(['gc', '--runs-root', str(root),
                       '--lock-path', str(root.parent / 'navigation.lease'), *options])
    return status, json.loads(capsys.readouterr().out)


def test_gc_dry_run_does_not_create_locks_or_delete_payload(tmp_path, capsys):
    """Preview is side-effect free and reports exact targets before deletion."""
    root = tmp_path / 'runs'
    record = _record(root)
    before = {str(p): p.read_bytes() for p in root.rglob('*') if p.is_file()}
    status, report = _call(root, capsys, '--max-runs', '0', '--dry-run')
    assert status == 0 and report['dry_run'] is True
    assert report['pruned_runs'] == [record.run_id]
    assert {str(p): p.read_bytes() for p in root.rglob('*') if p.is_file()} == before
    assert not (tmp_path / 'navigation.lease').exists()


def test_gc_applies_and_permanently_preserves_manifest_and_summary(tmp_path, capsys):
    """Pruning removes only replaceable record artifacts, never its identity and summary."""
    root = tmp_path / 'runs'
    record = _record(root)
    manifest = record.manifest.read_bytes()
    summary = record.summary.read_bytes()
    status, report = _call(root, capsys, '--max-runs', '0')
    assert status == 0 and report['removed_bytes'] == 150
    assert sorted(p.name for p in record.path.iterdir()) == ['manifest.yaml', 'summary.md']
    assert record.manifest.read_bytes() == manifest and record.summary.read_bytes() == summary
    assert (root / 'latest').readlink().name == record.run_id


@pytest.mark.parametrize('mutation', ['unfinished', 'invalid', 'identity', 'missing_summary'])
def test_gc_skips_unproven_completion(tmp_path, capsys, mutation):
    """Neither malformed nor incomplete records are authoritative deletion candidates."""
    root = tmp_path / 'runs'
    record = _record(root, complete=mutation != 'unfinished')
    if mutation == 'invalid':
        record.manifest.write_text('not: [valid')
    elif mutation == 'identity':
        data = yaml.safe_load(record.manifest.read_text())
        data['run_id'] = 'another_run'
        record.manifest.write_text(yaml.safe_dump(data))
    elif mutation == 'missing_summary':
        record.summary.unlink()
    status, report = _call(root, capsys, '--max-runs', '0')
    assert status == 0 and report['pruned_runs'] == []
    assert (record.logs / 'payload').read_bytes() == b'x' * 100


def test_busy_navigation_lease_prevents_mutating_gc(tmp_path, capsys):
    """An explicit maintenance invocation cannot race a managed navigation startup."""
    root = tmp_path / 'runs'
    record = _record(root)
    with RunLock(tmp_path / 'navigation.lease'):
        status, report = _call(root, capsys, '--max-runs', '0')
        assert status == 2 and report['applied'] is False
    assert (record.logs / 'payload').exists()


@pytest.mark.parametrize('option,value', [
    ('--max-runs', '-1'), ('--max-total-bytes', '-1'), ('--bag-max-age-days', 'nan')])
def test_invalid_policy_is_rejected_before_any_lock_or_deletion(tmp_path, capsys, option, value):
    """Invalid retention limits must not turn into delete-everything policies."""
    root = tmp_path / 'runs'
    record = _record(root)
    status, report = _call(root, capsys, option, value)
    assert status == 2 and report['applied'] is False
    assert (record.logs / 'payload').exists()
    assert not (tmp_path / 'navigation.lease').exists()


def test_policy_excludes_unfinished_history_even_outside_current_run(tmp_path):
    """Shared planning must protect other wraps, not only the caller's current record."""
    record = _record(tmp_path, complete=False)
    plan = retention.plan_gc([record], retention.RetentionPolicy(max_runs=0), NOW)
    assert not plan.prune_runs and not plan.delete_bags


def test_apply_refuses_run_symlink_to_another_valid_record(tmp_path):
    """A swapped planned path must not turn a different valid record into the victim."""
    first = _record(tmp_path, 10)
    keep = _record(tmp_path, 11)
    plan = retention.plan_gc([first], retention.RetentionPolicy(max_runs=0), NOW)
    first.path.rename(tmp_path / 'saved')
    first.path.symlink_to(keep.path, target_is_directory=True)
    with pytest.raises((ValueError, OSError)):
        retention.apply_gc(plan, tmp_path)
    assert (keep.logs / 'payload').exists()


def test_apply_refuses_real_directory_replacement_since_planning(tmp_path):
    """Even a non-symlink replacement with a valid manifest is a different incarnation."""
    record = _record(tmp_path)
    plan = retention.plan_gc([record], retention.RetentionPolicy(max_runs=0), NOW)
    record.path.rename(tmp_path / 'saved')
    replacement = _record(tmp_path)
    with pytest.raises(ValueError):
        retention.apply_gc(plan, tmp_path)
    assert (replacement.logs / 'payload').exists()


def test_symlink_payload_is_unlinked_without_touching_target(tmp_path):
    """Record-local symlinks are entries, never traversal authority into external data."""
    root = tmp_path / 'runs'
    record = _record(root)
    outside = tmp_path / 'outside'
    outside.mkdir()
    (outside / 'important').write_text('keep')
    (record.path / 'redirect').symlink_to(outside, target_is_directory=True)
    plan = retention.plan_gc([record], retention.RetentionPolicy(max_runs=0), NOW)
    retention.apply_gc(plan, root)
    assert (outside / 'important').read_text() == 'keep'


def test_manifest_changed_to_unfinished_after_planning_is_protected(tmp_path):
    """Application rechecks completion instead of blindly trusting the old plan."""
    record = _record(tmp_path)
    plan = retention.plan_gc([record], retention.RetentionPolicy(max_runs=0), NOW)
    data = yaml.safe_load(record.manifest.read_text())
    data['ended_at'] = None
    record.manifest.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        retention.apply_gc(plan, tmp_path)
    assert (record.logs / 'payload').exists()


def test_missing_root_stays_missing(tmp_path, capsys):
    """No records to collect is a successful no-op, not directory creation."""
    root = tmp_path / 'missing'
    status, report = _call(root, capsys)
    assert status == 0 and not report['pruned_runs'] and not root.exists()


def test_special_payload_skips_record_without_opening_fifo(tmp_path):
    """Unrecognized file types are not read or treated as normal retained artifacts."""
    record = _record(tmp_path)
    os.mkfifo(record.path / 'pipe')
    plan = retention.plan_gc([record], retention.RetentionPolicy(max_runs=0), NOW)
    assert not plan.prune_runs


def test_parent_replacement_after_inventory_cannot_reach_external_files(tmp_path):
    """Descriptor-relative deletion refuses a payload parent swapped for a symlink."""
    record = _record(tmp_path)
    with safe_retention.open_root(tmp_path) as descriptor:
        snapshot = safe_retention.inspect_at(descriptor, record.run_id)
        saved = tmp_path / 'saved_logs'
        record.logs.rename(saved)
        record.logs.symlink_to(saved, target_is_directory=True)
        with pytest.raises((ValueError, OSError)):
            safe_retention.remove_at(descriptor, record.run_id, snapshot, False)
    assert (saved / 'payload').read_bytes() == b'x' * 100


@pytest.mark.parametrize('filename', ['manifest.yaml', 'summary.md'])
def test_linked_permanent_metadata_is_not_completion_evidence(tmp_path, filename):
    """Manifest and summary must be real, unaliased files of the selected run."""
    record = _record(tmp_path)
    metadata = record.path / filename
    saved = tmp_path / filename
    metadata.rename(saved)
    metadata.symlink_to(saved)
    plan = retention.plan_gc([record], retention.RetentionPolicy(max_runs=0), NOW)
    assert not plan.prune_runs and record.logs.exists()


def test_collectors_share_a_per_root_lease(tmp_path):
    """A second collector cannot interleave deletion of a completed record."""
    record = _record(tmp_path)
    plan = retention.plan_gc([record], retention.RetentionPolicy(max_runs=0), NOW)
    from nav_bringup.navrun.locking import LockBusy
    with RunLock(tmp_path / '.gc.lock'):
        with pytest.raises(LockBusy):
            retention.apply_gc(plan, tmp_path)
    assert (record.logs / 'payload').exists()


def test_missing_bag_after_planning_fails_without_deleting_logs(tmp_path):
    """A stale age-only target is a controlled error, not a widened full prune."""
    record = _record(tmp_path)
    plan = retention.plan_gc([record], retention.RetentionPolicy(), NOW)
    record.bag.rename(tmp_path / 'saved_bag')
    with pytest.raises(ValueError):
        retention.apply_gc(plan, tmp_path)
    assert (record.logs / 'payload').exists()


def test_actual_age_only_gc_preserves_recent_payload(tmp_path):
    """The age policy removes just the old bag and preserves complete recent runs."""
    old = _record(tmp_path, 10)
    recent = _record(tmp_path, 19)
    plan = retention.plan_gc([old, recent], retention.RetentionPolicy(), NOW)
    assert retention.apply_gc(plan, tmp_path) == 50
    assert not old.bag.exists() and (old.logs / 'payload').exists()
    assert (recent.bag / 'data.db3').exists()


def test_interrupted_gc_reports_partial_possibility_and_releases_lease(
        tmp_path, capsys, monkeypatch):
    """Interruption cannot claim complete collection or leave the navigation lease held."""
    root = tmp_path / 'runs'
    _record(root)

    def interrupted(*args):
        raise KeyboardInterrupt()

    monkeypatch.setattr(retention, 'apply_gc', interrupted)
    status, report = _call(root, capsys, '--max-runs', '0')
    assert status == 130 and report['partial_deletion_possible'] and not report['applied']
    with RunLock(tmp_path / 'navigation.lease'):
        pass
