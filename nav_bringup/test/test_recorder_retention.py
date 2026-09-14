"""Tests for the run retention policy."""

import datetime

import pytest

from nav_bringup.recorder import retention
from nav_bringup.recorder import run_dir

NOW = datetime.datetime(2026, 9, 20, 12, 0, 0)


def _make_run(root, run_id, payload_bytes=100, bag_bytes=0):
    run = run_dir.create_run_directory(root, run_id)
    run.manifest.write_text('run_id: x\n', encoding='utf-8')
    run.summary.write_text('# summary\n', encoding='utf-8')
    (run.logs / 'launch.log').write_bytes(b'x' * payload_bytes)
    if bag_bytes:
        (run.bag / 'bag_0.db3').write_bytes(b'b' * bag_bytes)
    return run


def test_max_runs_prunes_oldest_runs(tmp_path):
    """Runs beyond the newest max_runs are pruned, oldest first."""
    runs = [
        _make_run(tmp_path, f'202609{day:02d}_120000_tour_fe1ac78') for day in (10, 11, 12, 13)
    ]

    plan = retention.plan_gc(runs, retention.RetentionPolicy(max_runs=2), NOW)

    assert [run.run_id for run in plan.prune_runs] == [
        '20260911_120000_tour_fe1ac78', '20260910_120000_tour_fe1ac78']


def test_size_budget_prunes_older_runs(tmp_path):
    """Once the newest runs use up the byte budget, older runs are pruned."""
    old = _make_run(tmp_path, '20260918_120000_tour_fe1ac78', payload_bytes=5000)
    new = _make_run(tmp_path, '20260919_120000_tour_fe1ac78', payload_bytes=5000)
    policy = retention.RetentionPolicy(max_total_bytes=8000)

    plan = retention.plan_gc([old, new], policy, NOW)

    assert plan.prune_runs == [old]


def test_protected_run_is_never_pruned(tmp_path):
    """The current run stays complete even when it breaks the budget."""
    current = _make_run(tmp_path, '20260919_120000_tour_fe1ac78', payload_bytes=5000)
    policy = retention.RetentionPolicy(max_runs=0, max_total_bytes=1)

    plan = retention.plan_gc([current], policy, NOW, protected_run_id=current.run_id)

    assert plan == retention.GcPlan()


def test_old_bags_of_complete_runs_are_deleted(tmp_path):
    """Bags older than the age limit go; recent bags and empty bag dirs stay."""
    old = _make_run(tmp_path, '20260910_120000_tour_fe1ac78', bag_bytes=1000)
    recent = _make_run(tmp_path, '20260919_120000_tour_fe1ac78', bag_bytes=1000)
    old_empty = _make_run(tmp_path, '20260909_120000_tour_fe1ac78')

    plan = retention.plan_gc([old, recent, old_empty], retention.RetentionPolicy(), NOW)

    assert plan.delete_bags == [old.bag]
    assert plan.prune_runs == []


def test_already_pruned_runs_are_ignored(tmp_path):
    """Pruned runs neither count against the budget nor get pruned again."""
    pruned = _make_run(tmp_path, '20260910_120000_tour_fe1ac78')
    retention.apply_gc(retention.GcPlan(prune_runs=[pruned]), tmp_path)
    fresh = _make_run(tmp_path, '20260911_120000_tour_fe1ac78')

    plan = retention.plan_gc([pruned, fresh], retention.RetentionPolicy(max_runs=1), NOW)

    assert retention.is_pruned(pruned)
    assert plan == retention.GcPlan()


def test_apply_gc_keeps_manifest_and_summary(tmp_path):
    """Pruning removes everything except the manifest and the summary."""
    run = _make_run(tmp_path, '20260910_120000_tour_fe1ac78', payload_bytes=300, bag_bytes=700)
    keep = _make_run(tmp_path, '20260912_120000_tour_fe1ac78', bag_bytes=50)
    plan = retention.GcPlan(prune_runs=[run], delete_bags=[keep.bag])

    freed = retention.apply_gc(plan, tmp_path)

    assert sorted(entry.name for entry in run.path.iterdir()) == ['manifest.yaml', 'summary.md']
    assert not keep.bag.exists()
    assert keep.manifest.exists()
    assert freed == 300 + 700 + 50


def test_apply_gc_refuses_paths_outside_root(tmp_path):
    """A plan cannot delete anything that is not a run directory of the root."""
    outside = run_dir.RunDirectory(tmp_path / 'elsewhere', '20260910_120000_tour_fe1ac78')
    outside.path.mkdir(parents=True)
    (outside.path / 'important.txt').write_text('keep', encoding='utf-8')
    (tmp_path / 'runs').mkdir()

    with pytest.raises(ValueError):
        retention.apply_gc(retention.GcPlan(prune_runs=[outside]), tmp_path / 'runs')
    assert (outside.path / 'important.txt').exists()


def test_directory_size_ignores_symlinks_and_missing(tmp_path):
    """Sizes count regular files only."""
    (tmp_path / 'a.bin').write_bytes(b'x' * 10)
    (tmp_path / 'link').symlink_to(tmp_path / 'a.bin')

    assert retention.directory_size(tmp_path) == 10
    assert retention.directory_size(tmp_path / 'missing') == 0
