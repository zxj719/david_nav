"""DDS hygiene uses private temporary files and fake proc maps, never real shared memory."""

from pathlib import Path

import pytest

from nav_bringup.navrun.dds_hygiene import cleanup_hygiene, MappingView
from nav_bringup.navrun.dds_hygiene import plan_hygiene, snapshot_mappings


def _view(*names, complete=True):
    return MappingView(frozenset(names), complete, () if complete else ('unreadable maps',))


def _files(root, *names):
    for name in names:
        (root / name).write_bytes(b'fixture')


def test_only_unmapped_fastdds_files_are_candidates(tmp_path: Path) -> None:
    """Unrelated shared memory objects and mapped segments stay untouched."""
    _files(tmp_path, 'fastrtps_old', 'fastrtps_old_el', 'fastrtps_live', 'other_segment')
    plan = plan_hygiene(tmp_path, _view('fastrtps_live'))
    assert [entry.name for entry in plan.candidates] == ['fastrtps_old', 'fastrtps_old_el']
    result = cleanup_hygiene(plan, lambda: _view('fastrtps_live'))
    assert result.removed == ('fastrtps_old', 'fastrtps_old_el')
    assert (tmp_path / 'fastrtps_live').is_file()
    assert (tmp_path / 'other_segment').is_file()


@pytest.mark.parametrize('mapped', ['fastrtps_live', 'fastrtps_live_el'])
def test_either_mapped_pair_member_protects_both(tmp_path: Path, mapped: str) -> None:
    """An in-use lock companion also protects the segment it belongs to."""
    _files(tmp_path, 'fastrtps_live', 'fastrtps_live_el')
    assert not plan_hygiene(tmp_path, _view(mapped)).candidates


def test_incomplete_mapping_view_never_yields_delete_candidates(tmp_path: Path) -> None:
    """Permission failures are not proof of an orphan segment."""
    _files(tmp_path, 'fastrtps_old')
    plan = plan_hygiene(tmp_path, _view(complete=False))
    assert not plan.candidates
    assert 'incomplete' in plan.retained[0][1]


@pytest.mark.parametrize('kind', ['symlink', 'hardlink', 'directory'])
def test_unsafe_file_or_companion_preserves_the_group(tmp_path: Path, kind: str) -> None:
    """A shared-memory-looking alias cannot redirect removal into unrelated data."""
    _files(tmp_path, 'fastrtps_old', 'original')
    companion = tmp_path / 'fastrtps_old_el'
    if kind == 'symlink':
        companion.symlink_to(tmp_path / 'original')
    elif kind == 'hardlink':
        companion.hardlink_to(tmp_path / 'original')
    else:
        companion.mkdir()
    assert not plan_hygiene(tmp_path, _view()).candidates
    assert (tmp_path / 'original').read_bytes() == b'fixture'


def test_dry_run_does_not_refresh_or_remove(tmp_path: Path) -> None:
    """Dry-run inspects the existing plan without invoking runtime callbacks."""
    _files(tmp_path, 'fastrtps_old')
    plan = plan_hygiene(tmp_path, _view())
    result = cleanup_hygiene(plan, lambda: pytest.fail('observer called'), dry_run=True)
    assert not result.removed
    assert result.planned == ('fastrtps_old',)
    assert (tmp_path / 'fastrtps_old').is_file()


def test_new_mapping_after_planning_preserves_segment(tmp_path: Path) -> None:
    """Mapping state is refreshed before deletion, not merely at the original snapshot."""
    _files(tmp_path, 'fastrtps_old', 'fastrtps_old_el')
    plan = plan_hygiene(tmp_path, _view())
    result = cleanup_hygiene(plan, lambda: _view('fastrtps_old'))
    assert not result.removed
    assert len(list(tmp_path.iterdir())) == 2


def test_new_mapping_between_pair_deletions_preserves_remaining_member(tmp_path: Path) -> None:
    """Even the second unlink requires a fresh mapping observation."""
    _files(tmp_path, 'fastrtps_old', 'fastrtps_old_el')
    plan = plan_hygiene(tmp_path, _view())
    views = iter([_view(), _view('fastrtps_old_el')])
    result = cleanup_hygiene(plan, lambda: next(views))
    assert result.removed == ('fastrtps_old',)
    assert (tmp_path / 'fastrtps_old_el').is_file()


def test_replaced_inode_or_modified_file_is_not_removed(tmp_path: Path) -> None:
    """The initial file generation is the only eligible cleanup target."""
    _files(tmp_path, 'fastrtps_old')
    plan = plan_hygiene(tmp_path, _view())
    (tmp_path / 'fastrtps_old').write_bytes(b'new generation')
    result = cleanup_hygiene(plan, lambda: _view())
    assert not result.removed
    assert (tmp_path / 'fastrtps_old').read_bytes() == b'new generation'


def test_new_unrelated_candidate_is_not_added_to_cleanup(tmp_path: Path) -> None:
    """Only the original generation can be removed, even when later observations find more."""
    _files(tmp_path, 'fastrtps_old')
    plan = plan_hygiene(tmp_path, _view())
    _files(tmp_path, 'fastrtps_new')
    result = cleanup_hygiene(plan, lambda: _view())
    assert result.removed == ('fastrtps_old',)
    assert (tmp_path / 'fastrtps_new').is_file()


def test_other_owners_files_are_retained(tmp_path: Path) -> None:
    """A complete mapping view alone does not grant ownership of another user's files."""
    _files(tmp_path, 'fastrtps_old')
    assert not plan_hygiene(tmp_path, _view(), owner_uid=999999).candidates


def test_late_incomplete_observation_retains_all_candidates(tmp_path: Path) -> None:
    """Permission loss after planning does not fall back to the earlier complete view."""
    _files(tmp_path, 'fastrtps_old')
    plan = plan_hygiene(tmp_path, _view())
    result = cleanup_hygiene(plan, lambda: _view(complete=False))
    assert not result.removed
    assert (tmp_path / 'fastrtps_old').is_file()


def test_replaced_directory_is_rejected_without_touching_new_files(tmp_path: Path) -> None:
    """Matching names inside a replacement root are not the originally inspected files."""
    root = tmp_path / 'root'
    root.mkdir()
    _files(root, 'fastrtps_old')
    plan = plan_hygiene(root, _view())
    root.rename(tmp_path / 'original')
    root.mkdir()
    _files(root, 'fastrtps_old')
    with pytest.raises(ValueError, match='directory identity changed'):
        cleanup_hygiene(plan, lambda: _view())
    assert (root / 'fastrtps_old').is_file()


def test_new_companion_blocks_old_segment_removal(tmp_path: Path) -> None:
    """A changed pair after planning cannot be treated as the old orphan group."""
    _files(tmp_path, 'fastrtps_old')
    plan = plan_hygiene(tmp_path, _view())
    _files(tmp_path, 'fastrtps_old_el')
    assert not cleanup_hygiene(plan, lambda: _view()).removed


def test_failed_unlink_retains_companion_and_reports_error(tmp_path: Path, monkeypatch) -> None:
    """An unsuccessful first unlink does not trigger cleanup of the other pair member."""
    _files(tmp_path, 'fastrtps_old', 'fastrtps_old_el')
    plan = plan_hygiene(tmp_path, _view())
    original = Path.unlink

    def denied(path, *args, **kwargs):
        if path.name == 'fastrtps_old':
            raise PermissionError('fixture denial')
        return original(path, *args, **kwargs)

    monkeypatch.setattr(Path, 'unlink', denied)
    result = cleanup_hygiene(plan, lambda: _view())
    assert not result.removed
    assert result.errors
    assert (tmp_path / 'fastrtps_old_el').is_file()


def test_symlinked_root_is_rejected(tmp_path: Path) -> None:
    """The cleanup root must be the actual explicitly supplied directory."""
    actual = tmp_path / 'actual'
    actual.mkdir()
    link = tmp_path / 'link'
    link.symlink_to(actual, target_is_directory=True)
    with pytest.raises(ValueError, match='directory'):
        plan_hygiene(link, _view())


def test_maps_reader_preserves_deleted_and_lock_companion_names(tmp_path: Path) -> None:
    """Read-only proc parsing retains conservative evidence, including deleted mappings."""
    process = tmp_path / '123'
    process.mkdir()
    (process / 'maps').write_text(
        '1000-2000 rw-s 0000 00:01 10 /dev/shm/fastrtps_old (deleted)\n'
        '2000-3000 rw-s 0000 00:01 11 /dev/shm/fastrtps_live_el\n')
    view = snapshot_mappings(tmp_path)
    assert view.complete
    assert view.names == frozenset({'fastrtps_old', 'fastrtps_live_el'})


def test_unreadable_maps_produce_incomplete_view(tmp_path: Path, monkeypatch) -> None:
    """An inaccessible process prevents proving that any segment is orphaned."""
    process = tmp_path / '123'
    process.mkdir()
    (process / 'maps').write_text('')
    original = Path.read_text

    def denied(path, *args, **kwargs):
        if path == process / 'maps':
            raise PermissionError('fixture denial')
        return original(path, *args, **kwargs)

    monkeypatch.setattr(Path, 'read_text', denied)
    view = snapshot_mappings(tmp_path)
    assert not view.complete
    assert view.issues


def test_new_process_during_maps_scan_makes_view_incomplete(tmp_path: Path, monkeypatch) -> None:
    """An unobserved new participant cannot be silently omitted from the deletion proof."""
    first = tmp_path / '123'
    first.mkdir()
    (first / 'maps').write_text('')
    original = Path.read_text

    def spawn(path, *args, **kwargs):
        result = original(path, *args, **kwargs)
        new = tmp_path / '456'
        new.mkdir(exist_ok=True)
        return result

    monkeypatch.setattr(Path, 'read_text', spawn)
    assert not snapshot_mappings(tmp_path).complete
