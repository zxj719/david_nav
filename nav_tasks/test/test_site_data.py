"""Site data persistence, configured frames and source ownership contracts."""

from dataclasses import replace
import json
from pathlib import Path

import pytest

from nav_tasks.domain.types import PoseRecord, StoredPoi, Transform2D
from nav_tasks.nav_site_data.named_poses_importer import import_named_poses
from nav_tasks.nav_site_data import yaml_poi_repository as storage


def _poi(key='p', session='live'):
    pose = PoseRecord('world', 1.0, 2.0, 0.5, 3.0, 42.0)
    return StoredPoi(key, '大厅', pose, replace(pose, frame_id='local'), session)


def _store(tmp_path):
    return storage.YamlPoiRepository(tmp_path / 'store.yaml', map_frame='world')


def test_reload_round_trip_and_unknown_lookup(tmp_path):
    """Immutable fields and Unicode names survive persistence and reload."""
    store = _store(tmp_path)
    store.load()
    assert store.get('missing') is None
    assert store.upsert(_poi()) == _poi()
    reloaded = _store(tmp_path)
    reloaded.load()
    assert reloaded.get('p') == _poi()
    assert '大厅' in store.path.read_text()


def test_source_is_read_only_and_local_overlay_wins(tmp_path):
    """Local records override source IDs without modifying source bytes."""
    source = tmp_path / 'source.yaml'
    source.write_text('pois:\n- id: shared\n  name: source\n- id: source_only\n')
    original = source.read_bytes()
    store = storage.YamlPoiRepository(tmp_path / 'store.yaml', source, map_frame='world')
    store.upsert(_poi('shared'))
    store.load()
    assert store.get('shared') == _poi('shared')
    assert store.get('source_only').map_pose.frame_id == 'world'
    # The old loader uses the same global fallback for missing odom frame too.
    assert store.get('source_only').odom_pose.frame_id == 'world'
    store.rename_many([('source_only', 'renamed'), ('missing', 'ignored')])
    assert store.get('source_only').name == 'renamed'
    assert source.read_bytes() == original


@pytest.mark.parametrize('failure', ['serialize', 'replace'])
def test_failed_write_keeps_previous_complete_disk_file(tmp_path, monkeypatch, failure):
    """Disk replacement is atomic; legacy memory-before-write behavior stays explicit."""
    store = _store(tmp_path)
    store.upsert(_poi())
    before = store.path.read_bytes()
    updated = replace(_poi(), name='new')

    def fail(*args, **kwargs):
        if failure == 'serialize':
            args[1].write('partial temporary content')
        raise OSError('injected write failure')

    if failure == 'serialize':
        monkeypatch.setattr(storage.yaml, 'safe_dump', fail)
    else:
        monkeypatch.setattr(storage.os, 'replace', fail)
    with pytest.raises(OSError, match='injected'):
        store.upsert(updated)
    assert store.path.read_bytes() == before
    assert store.get('p') == updated
    reloaded = _store(tmp_path)
    reloaded.load()
    assert reloaded.get('p') == _poi()


def test_replace_receives_closed_sibling_file_and_complete_yaml(tmp_path, monkeypatch):
    """The destination is not truncated before successful atomic replacement."""
    store = _store(tmp_path)
    original_replace = storage.os.replace
    observed = []

    def replace_file(source, target):
        assert Path(source).parent == Path(target).parent
        payload = storage.yaml.safe_load(Path(source).read_text())
        observed.append(payload['pois'][0]['id'])
        original_replace(source, target)

    monkeypatch.setattr(storage.os, 'replace', replace_file)
    store.upsert(_poi())
    assert observed == ['p']
    assert not store.path.with_suffix('.yaml.tmp').exists()


def test_reprojection_keeps_raw_yaw_configured_frame_height_and_stamp(tmp_path):
    """Crossing pi must not change existing persisted angles or threshold semantics."""
    store = _store(tmp_path)
    store.upsert(_poi())
    result = store.reproject_session('live', Transform2D(2, 3, 1), 0, 0)
    assert len(result) == 1
    assert result[0].map_pose.yaw == 4.0
    assert result[0].map_pose.frame_id == 'world'
    assert result[0].map_pose.z == 0.5
    assert result[0].map_pose.stamp_sec == 42.0
    assert result[0].odom_pose == _poi().odom_pose
    before = store.path.read_bytes()
    assert store.reproject_session('live', Transform2D(2, 3, 1), 0, 0) == []
    assert store.path.read_bytes() == before


def test_prune_only_removes_matching_session_and_persists(tmp_path):
    """Manual-source pruning cannot delete measured live-session anchors."""
    store = _store(tmp_path)
    for poi in (_poi('keep', 'named_poses'), _poi('stale', 'named_poses'), _poi('live')):
        store.upsert(poi)
    assert store.prune_session('named_poses', {'keep'}) == ['stale']
    store.load()
    assert store.get('stale') is None
    assert store.get('keep') is not None
    assert store.get('live') is not None


@pytest.mark.parametrize('shape', ['list', 'poses', 'named_poses', 'fallback'])
def test_named_source_shapes_duplicate_ids_and_pruning(tmp_path, shape):
    """JSON is read-only; duplicates count separately and last record wins."""
    store = _store(tmp_path)
    store.upsert(_poi('stale', 'named_poses'))
    store.upsert(_poi('live'))
    items = [None, {'id': 'p', 'name': 'first'},
             {'id': 'p', 'name': '最后', 'frame_id': 'manual',
              'position': {'x': 4, 'y': 5, 'z': 6}, 'yaw': 7}]
    payload = items if shape == 'list' else {shape: items}
    if shape == 'fallback':
        payload = {'poses': [], 'named_poses': items}
    source = tmp_path / 'named.json'
    source.write_text(json.dumps(payload, ensure_ascii=False))
    before = source.read_bytes()
    lines = []
    assert import_named_poses(store, source, map_frame='world', odom_frame='local',
                              log=lines.append) == 2
    assert store.get('p').name == '最后'
    assert store.get('p').map_pose == PoseRecord('manual', 4, 5, 6, 7, 0)
    assert store.get('p').odom_pose.frame_id == 'local'
    assert store.get('live') is not None
    assert store.get('stale') is None
    assert lines == ["pruned stale named_poses POIs: ['stale']"]
    assert source.read_bytes() == before


def test_failed_import_keeps_legacy_partial_writes_without_pruning(tmp_path):
    """A malformed later record does not silently prune or roll back prior imports."""
    store = _store(tmp_path)
    store.upsert(_poi('stale', 'named_poses'))
    source = tmp_path / 'named.json'
    source.write_text('[{"id": "good"}, {"name": "missing ID"}]')
    with pytest.raises(KeyError):
        import_named_poses(store, source, map_frame='world', odom_frame='local')
    store.load()
    assert store.get('good') is not None
    assert store.get('stale') is not None
