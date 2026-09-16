"""Compare new repository/importer bytes against the unchanged legacy implementations."""

from dataclasses import replace
import json
import random

import pytest

from legacy_entry_reference import load_entry
from david_nav.poi_store import LocalPoiStore
from nav_tasks.domain.types import PoseRecord, StoredPoi, Transform2D
from nav_tasks.nav_site_data.named_poses_importer import import_named_poses
from nav_tasks.nav_site_data.yaml_poi_repository import YamlPoiRepository

_populate_poi_store_from_named_poses_json = load_entry(
    'navigation_executor')._populate_poi_store_from_named_poses_json


@pytest.mark.parametrize('seed', range(8))
def test_randomized_operations_preserve_stored_values_and_yaml_bytes(tmp_path, seed):
    """Ordered upsert/rename/reprojection/prune produces identical persisted data."""
    rng = random.Random(seed)
    old = LocalPoiStore(tmp_path / 'old.yaml')
    new = YamlPoiRepository(tmp_path / 'new.yaml', map_frame='map')
    keys = []
    for index in range(12):
        key = f'p{index % 9}'
        keys.append(key)
        pose = PoseRecord('map', rng.uniform(-8, 8), rng.uniform(-8, 8), 0.3,
                          rng.uniform(-12, 12), 42.0)
        odom = replace(pose, frame_id='odom', x=rng.uniform(-5, 5))
        poi = StoredPoi(key, f'点位{index}', pose, odom, f's{index % 2}')
        old.upsert_marked_poi(key, poi.name, pose, odom, poi.slam_session_id)
        new.upsert(poi)
        assert old.path.read_bytes() == new.path.read_bytes()
    pairs = [('p2', '门口'), ('missing', 'ignored'), ('p4', '走廊')]
    old.apply_semantic_directory([{'id': key, 'name': name} for key, name in pairs])
    new.rename_many(pairs)
    assert old.path.read_bytes() == new.path.read_bytes()
    for _ in range(10):
        transform = Transform2D(rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-6, 6))
        distance, angle = rng.choice([0, 0.05, 10]), rng.choice([0, 0.03, 12])
        assert old.reproject_session('s0', transform, distance, angle) == (
            new.reproject_session('s0', transform, distance, angle))
        assert old.path.read_bytes() == new.path.read_bytes()
    assert old.prune_session('s1', {'p1', 'p7'}) == new.prune_session('s1', {'p1', 'p7'})
    assert old.path.read_bytes() == new.path.read_bytes()
    old.load()
    new.load()
    for key in keys:
        assert old._pois.get(key) == new.get(key)


@pytest.mark.parametrize('payload', [
    {}, {'pois': None}, {'pois': ['skip', {}, {'id': 3, 'position': {'x': 2}}]},
    [{'id': 'p', 'name': '', 'map_pose': {'x': 1}, 'odom_pose': {'y': 2}}],
    {'pois': [{'id': 'p', 'frame_id': '', 'slam_session_id': '', 'yaw': 4.7}]},
])
def test_loader_legacy_fallbacks_and_ignored_records(tmp_path, payload):
    """Sparse and old flat records retain all fallback values and skip behavior."""
    source = tmp_path / 'input.yaml'
    source.write_text(json.dumps(payload))
    old = LocalPoiStore(source)
    new = YamlPoiRepository(source, map_frame='map')
    old.load()
    new.load()
    assert old._pois == new._pois


@pytest.mark.parametrize('payload', [
    [{'id': 'p', 'position': {'x': 1, 'y': 2}, 'yaw': 7}],
    {'poses': [], 'named_poses': [None, {'id': 1, 'name': None, 'frame_id': None}]},
    {'poses': [{'id': 'p'}, {'id': 'p', 'name': '重复'}]},
])
def test_importer_file_bytes_and_prune_log_match_old_entry(tmp_path, capsys, payload):
    """Manual source semantics, duplicate count and frozen pruning log are unchanged."""
    source = tmp_path / 'named.json'
    source.write_text(json.dumps(payload, ensure_ascii=False))
    before = source.read_bytes()
    old = LocalPoiStore(tmp_path / 'old.yaml')
    new = YamlPoiRepository(tmp_path / 'new.yaml', map_frame='map')
    pose = PoseRecord('map', 0, 0, 0, 0, 0)
    old.upsert_marked_poi('stale', '旧', pose, pose, 'named_poses')
    new.upsert(StoredPoi('stale', '旧', pose, pose, 'named_poses'))
    count = _populate_poi_store_from_named_poses_json(old, str(source))
    old_log = capsys.readouterr().out
    lines = []
    assert import_named_poses(new, source, map_frame='map', odom_frame='odom',
                              log=lines.append) == count
    assert old.path.read_bytes() == new.path.read_bytes()
    assert old_log == ''.join(f'[navigation_executor] {line}\n' for line in lines)
    assert source.read_bytes() == before
