"""Compare extracted snap search against the untouched legacy implementation."""

import random
import sys
from types import ModuleType, SimpleNamespace

import pytest

from david_nav.navigation_goal_resolver import NavigationGoalResolver
from nav_tasks.core.goal_resolution import GridGoalResolver, SnapSearch
from nav_tasks.domain.grid import GridMapView
from nav_tasks.domain.types import PoseRecord, StoredPoi


def _fake_messages(monkeypatch):
    package = ModuleType('nav_msgs')
    module = ModuleType('nav_msgs.msg')
    module.OccupancyGrid = object
    package.msg = module
    monkeypatch.setitem(sys.modules, 'nav_msgs', package)
    monkeypatch.setitem(sys.modules, 'nav_msgs.msg', module)


@pytest.mark.parametrize('seed', range(8))
def test_search_matches_legacy_across_maps_settings_and_candidate_order(monkeypatch, seed):
    """Check seeded free/unknown/inflated grids, edge points and robot ordering."""
    _fake_messages(monkeypatch)
    rng = random.Random(seed)
    for index in range(20):
        resolution = rng.choice([0.1, 0.25, 0.5])
        width, height = 20, 18
        grid = GridMapView(
            rng.choices([0, -1, 100], weights=[9, 1, 1], k=width * height),
            width, height, resolution, -width * resolution / 2, -height * resolution / 2)
        costmap = GridMapView(
            rng.choices([0, 1, -1, 100], weights=[10, 1, 1, 1], k=width * height),
            width, height, resolution, grid.origin_x, grid.origin_y)
        if index % 7 == 0:
            grid = None
        if index % 11 == 0:
            costmap = None
        settings = {
            'snap_radius': rng.choice([0.25, 0.5, 0.75]),
            'fallback_snap_radius': rng.choice([0.5, 1.0]),
            'goal_clearance_radius': rng.choice([0.0, 0.1, 0.3]),
            'max_snap_radius': rng.choice([0.25, 1.5]),
            'snap_step': rng.choice([0.01, 0.25, 0.5]),
        }
        pose = PoseRecord('world', rng.uniform(-2, 2), rng.uniform(-2, 2), 0.4, 1.3, 123)
        poi = StoredPoi('id', '名称', pose, pose, 'session')
        robot_xy = None if index % 3 == 0 else (rng.uniform(-2, 2), rng.uniform(-2, 2))

        def robot_pose():
            if robot_xy is None:
                raise RuntimeError('pose unavailable')
            return SimpleNamespace(x=robot_xy[0], y=robot_xy[1])

        poses = SimpleNamespace(current_map_pose=robot_pose)
        old = NavigationGoalResolver(
            SimpleNamespace(create_subscription=lambda *args: None), poses, **settings)
        # Supply identical immutable views without message conversion noise.
        old._grid_view = lambda view: view
        old._map_msg = grid
        old._global_costmap_msg = costmap
        search = SnapSearch(**settings)
        new = GridGoalResolver(lambda: (grid, costmap), poses, search)
        anchor = (pose.x, pose.y)
        assert search.snap_step == old.snap_step
        assert search.max_snap_radius == old.max_snap_radius
        assert search.is_admissible(anchor, grid, costmap) == old._is_goal_admissible(anchor)
        for radius in (0.25, 0.5, 1.0):
            assert search._ring_candidates(anchor, radius, costmap) == old._ring_candidates(
                anchor, radius)
        expected = old.resolve_initial_goal(poi)
        actual = new.resolve(poi)
        if expected is None:
            assert actual is None
        else:
            assert actual.poi == expected.poi
            assert actual.used_snap == expected.used_snap
        assert new.is_admissible(poi) == old.is_anchor_admissible(poi)
