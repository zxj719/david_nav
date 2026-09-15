"""Pure grid-search boundary tests independent of ROS message subscriptions."""

from dataclasses import replace
from types import SimpleNamespace

import pytest

from nav_tasks.core.goal_resolution import DirectGoalResolver, GridGoalResolver, SnapSearch
from nav_tasks.domain.grid import GridMapView
from nav_tasks.domain.tasks import ResolvedGoal
from nav_tasks.domain.types import PoseRecord, StoredPoi


def _grid(value=0):
    return GridMapView([value] * 100, 10, 10, 0.25, -1.25, -1.25)


@pytest.mark.parametrize('grid,costmap', [(None, None), (None, _grid(100)), (_grid(100), None)])
def test_missing_grid_retains_legacy_direct_anchor_policy(grid, costmap):
    """Extraction does not silently change permissive missing-map behavior."""
    search = SnapSearch()
    assert search.resolve((0, 0), None, grid, costmap) == (0, 0)


def test_unknown_inflated_occupied_outside_and_clearance_boundaries():
    """Only zero is free; even clearance bounding-square map overflow rejects."""
    search = SnapSearch(goal_clearance_radius=0)
    grid = _grid()
    for value in (-1, 1, 100):
        assert not search.is_admissible((0, 0), grid, _grid(value))
    assert search.is_admissible((0, 0), grid, grid)
    assert not search.is_admissible((100, 0), grid, grid)
    edge = grid.cell_center_world(0, 0)
    assert search.is_admissible(edge, grid, grid)
    # Legacy checks bounds before circular distance, including outside corners.
    assert not SnapSearch(goal_clearance_radius=0.1).is_admissible(edge, grid, grid)


def test_exhausted_search_and_legacy_config_clamps():
    """Honor fallback upper bound and minimum step without changing ring policy."""
    search = SnapSearch(max_snap_radius=0.2, fallback_snap_radius=0.3, snap_step=0.01)
    assert search.max_snap_radius == 0.3
    assert search.snap_step == 0.05
    assert search.resolve((0, 0), None, _grid(100), _grid(100)) is None
    assert search.resolve((100, 100), None, _grid(), _grid()) is None


def test_grid_resolver_preserves_poi_fields_and_rechecks_fresh_grid():
    """Only snapped global x/y change; odometry, orientation and session survive."""
    grid = _grid()
    costmap = _grid()
    costmap.cells[5 * 10 + 5] = 100
    snapshots = [grid, costmap]
    pose = PoseRecord('world', 0.125, 0.125, 0.7, 1.2, 55)
    poi = StoredPoi('poi', '位置', pose, replace(pose, frame_id='odom_fixture'), 'session')
    resolver = GridGoalResolver(
        lambda: tuple(snapshots), SimpleNamespace(current_map_pose=lambda: pose),
        SnapSearch(goal_clearance_radius=0))
    result = resolver.resolve(poi)
    assert result.used_snap
    assert result.poi.odom_pose is poi.odom_pose
    assert result.poi.slam_session_id == poi.slam_session_id
    assert (result.poi.map_pose.yaw, result.poi.map_pose.z, result.poi.map_pose.stamp_sec) == (
        1.2, 0.7, 55)
    assert (result.poi.poi_id, result.poi.name) == (poi.poi_id, poi.name)
    assert not resolver.is_admissible(poi)
    snapshots[1] = grid
    assert resolver.is_admissible(poi)
    assert resolver.resolve(poi) == ResolvedGoal(poi, False)


def test_missing_pose_only_disables_candidate_sorting():
    """Pose capture failure keeps the original row-major fallback order."""
    grid = _grid()
    costmap = _grid()
    costmap.cells[5 * 10 + 5] = 100
    pose = PoseRecord('world', 0.125, 0.125, 0, 0, 0)
    poi = StoredPoi('poi', 'name', pose, pose, 'session')

    def unavailable():
        raise RuntimeError('no pose')

    search = SnapSearch(goal_clearance_radius=0)
    resolver = GridGoalResolver(lambda: (grid, costmap), SimpleNamespace(
        current_map_pose=unavailable), search)
    actual = resolver.resolve(poi)
    assert (actual.poi.map_pose.x, actual.poi.map_pose.y) == search.resolve(
        (pose.x, pose.y), None, grid, costmap)
    assert DirectGoalResolver().resolve(poi) == ResolvedGoal(poi, False)
    assert DirectGoalResolver().is_admissible(poi)
