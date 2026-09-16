"""Regression tests for dock approach-map checks and recovery-tree control flow."""

from dataclasses import replace
from pathlib import Path
import xml.etree.ElementTree as ET

from geometry_msgs.msg import PoseStamped
import pytest

from legacy_entry_reference import load_entry
from david_nav.dock_approach import FootprintCheck
from david_nav.dock_approach import OccupancyGridView
from david_nav.dock_approach import check_footprint
from david_nav.dock_approach import parse_footprint
from david_nav.dock_settle import SettleSpec

dock_navigator = load_entry('dock_navigator')

PACKAGE_DIR = Path(__file__).resolve().parents[1]
FOOTPRINT = parse_footprint('[[-0.4, -0.4], [0.4, -0.4], [0.4, 0.4], [-0.4, 0.4]]')


def _grid(occupied=(), unknown=()):
    data = [0] * 100
    for x, y in occupied:
        data[y * 10 + x] = 100
    for x, y in unknown:
        data[y * 10 + x] = -1
    return OccupancyGridView('map', 10, 10, 1.0, 0.0, 0.0, 0.0, tuple(data))


def test_occupied_cell_anywhere_under_approach_footprint_is_rejected():
    """An occupied cell intersecting the footprint interior blocks docking."""
    result = check_footprint(_grid(occupied=[(4, 4)]), FOOTPRINT, (4.5, 4.5, 0.0))

    assert result == FootprintCheck(False, 'occupied_static_cell', (4, 4, 100))


def test_rotated_footprint_checks_cells_crossed_only_by_an_edge():
    """Collision checks include cell-area intersections, not only cell centers."""
    narrow = parse_footprint('[[-1.2, -0.1], [1.2, -0.1], [1.2, 0.1], [-1.2, 0.1]]')

    result = check_footprint(
        _grid(occupied=[(5, 5)]), narrow, (4.5, 4.5, 0.7853981633974483))

    assert not result.clear
    assert result.occupied_cell == (5, 5, 100)


def test_unknown_cells_are_allowed_but_outside_map_is_rejected():
    """Only occupied cells block inside the map; unverifiable outside area also blocks."""
    assert check_footprint(
        _grid(unknown=[(4, 4)]), FOOTPRINT, (4.5, 4.5, 0.0)).clear
    assert check_footprint(
        _grid(), FOOTPRINT, (0.1, 0.1, 0.0)).reason == 'footprint_outside_static_map'


@pytest.mark.parametrize('cell', [(3, 4), (5, 4), (4, 3), (4, 5)])
def test_touching_an_occupied_cell_boundary_is_rejected_on_every_side(cell):
    """Closed footprint/cell intersection is symmetric at lower and upper grid lines."""
    square = ((-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5))
    result = check_footprint(_grid(occupied=[cell]), square, (4.5, 4.5, 0.0))
    assert result.reason == 'occupied_static_cell'


@pytest.mark.parametrize('field', ['resolution', 'origin_x', 'origin_y', 'origin_yaw'])
@pytest.mark.parametrize('value', [float('nan'), float('inf')])
def test_nonfinite_map_metadata_fails_closed(field, value):
    """Malformed map geometry returns a rejection instead of raising or approving."""
    result = check_footprint(replace(_grid(), **{field: value}), FOOTPRINT, (4.5, 4.5, 0.0))
    assert result == FootprintCheck(False, 'invalid_static_map')


@pytest.mark.parametrize('index', [0, 1, 2])
@pytest.mark.parametrize('value', [float('nan'), float('inf')])
def test_nonfinite_pose_fails_closed(index, value):
    """A nonfinite approach pose must never produce a navigation target."""
    pose = [4.5, 4.5, 0.0]
    pose[index] = value
    assert check_footprint(_grid(), FOOTPRINT, pose) == FootprintCheck(False, 'invalid_pose')


@pytest.mark.parametrize('polygon', [
    ((0.0, 0.0), (float('inf'), 0.0), (1.0, 1.0)),
    ((0.0, 0.0), (float('nan'), 0.0), (1.0, 1.0)),
    ((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)),
    ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 0.0)),
    ((0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (1.0, 0.0)),
])
def test_nonfinite_or_degenerate_footprint_fails_closed(polygon):
    """Invalid body geometry cannot be interpreted as a clear footprint."""
    assert check_footprint(_grid(), polygon, (4.5, 4.5, 0.0)).reason == 'invalid_footprint'


def test_parser_rejects_nonfinite_coordinates():
    """A numeric literal overflowing to infinity is an invalid footprint parameter."""
    with pytest.raises(ValueError):
        parse_footprint('[[0, 0], [1e309, 0], [0, 1]]')


def test_thin_rotated_footprint_crossing_a_small_cell_is_detected():
    """Small segment orientation products must not suppress a genuine crossing."""
    narrow = ((-0.002, -0.00001), (0.002, -0.00001),
              (0.002, 0.00001), (-0.002, 0.00001))
    grid = replace(_grid(occupied=[(4, 4)]), resolution=0.001)
    assert not check_footprint(grid, narrow, (0.0045, 0.0045, 0.7853981633974483)).clear


def test_default_settle_rate_allows_observed_static_yaw_jitter():
    """The operational angular settle threshold is relaxed to about 0.05 rad/s."""
    assert SettleSpec().max_rate_rad_s == 0.05


def test_recovery_guard_retries_the_whole_navigation_pipeline():
    """Escape success cannot finish the goal; it retries planning and FollowPath."""
    names = (
        'navigate_to_pose_w_david_embedded_recovery.xml',
        'navigate_through_poses_w_david_embedded_recovery.xml',
    )
    for name in names:
        root = ET.parse(PACKAGE_DIR / 'behavior_trees' / name).getroot()
        guard = root.find(".//RecoveryNode[@name='EmbeddedRecoveryGuard']")
        assert guard is not None and len(guard) == 2
        pipeline, recovery = guard
        assert pipeline.tag == 'PipelineSequence'
        assert pipeline.find('./Inverter/RobotEmbeddedInObstacle') is not None
        assert pipeline.find('.//FollowPath') is not None
        assert recovery.find('.//EscapeObstacle') is not None


class _Logger:
    def info(self, _message):
        pass


class _BlockedDock:
    approach_dist = 0.5

    def __init__(self):
        self.reported = None
        self.action_calls = 0

    def get_logger(self):
        return _Logger()

    def _check_approach_footprint(self, _x, _y, _yaw):
        return FootprintCheck(False, 'occupied_static_cell', (3, 4, 100))

    def report(self, ok, message):
        self.reported = (ok, message)

    def run_action(self, *_args, **_kwargs):
        self.action_calls += 1
        raise AssertionError('a blocked approach must not send a Nav2 goal')


def test_dock_flow_checks_approach_before_sending_stage_one():
    """A blocked pre-dock footprint aborts before any NavigateToPose request."""
    dock = _BlockedDock()
    goal = PoseStamped()
    goal.pose.orientation.w = 1.0

    outcome = dock_navigator.DockNavigator.execute_docking(dock, goal)

    assert outcome == 'failed'
    assert dock.action_calls == 0
    assert dock.reported[0] is False
    assert 'occupied_static_cell' in dock.reported[1]
