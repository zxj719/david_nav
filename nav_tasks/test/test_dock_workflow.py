"""Dock orchestration contracts with in-memory actions, maps and injected time only."""

from dataclasses import replace
import math

import pytest

from nav_tasks.domain.footprint import FootprintCheck, OccupancyGridView
from nav_tasks.skills.docking.approach import check_approach_footprint
from nav_tasks.skills.docking.finish import DockTolerance
from nav_tasks.skills.docking.ports import DockActionResult
from nav_tasks.skills.docking.settle import SettleSpec
from nav_tasks.skills.docking.workflow import DockConfig, DockWorkflow


class _Clock:
    def __init__(self):
        self.value = 0.0

    def now(self):
        return self.value

    def sleep(self, duration):
        self.value += duration


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(('info', text))

    def warn(self, text):
        self.lines.append(('warn', text))

    def error(self, text):
        self.lines.append(('error', text))


class _Motion:
    def __init__(self, outcomes=None):
        self.calls = []
        self.outcomes = list(outcomes or [DockActionResult(True), DockActionResult(True)])
        self.on_spin = lambda: None

    def navigate(self, pose, label, timeout_sec, extra_distance):
        self.calls.append(('navigate', pose, label, timeout_sec, extra_distance))
        return self.outcomes.pop(0)

    def spin(self, yaw, label, time_allowance_sec, timeout_sec):
        self.calls.append(('spin', yaw, label, time_allowance_sec, timeout_sec))
        self.on_spin()
        return DockActionResult(True)


def _config():
    return DockConfig(
        approach_dist=0.5, tolerance=DockTolerance(0.1, math.radians(25), 0),
        settle_spec=SettleSpec(), nav_timeout_sec=120, spin_timeout_sec=30, settle_timeout_sec=8)


def _workflow(*, pose=(2.0, 3.0, 0.0), outcomes=None, check=None, config=None):
    motion, log, clock = _Motion(outcomes), _Log(), _Clock()
    pose_fn = pose if callable(pose) else lambda: pose
    flow = DockWorkflow(
        config or _config(), motion, pose_fn,
        check or (lambda point: FootprintCheck(True, 'clear')), log,
        now_fn=clock.now, sleep_fn=clock.sleep)
    return flow, motion, log, clock


def test_two_stages_use_exact_approach_and_wait_before_success():
    """The stage-one result is not the Dock result; success follows measured settling."""
    flow, motion, log, clock = _workflow()
    assert flow.execute((2, 3, 0)) == 'succeeded'
    assert motion.calls == [
        ('navigate', (1.5, 3.0, 0), '阶段1(预备点)', 120, 0.5),
        ('navigate', (2, 3, 0), '阶段2(入位)', 120, 0.0)]
    assert clock.value == pytest.approx(0.4)
    assert log.lines[-1] == ('info', '[dock] ===== 成功: 一次入位达标'
                             '  (实测 exy=0.000m eyaw=0.0°)')


@pytest.mark.parametrize('reason', [
    'static_map_not_received', 'static_map_frame_mismatch:other!=world',
    'footprint_outside_static_map', 'invalid_static_map', 'occupied_static_cell'])
def test_approach_failure_prevents_all_actions(reason):
    """Full-footprint approval must precede even the first stage request."""
    result = FootprintCheck(
        False, reason, (4, 5, 100) if reason == 'occupied_static_cell' else None)
    flow, motion, log, clock = _workflow(check=lambda pose: result)
    assert flow.execute((2, 3, 0)) == 'failed'
    assert motion.calls == [] and clock.value == 0
    expected = reason + (' cell=(4,5) value=100' if result.occupied_cell else '')
    assert log.lines[-1] == (
        'error', '[dock] ===== 失败: 预备点车身轮廓静态地图校验失败: ' + expected)


def test_static_guard_preserves_missing_frame_rotation_and_area_rules():
    """Use the complete static footprint, not ordinary goal snapping or a center-cell test."""
    shape = ((-0.4, -0.4), (0.4, -0.4), (0.4, 0.4), (-0.4, 0.4))
    grid = OccupancyGridView('world', 10, 10, 1.0, 10, 10, math.pi / 2, (0,) * 100)
    assert check_approach_footprint(None, 'world', shape, (0, 0, 0), 50).reason == (
        'static_map_not_received')
    assert check_approach_footprint(grid, 'other', shape, (0, 0, 0), 50).reason == (
        'static_map_frame_mismatch:world!=other')
    pose = (5.5, 14.5, math.pi / 2)  # Grid-local (4.5, 4.5).
    assert check_approach_footprint(grid, 'world', shape, pose, 50).clear
    cells = list(grid.data)
    cells[4 * 10 + 4] = 100
    occupied = replace(grid, data=tuple(cells))
    result = check_approach_footprint(occupied, 'world', shape, pose, 50)
    assert result.occupied_cell == (4, 4, 100)
    assert not check_approach_footprint(grid, 'world', shape, (0, 0, 0), 50).clear
