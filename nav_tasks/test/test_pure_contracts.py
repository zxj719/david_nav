"""Exercise migrated values and settling without ROS, motion or wall-clock waits."""

import math
from pathlib import Path
import subprocess
import sys

from nav_tasks.api import CapturedPose, PoseRecord, StoredPoi, Transform2D
from nav_tasks.domain.grid import GridMapView
from nav_tasks.skills.docking.finish import DockTolerance, finish_docking
from nav_tasks.skills.docking.settle import is_settled, SettleSpec, wait_until_settled


def test_domain_values_keep_constructor_order_and_quaternion_order():
    """Dataclass identity and serialized quaternion order are stable contracts."""
    pose = PoseRecord('fixture', 1.0, 2.0, 0.0, math.pi, 10.0)
    assert list(pose.orientation()) == ['x', 'y', 'z', 'w']
    assert pose.orientation()['z'] == 1.0
    poi = StoredPoi('id', 'name', pose, pose, 'session')
    assert CapturedPose(poi.map_pose, poi.odom_pose, poi.slam_session_id).map_pose == pose
    assert Transform2D(1.0, 2.0, 0.5).yaw == 0.5


def test_grid_keeps_zero_cost_threshold_and_unknown_rules():
    """Grid migration does not add bounds, origin-rotation or occupancy policy changes."""
    grid = GridMapView([0, 1, 100, -1], 2, 2, 0.5, -1.0, -2.0)
    assert grid.world_to_cell(-0.75, -1.75) == (0, 0)
    assert grid.world_to_cell(0.0, -1.0) is None
    assert grid.cell_center_world(1, 1) == (-0.25, -1.25)
    assert grid.is_goal_free(0, 0) and not grid.is_goal_free(1, 0)
    assert grid.is_blocked(0, 1, 100, False)
    assert grid.is_blocked(1, 1, 100, True)
    assert not grid.is_blocked(1, 1, 100, False)


def test_settle_defaults_and_window_boundary_are_unchanged():
    """The requested 0.05 rad/s threshold survives relocation without rounding changes."""
    spec = SettleSpec()
    assert (spec.max_rate_rad_s, spec.window_sec, spec.sample_period_sec) == (0.05, 0.4, 0.05)
    assert is_settled([(0.0, 0.0), (0.4, 0.019)], spec)
    assert not is_settled([(0.0, 0.0), (0.4, 0.021)], spec)
    assert not is_settled([(0.0, 0.0), (0.39, 0.0)], spec)


def test_finish_waits_on_injected_clock_and_does_not_spin_when_settled():
    """Callbacks stay pure test fakes; no goal or command publisher exists."""
    now, spins, events = [0.0], [], []

    def sleep(seconds):
        now[0] += seconds

    outcome = finish_docking(
        (0.0, 0.0, 0.0), lambda: (0.0, 0.0, 0.0), spins.append,
        DockTolerance(), SettleSpec(), 2.0, now_fn=lambda: now[0], sleep_fn=sleep,
        on_event=lambda name, **values: events.append((name, values)))
    assert outcome.ok and outcome.settled and outcome.reason == 'settled_ok'
    assert outcome.n_spin == 0 and not spins
    assert now[0] >= 0.4 - 1e-9
    assert events == [('measured', {'settled': True, 'exy': 0.0, 'eyaw': 0.0})]


def test_settle_timeout_keeps_last_pose():
    """A failed wait still reports its measurement rather than pretending success."""
    assert wait_until_settled(lambda: (1.0, 2.0, 3.0), SettleSpec(), 0.0,
                              now_fn=lambda: 0.0) == (False, (1.0, 2.0, 3.0))


def test_imports_work_without_site_packages():
    """An isolated interpreter proves the pure modules need only the standard library."""
    root = Path(__file__).resolve().parents[1]
    code = (
        'import sys; sys.path.insert(0, ' + repr(str(root)) + '); '
        'import nav_tasks.api, nav_tasks.domain.grid, nav_tasks.skills.docking.finish; '
        'assert not any(n in sys.modules for n in ("rclpy", "yaml", "websockets"))')
    result = subprocess.run([sys.executable, '-I', '-S', '-c', code],
                            capture_output=True, text=True, timeout=10, check=False)
    assert result.returncode == 0, result.stderr
