"""Compare the new blocking workflow with frozen legacy methods and actual bench parsing."""

import ast
from dataclasses import replace
import functools
import math
from pathlib import Path
import runpy
from types import SimpleNamespace as NS

import pytest

from nav_tasks.domain.footprint import FootprintCheck
from nav_tasks.skills.docking.finish import finish_docking
from nav_tasks.skills.docking.ports import DockActionResult
from nav_tasks.skills.docking.workflow import DockWorkflow

from test_dock_workflow import _Clock, _config, _Log

ROOT = Path(__file__).resolve().parents[2]
LEGACY = ROOT / 'david_nav/test/golden/legacy_entries/dock_navigator.py.txt'
BENCH = ROOT / 'david_nav/scripts/dock_bench/dock_common.py'


def _legacy_methods(clock):
    # Extract only pure control-flow methods: no ROS imports, Node or ActionClient.
    module = ast.parse(LEGACY.read_text())
    node = next(item for item in module.body if isinstance(item, ast.ClassDef)
                and item.name == 'DockNavigator')
    methods = [item for item in node.body if isinstance(item, ast.FunctionDef) and
               item.name in ('execute_docking', 'report', '_log_dock_event')]
    assert len(methods) == 3
    constant = next(item.value for item in module.body if isinstance(item, ast.Assign)
                    and any(isinstance(name, ast.Name) and name.id == 'DOCK_FAILURE_REASONS'
                            for name in item.targets))
    namespace = {
        'math': math, 'yaw_of': lambda quaternion: quaternion.yaw,
        'wrap': lambda angle: (angle + math.pi) % (2 * math.pi) - math.pi,
        'NavigateToPose': NS(Goal=lambda: NS()),
        'DOCK_FAILURE_REASONS': ast.literal_eval(constant),
        'finish_docking': functools.partial(
            finish_docking, now_fn=clock.now, sleep_fn=clock.sleep),
    }
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(LEGACY), 'exec'), namespace)
    return type('LegacyMethods', (), {method.name: namespace[method.name] for method in methods})


class _Rig:
    def __init__(self, case, goal):
        self.case, self.goal = case, goal
        self.clock, self.log = _Clock(), _Log()
        self.calls, self.checks = [], []
        self.events = []
        self.rotations = 0
        self.config = _config()
        if case.startswith('spin_'):
            self.config = replace(
                self.config, tolerance=replace(self.config.tolerance, max_yaw_retries=2))

    def pose(self):
        x, y, yaw = self.goal
        if self.case == 'no_pose' or (self.case == 'spin_no_pose' and self.rotations):
            return None
        if self.case == 'moving' or (self.case == 'spin_moving' and self.rotations):
            return x, y, yaw + self.clock.value * 0.5
        if self.case == 'xy_off':
            return x + 0.3, y, yaw
        if self.case == 'yaw_off' or self.case.startswith('spin_'):
            if self.rotations and self.case in (
                    'spin_ok', 'spin_aborted_but_arrived', 'spin_cancel_but_arrived'):
                return self.goal
            return x, y, yaw + 0.7
        return self.goal

    def check(self, pose):
        self.checks.append(pose)
        if self.case == 'blocked':
            return FootprintCheck(False, 'occupied_static_cell', (3, 4, 100))
        return FootprintCheck(True, 'clear')

    def navigate(self, pose, label, timeout_sec, extra_distance):
        self.calls.append(('navigate', pose, label, timeout_sec, extra_distance))
        stage = len(self.calls)
        canceled = self.case == f'stage{stage}_cancel'
        succeeded = not canceled and self.case != f'stage{stage}_failed'
        status = 'SUCCEEDED' if succeeded else 'status=5' if canceled else 'status=6'
        self.log.info('[dock] %s: %s' % (label, status))
        return DockActionResult(succeeded, canceled)

    def spin(self, yaw, label, time_allowance_sec, timeout_sec):
        self.calls.append(('spin', yaw, label, time_allowance_sec, timeout_sec))
        self.rotations += 1
        canceled = self.case in ('spin_cancel', 'spin_cancel_but_arrived')
        succeeded = self.case not in ('spin_failed', 'spin_aborted_but_arrived') and not canceled
        status = 'SUCCEEDED' if succeeded else 'status=5' if canceled else 'status=6'
        self.log.info('[dock] %s: %s' % (label, status))
        return DockActionResult(succeeded, canceled)

    def legacy(self):
        legacy = _legacy_methods(self.clock)()
        config = self.config
        legacy.approach_dist = config.approach_dist
        legacy.nav_timeout = config.nav_timeout_sec
        legacy.spin_timeout = config.spin_timeout_sec
        legacy.settle_timeout = config.settle_timeout_sec
        legacy.xy_ok = config.tolerance.xy_ok
        legacy.dock_tol = config.tolerance
        legacy.settle_spec = config.settle_spec
        legacy.pose_now = self.pose
        legacy._last_run_canceled = False
        legacy.nav_cli = object()
        legacy.get_logger = lambda: self.log
        legacy._check_approach_footprint = lambda x, y, yaw: self.check((x, y, yaw))
        legacy.make_pose = lambda x, y, yaw: (x, y, yaw)
        legacy._feedback_forwarder = lambda publisher, extra: extra

        def navigate(client, goal, label, timeout_sec, feedback):
            result = self.navigate(goal.pose, label, timeout_sec, feedback)
            legacy._last_run_canceled = result.canceled
            return result.succeeded

        def spin(yaw):
            result = self.spin(yaw, 'Spin#%d' % legacy._spin_idx,
                               config.spin_timeout_sec, config.spin_timeout_sec + 5)
            legacy._last_run_canceled = result.canceled
            return result.succeeded

        legacy.run_action, legacy._spin_by = navigate, spin
        x, y, yaw = self.goal
        return legacy.execute_docking(NS(pose=NS(position=NS(x=x, y=y), orientation=NS(yaw=yaw))))

    def migrated(self):
        return DockWorkflow(
            self.config, self, self.pose, self.check, self.log,
            now_fn=self.clock.now, sleep_fn=self.clock.sleep,
            on_event=lambda name, data: self.events.append((name, data))).execute(self.goal)


@pytest.mark.parametrize('case,expected', [
    ('success', 'succeeded'), ('blocked', 'failed'),
    ('stage1_failed', 'failed'), ('stage1_cancel', 'canceled'),
    ('stage2_failed', 'succeeded'), ('stage2_cancel', 'canceled'),
    ('no_pose', 'failed'), ('moving', 'failed'), ('xy_off', 'failed'), ('yaw_off', 'failed'),
    ('spin_ok', 'succeeded'), ('spin_aborted_but_arrived', 'succeeded'),
    ('spin_failed', 'failed'), ('spin_no_pose', 'failed'), ('spin_moving', 'failed'),
    ('spin_cancel', 'canceled'), ('spin_cancel_but_arrived', 'succeeded'),
])
@pytest.mark.parametrize('goal', [(2.0, 3.0, 0.0), (-2.25, 1.3, 1.2), (5.0, -2.0, -math.pi)])
def test_logs_actions_settle_timing_and_bench_verdict_match_legacy(case, expected, goal):
    """All non-ROS workflow observations retain old values, including awkward finish cases."""
    old, new = _Rig(case, goal), _Rig(case, goal)
    assert old.legacy() == new.migrated() == expected
    assert new.events[0][0] == 'started'
    assert old.calls == new.calls
    assert old.checks == new.checks
    assert old.clock.value == new.clock.value
    assert old.log.lines == new.log.lines
    parse = runpy.run_path(str(BENCH))['parse_dock_timeline']
    old_rows = [(index, line) for index, (_, line) in enumerate(old.log.lines)]
    new_rows = [(index, line) for index, (_, line) in enumerate(new.log.lines)]
    assert parse(old_rows) == parse(new_rows)
    if expected == 'succeeded':
        assert parse(new_rows)['verdict_class'] == 'SUCCESS'
    elif case == 'moving':
        assert parse(new_rows)['verdict_class'] == 'NOT_SETTLED'
    elif case == 'xy_off':
        assert parse(new_rows)['verdict_class'] == 'XY_OUT_OF_TOL'


def test_legacy_footprint_exports_are_the_same_implementation():
    """Public identity prevents parallel copies of the mandatory static-map safety check."""
    from nav_tasks.domain import footprint

    old = runpy.run_path(str(ROOT / 'david_nav/david_nav/dock_approach.py'))
    for name in ('FootprintCheck', 'OccupancyGridView', 'parse_footprint', 'check_footprint'):
        assert old[name] is getattr(footprint, name)
