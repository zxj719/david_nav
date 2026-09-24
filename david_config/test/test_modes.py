"""Mode overlays preserve one Nav2 base and only declare real differences."""

import math
from pathlib import Path

import pytest
import yaml

from nav_bringup.navcfg import cli, render

PACKAGE_DIR = Path(__file__).resolve().parents[1]
SOURCE_DIR = PACKAGE_DIR.parent
PROFILE = PACKAGE_DIR / 'robot/david.yaml'
MODES_DIR = PACKAGE_DIR / 'modes'
NAV2_CONFIG_DIR = SOURCE_DIR / 'nav_bringup/config'


def _load(path):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


def _render(mode):
    return render.render_files(
        NAV2_CONFIG_DIR / 'nav2_base.yaml', NAV2_CONFIG_DIR / 'nav2_bindings.yaml',
        PROFILE, MODES_DIR / f'{mode}.yaml')


def test_mode_files_are_named_after_their_modes():
    """Only supported modes exist and each filename matches its declared name."""
    names = sorted(path.stem for path in MODES_DIR.glob('*.yaml'))
    assert names == ['chassis', 'precise', 'teleop', 'tour']
    for name in names:
        assert render.parse_mode(_load(MODES_DIR / f'{name}.yaml'), name).name == name


@pytest.mark.parametrize('mode', ['tour', 'precise'])
def test_navigation_modes_use_one_static_default_pipeline(mode):
    """Both navigation modes inherit the same obstacle-free base layer list."""
    tree = _render(mode)
    local = tree['local_costmap']['local_costmap']['ros__parameters']
    global_ = tree['global_costmap']['global_costmap']['ros__parameters']
    assert local['plugins'] == global_['plugins'] == ['static_layer', 'inflation_layer']
    assert 'obstacle_layer' not in local and 'obstacle_layer' not in global_
    assert 'scan_priority_layer' not in local


@pytest.mark.parametrize('mode', ['tour', 'precise'])
def test_goal_checks_are_consistent(mode):
    """Each mode keeps controller and goal-checker tolerances compatible."""
    controller = _render(mode)['controller_server']['ros__parameters']
    checker = controller['general_goal_checker']
    follow_path = controller['FollowPath']
    assert checker['xy_goal_tolerance'] == follow_path['xy_goal_tolerance']
    assert checker['yaw_goal_tolerance'] >= math.tau or 'RotateToGoal' in follow_path['critics']


def test_teleop_routes_keyboard_through_the_adapter():
    """Teleoperation feeds the shared adapter within its hard speed limits."""
    overlay = render.parse_mode(_load(MODES_DIR / 'teleop.yaml'), 'teleop.yaml')
    profile = render.deep_merge(_load(PROFILE), overlay.profile)
    assert not overlay.nav2_overrides
    assert profile['chassis']['adapter']['input_topic'] == profile['teleop']['cmd_topic']
    assert profile['teleop']['max_linear_x'] <= profile['chassis']['adapter']['hard_max_linear_x']
    assert profile['teleop']['max_angular_z'] <= profile['chassis']['adapter'][
        'hard_max_angular_z']


def test_navcfg_render_finds_the_installed_base(tmp_path):
    """The public renderer produces the same tree as the Python API."""
    output = tmp_path / 'nav2_tour.yaml'
    assert cli.main(['render', '--profile', str(PROFILE), '--mode',
                     str(MODES_DIR / 'tour.yaml'), '--output', str(output)]) == 0
    assert _load(output) == _render('tour')
