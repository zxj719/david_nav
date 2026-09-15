"""Mode overlays reproduce the Nav2 parameter files of today (PROGRESS P2.2c, P2.3)."""

import math
from pathlib import Path

import pytest
import yaml

from nav_bringup.navcfg import cli
from nav_bringup.navcfg import render

PACKAGE_DIR = Path(__file__).resolve().parents[1]
SOURCE_DIR = PACKAGE_DIR.parent
PROFILE = PACKAGE_DIR / 'robot' / 'david.yaml'
MODES_DIR = PACKAGE_DIR / 'modes'
NAV2_CONFIG_DIR = SOURCE_DIR / 'nav_bringup' / 'config'
PRODUCTION_FILES = {
    'tour': 'nav2_params_2d.yaml',
    'precise': 'nav2_params_2d_precise.yaml',
    'bench_slamgt': 'nav2_params_2d_bench_slamgt.yaml',
}
FOOTPRINT_PATHS = (
    'local_costmap/local_costmap/ros__parameters/footprint',
    'global_costmap/global_costmap/ros__parameters/footprint',
)
PRODUCTION_LAUNCH_OVERRIDES = {
    'local_costmap/local_costmap/ros__parameters/scan_priority_layer/require_live_scan':
        False,
    'local_costmap/local_costmap/ros__parameters/static_layer/map_topic': '/map',
    'global_costmap/global_costmap/ros__parameters/static_layer/map_topic': '/map',
    'velocity_smoother/ros__parameters/feedback': 'OPEN_LOOP',
    'velocity_smoother/ros__parameters/smoothing_frequency': 20.0,
    'velocity_smoother/ros__parameters/max_accel': [99.0, 0.0, 99.0],
    'velocity_smoother/ros__parameters/max_decel': [-99.0, 0.0, -99.0],
}


def _load(path):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


def _render(mode):
    return render.render_files(
        NAV2_CONFIG_DIR / 'nav2_base.yaml', NAV2_CONFIG_DIR / 'nav2_bindings.yaml', PROFILE,
        MODES_DIR / f'{mode}.yaml')


def _with_parsed_footprints(tree):
    for path in FOOTPRINT_PATHS:
        render.set_path(tree, path, yaml.safe_load(render.get_path(tree, path)))
    return tree


def _production_parameters(mode):
    tree = _load(SOURCE_DIR / 'david_nav' / 'config' / PRODUCTION_FILES[mode])
    if mode in ('tour', 'precise'):
        for path, value in PRODUCTION_LAUNCH_OVERRIDES.items():
            render.set_path(tree, path, value)
    return tree


@pytest.mark.parametrize('mode', sorted(PRODUCTION_FILES))
def test_rendered_parameters_equal_the_production_file(mode):
    """Rendering the profile with a mode gives the Nav2 parameter file of that mode.

    Footprints are compared as polygons because the rendered string spells the
    numbers without trailing zeros.
    """
    production = _production_parameters(mode)

    rendered = _render(mode)

    assert _with_parsed_footprints(rendered) == _with_parsed_footprints(production)


def test_mode_files_are_named_after_their_modes():
    """The four plan modes exist and each file declares the mode of its name."""
    names = sorted(path.stem for path in MODES_DIR.glob('*.yaml'))

    assert names == ['bench_slamgt', 'precise', 'teleop', 'tour']
    for name in names:
        assert render.parse_mode(_load(MODES_DIR / f'{name}.yaml'), name).name == name


@pytest.mark.parametrize('mode', sorted(PRODUCTION_FILES))
def test_goal_checks_are_consistent(mode):
    """Both xy tolerances agree; a yaw tolerance below 2π needs RotateToGoal (plan §12.4)."""
    controller = _render(mode)['controller_server']['ros__parameters']
    checker = controller['general_goal_checker']
    follow_path = controller['FollowPath']

    assert checker['xy_goal_tolerance'] == follow_path['xy_goal_tolerance']
    assert checker['yaw_goal_tolerance'] >= math.tau or 'RotateToGoal' in follow_path['critics']


def test_teleop_gives_the_adapter_the_david_move_keyboard_values():
    """In teleop the chassis adapter takes the david_move values of david_cmd/config/cmd.yaml."""
    david_move = _load(SOURCE_DIR / 'david_cmd' / 'config' / 'cmd.yaml')['david_move'][
        'ros__parameters']
    overlay = render.parse_mode(_load(MODES_DIR / 'teleop.yaml'), 'teleop.yaml')
    profile = render.deep_merge(_load(PROFILE), overlay.profile)
    adapter = profile['chassis']['adapter']

    assert not overlay.nav2_overrides
    assert david_move['input_cmd_vel_topic'] == adapter['input_topic']
    assert david_move['output_cmd_vel_topic'] == profile['chassis']['cmd_topic']
    assert david_move['cmd_vel_timeout'] == adapter['cmd_timeout_sec']
    assert david_move['min_linear_x'] == adapter['min_linear_x']
    assert david_move['max_linear_x'] == adapter['hard_max_linear_x']
    assert david_move['min_angular_z'] == adapter['min_angular_z']
    assert david_move['min_stationary_angular_z'] == adapter['min_stationary_angular_z']
    assert david_move['stationary_angular_threshold'] == adapter['stationary_angular_threshold']
    assert david_move['max_angular_z'] == adapter['hard_max_angular_z']
    assert david_move['publish_hz'] == adapter['publish_hz']
    assert profile['teleop']['max_linear_x'] <= adapter['hard_max_linear_x']
    assert profile['teleop']['max_angular_z'] <= adapter['hard_max_angular_z']


def test_navcfg_render_finds_the_installed_base(tmp_path):
    """The command line renders with the base tree and binding table installed by nav_bringup."""
    output = tmp_path / 'nav2_tour.yaml'

    status = cli.main([
        'render', '--profile', str(PROFILE), '--mode', str(MODES_DIR / 'tour.yaml'),
        '--output', str(output)])

    assert status == 0
    assert _load(output) == _render('tour')
