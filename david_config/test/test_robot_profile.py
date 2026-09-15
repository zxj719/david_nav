"""The David robot profile holds the values production runs today (PROGRESS P2.2a).

Values are compared with the effective parameters dumped from the
pre-refactor bringup (``david_nav/test/golden/params``, PROGRESS P0.7) and
with the configuration files and launch files production uses, so a profile
edit that would change behavior fails here. ``test_modes.py`` additionally
renders the Nav2 parameter files from the profile (P2.3).
"""

from pathlib import Path
import re

import pytest
import yaml

from nav_bringup.navcfg import schema

PACKAGE_DIR = Path(__file__).resolve().parents[1]
SOURCE_DIR = PACKAGE_DIR.parent
PROFILE = PACKAGE_DIR / 'robot' / 'david.yaml'
GOLDEN = SOURCE_DIR / 'david_nav' / 'test' / 'golden' / 'params'
MODES = ('tour', 'precise')


def _load(path):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


def _params(mode, node_file):
    (content,) = _load(GOLDEN / mode / node_file).values()
    return content['ros__parameters']


def _launch_number(text, key):
    match = re.search(rf"'{key}':\s*(-?[0-9.]+)", text)
    assert match, f'{key} not found in the launch file'
    return float(match.group(1))


@pytest.fixture(scope='module')
def profile():
    """Load the robot profile."""
    return _load(PROFILE)


def test_limits_match_the_controller(profile):
    """Speed limits equal the controller of tour and precise."""
    for mode in MODES:
        follow_path = _params(mode, 'controller_server.yaml')['FollowPath']
        assert follow_path['max_vel_x'] == profile['limits']['max_vel_x']
        assert follow_path['max_speed_xy'] == profile['limits']['max_vel_x']
        assert follow_path['max_vel_theta'] == profile['limits']['max_vel_theta']


def test_footprint_and_base_frame_match_every_nav2_consumer(profile):
    """Footprint and navigation base frame equal costmaps, BT navigator and behaviors."""
    nav_base = profile['frames']['nav_base']
    costmaps = ('local_costmap__local_costmap.yaml', 'global_costmap__global_costmap.yaml')
    for mode in MODES:
        for costmap in costmaps:
            params = _params(mode, costmap)
            assert yaml.safe_load(params['footprint']) == profile['footprint']
            assert params['robot_base_frame'] == nav_base
        assert _params(mode, 'bt_navigator.yaml')['robot_base_frame'] == nav_base
        assert _params(mode, 'behavior_server.yaml')['robot_base_frame'] == nav_base
        assert _params(mode, 'global_costmap__global_costmap.yaml')['global_frame'] == (
            profile['frames']['map'])
        assert _params(mode, 'local_costmap__local_costmap.yaml')['global_frame'] == (
            profile['frames']['odom'])


def test_base_link_and_base_footprint_stay_apart(profile):
    """The body frame and the navigation base frame are different frames."""
    frames = profile['frames']

    assert (frames['nav_base'], frames['body']) == ('base_footprint', 'base_link')


def test_nav2_odom_topic_keeps_the_current_wiring(profile):
    """Nav2 keeps its current odom topic, which differs from localization (TODO-O1)."""
    for mode in MODES:
        assert _params(mode, 'bt_navigator.yaml')['odom_topic'] == profile['nav2']['odom_topic']
        assert _params(mode, 'controller_server.yaml')['odom_topic'] == (
            profile['nav2']['odom_topic'])
        assert _params(mode, 'velocity_smoother.yaml')['odom_topic'] == (
            profile['chassis']['odom_topic'])
    assert profile['localization']['odom_topic'] != profile['nav2']['odom_topic']


def test_tour_goal_tolerances_and_scan_topic_match_the_effective_parameters(profile):
    """Goal tolerances and the costmap scan topic equal the effective tour parameters.

    The static layer map topic is rewritten by the bringup launch (effective
    value ``/map``), so ``nav2.map_topic`` is only compared with the parameter
    files, in ``test_modes.py``; comparing rendered and effective parameters
    is PROGRESS P2.4.
    """
    checker = _params('tour', 'controller_server.yaml')['general_goal_checker']
    local_costmap = _params('tour', 'local_costmap__local_costmap.yaml')
    global_costmap = _params('tour', 'global_costmap__global_costmap.yaml')

    assert checker['xy_goal_tolerance'] == profile['goal']['xy_tolerance']
    assert checker['yaw_goal_tolerance'] == profile['goal']['yaw_tolerance']
    assert local_costmap['scan_priority_layer']['scan_topic'] == profile['nav2']['scan_topic']
    assert global_costmap['obstacle_layer']['scan']['topic'] == profile['nav2']['scan_topic']


def test_chassis_adapter_matches_david_move(profile):
    """The adapter section equals the david_move node started with navigation."""
    adapter = profile['chassis']['adapter']
    for mode in MODES:
        params = _params(mode, 'david_move.yaml')
        assert params['input_cmd_vel_topic'] == adapter['input_topic']
        assert params['output_cmd_vel_topic'] == profile['chassis']['cmd_topic']
        assert params['max_linear_x'] == adapter['hard_max_linear_x']
        assert params['max_angular_z'] == adapter['hard_max_angular_z']
        assert params['min_linear_x'] == adapter['min_linear_x']
        assert params['min_angular_z'] == adapter['min_angular_z']
        assert params['min_stationary_angular_z'] == adapter['min_stationary_angular_z']
        assert params['stationary_angular_threshold'] == adapter['stationary_angular_threshold']
        assert params['cmd_vel_timeout'] == adapter['cmd_timeout_sec']
        assert params['publish_hz'] == adapter['publish_hz']


def test_teleop_matches_the_keyboard_configuration(profile):
    """Teleop values equal david_keyboard in david_cmd/config/cmd.yaml."""
    keyboard = _load(SOURCE_DIR / 'david_cmd' / 'config' / 'cmd.yaml')['david_keyboard'][
        'ros__parameters']
    teleop = profile['teleop']

    assert keyboard['cmd_vel_topic'] == teleop['cmd_topic']
    assert keyboard['linear_speed'] == teleop['linear_speed']
    assert keyboard['angular_speed'] == teleop['angular_speed']
    assert keyboard['max_linear_x'] == teleop['max_linear_x']
    assert keyboard['max_angular_z'] == teleop['max_angular_z']
    assert keyboard['publish_hz'] == teleop['publish_hz']
    assert keyboard['key_timeout'] == teleop['key_timeout_sec']


def test_limits_stay_within_the_adapter(profile):
    """Navigation and teleop speeds never exceed the adapter hard limits (plan §12.4)."""
    adapter = profile['chassis']['adapter']

    assert profile['limits']['max_vel_x'] <= adapter['hard_max_linear_x']
    assert profile['limits']['max_vel_theta'] <= adapter['hard_max_angular_z']
    assert adapter['min_stationary_angular_z'] <= adapter['hard_max_angular_z']
    assert profile['teleop']['linear_speed'] <= adapter['hard_max_linear_x']
    assert profile['teleop']['angular_speed'] <= adapter['hard_max_angular_z']


def test_scan_from_cloud_matches_the_bringup_launch(profile):
    """Point cloud settings equal the frozen pre-migration launch values."""
    launch = (SOURCE_DIR / 'david_nav' / 'launch' / '2d_david_nav2_bringup.launch.py').read_text(
        encoding='utf-8')
    scan = profile['sensors']['scan_from_cloud']

    golden = _load(
        SOURCE_DIR / 'nav_perception_iface' / 'test' / 'golden' / 'pointcloud_to_scan.yaml')
    for key, value in golden['parameters'].items():
        actual = profile['frames']['scan_target'] if key == 'target_frame' else scan[key]
        assert actual == value
    assert scan['enabled'] == golden['enabled']
    assert golden['remappings'] == [
        ['cloud_in', scan['cloud_topic']], ['scan', scan['scan_topic']]]
    assert "get_package_share_directory('nav_perception_iface')" in launch


def test_driver_parameters_match_the_wheeltec_launch(profile):
    """Driver parameters equal the defaults of the vendored wheeltec launch."""
    launch_path = (
        SOURCE_DIR / 'third_party' / 'wheeltec_original' / 'launch'
        / 'turn_on_wheeltec_robot.launch.py')
    defaults = dict(re.findall(
        r"DeclareLaunchArgument\(\s*'(\w+)',\s*default_value='([^']*)'",
        launch_path.read_text(encoding='utf-8')))
    parameters = profile['chassis']['driver']['parameters']

    for key in ('usart_port_name', 'odom_frame_id', 'robot_frame_id', 'gyro_frame_id'):
        assert defaults[key] == parameters[key]
    assert int(defaults['serial_baud_rate']) == parameters['serial_baud_rate']
    for key in ('odom_x_scale', 'odom_y_scale', 'odom_z_scale_positive', 'odom_z_scale_negative'):
        assert float(defaults[key]) == parameters[key]


def test_urdf_equals_the_production_copy():
    """The URDF is byte-identical to the one the bringup launch publishes today."""
    urdf = PACKAGE_DIR / 'urdf' / 'david.urdf'
    production = SOURCE_DIR / 'david_nav' / 'urdf' / 'david.urdf'

    assert urdf.read_bytes() == production.read_bytes()


def test_profile_passes_the_package_schemas(profile):
    """Keys declared by the package schemas of this workspace hold valid values."""
    catalog = schema.load_catalog(schema.find_source_schemas(SOURCE_DIR))

    issues = catalog.check_profile(profile)
    errors = [issue.message for issue in issues if issue.severity == 'error']

    assert not errors
