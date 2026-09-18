"""Explicit old Dock YAML files bind through the public API without enabling motion."""

import asyncio
from pathlib import Path

import pytest
import yaml

from nav_bringup.launching.graph import GraphOptions
from nav_bringup.navcfg import render
from nav_bringup.navrun.prepared_parameters import node_parameters
from nav_tasks.api import ConfigError

from nav_executor.app.config import load_configuration
from nav_executor.app.runtime import run

PROFILE = Path(__file__).resolve().parents[2] / 'david_config/robot/david.yaml'


def _parameters():
    return dict(
        approach_dist=0.5, xy_ok=0.1, yaw_ok=0.4363, max_yaw_retries=0,
        goal_topic='/dock_goal', action_name='dock_to_pose',
        robot_base_frame='navigation_center', global_frame='world',
        nav_timeout_sec=120.0, spin_timeout_sec=30.0, static_map_topic='/static_grid',
        static_occupied_threshold=50, footprint='[[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]]',
        settle_max_rate_rad_s=0.05, settle_window_sec=0.4, settle_timeout_sec=8.0)


def _file(tmp_path, wrapped=False):
    path = tmp_path / 'dock.yaml'
    data = _parameters()
    if wrapped:
        data = {'dock_navigator': {'ros__parameters': data}}
    path.write_text(yaml.safe_dump(data))
    return path


@pytest.mark.parametrize('wrapped', [False, True])
def test_dock_file_binds_complete_settings_and_leaves_ros_arguments_untouched(tmp_path, wrapped):
    """Flat and existing navrun files produce identical explicit runtime/ingress values."""
    path = _file(tmp_path, wrapped)
    before = path.read_bytes()
    result = load_configuration([
        '--dock-config-file', str(path), '--dock-poi-id', 'p',
        '--ros-args', '-p', 'yaw_ok:=0.3'], profile_path=PROFILE, environ={})
    assert result.ros_args == ('--ros-args', '-p', 'yaw_ok:=0.3')
    assert result.dock_parameters == _parameters()
    assert result.dock.goal_topic == '/dock_goal'
    assert result.dock.action_name == 'dock_to_pose'
    assert result.config.dock_action_name == 'dock_to_pose'
    assert result.sources['dock_action_name'] == f'dock-file:{path}:action_name'
    assert result.sources['dock_config_file'] == 'cli'
    tasks = result.task_config()
    assert tasks.dock == result.dock.runtime
    assert tasks.dock_poi_ids == ('p',)
    assert tasks.dock.robot_base_frame == 'navigation_center'
    assert tasks.robot_base_frame != tasks.dock.robot_base_frame
    assert tasks.dock.static_map_topic == '/static_grid'
    assert tasks.dock.settle_max_rate_rad_s == 0.05
    assert tasks.dock.max_yaw_retries == 0 and tasks.dock.yaw_ok == 0.4363
    assert 'navigation_center' in result.log_line()
    assert path.read_bytes() == before
    with pytest.raises(TypeError):
        result.dock_parameters['yaw_ok'] = 0.2


@pytest.mark.parametrize('from_cli', [False, True])
def test_existing_dock_endpoint_override_wins_over_file(tmp_path, from_cli):
    """The old dock action flag/config override selects the same ingress endpoint."""
    path = _file(tmp_path)
    config = tmp_path / 'executor.yaml'
    config.write_text(yaml.safe_dump(dict(dock_config_file=str(path),
                                          dock_action_name='/yaml_dock')))
    args = ['--config', str(config)]
    if from_cli:
        args += ['--dock-action-name', '/cli_dock']
    result = load_configuration(args, profile_path=PROFILE, environ={})
    expected = '/cli_dock' if from_cli else '/yaml_dock'
    assert result.config.dock_action_name == result.dock.action_name == expected
    assert result.dock_parameters['action_name'] == expected
    assert result.sources['dock_action_name'] == ('cli' if from_cli else f'yaml:{config}')


def test_absent_dock_file_keeps_ordinary_configuration_unchanged():
    """Ordinary CLI/defaults create no Dock policy, resources or required robot keys."""
    result = load_configuration([], profile_path=PROFILE, environ={})
    assert result.dock is None and result.dock_parameters is None
    assert result.task_config() == result.config.task_config()
    assert result.task_config().dock is None
    assert 'dock_config_file' not in result.config.legacy_values()


@pytest.mark.parametrize('package,robot', [
    ('david_config', 'david'), ('example_diffdrive_config', 'example_diffdrive'),
])
def test_existing_robot_and_precise_mode_render_bind_without_policy_drift(
    tmp_path, package, robot,
):
    """Read the real profile/mode and old launch renderer; no David policy is guessed in code."""
    root = PROFILE.parents[2]
    profile_path = root / package / 'robot' / f'{robot}.yaml'
    original = render.load_yaml(profile_path)
    mode = render.load_yaml(root / package / 'modes/precise.yaml')
    profile = render.deep_merge(original, mode['profile'])
    rendered = node_parameters(GraphOptions('precise', str(tmp_path / 'run')), profile,
                               {'nav2': {}, 'robot_description': '<robot/>'})['dock']
    path = tmp_path / 'dock.yaml'
    path.write_text(yaml.safe_dump(rendered))
    # Only the Dock file is under test; ordinary executor settings retain the
    # selected standard profile and its separate POI capture contract.
    resolved = load_configuration(['--dock-config-file', str(path)],
                                  profile_path=PROFILE, environ={})
    dock = resolved.task_config().dock
    assert dock.robot_base_frame == profile['frames']['nav_base']
    assert dock.global_frame == profile['frames']['map']
    assert dock.static_map_topic == profile['nav2']['map_topic']
    assert dock.footprint == tuple(tuple(point) for point in profile['footprint'])
    assert dock.xy_ok == profile['dock']['xy_ok']
    assert dock.yaw_ok == profile['dock']['yaw_ok']
    assert dock.max_yaw_retries == 0 and dock.settle_max_rate_rad_s == 0.05
    assert resolved.dock_parameters == rendered['dock_navigator']['ros__parameters']


@pytest.mark.parametrize('content', [
    '[]', 'dock_navigator: []', 'dock_navigator: {ros__parameters: []}',
    'dock_navigator: {ros__parameters: {}, extra: 1}',
    'dock_navigator: {ros__parameters: {}}\nother_node: {}',
    'yaw_ok: 0.4',
])
def test_malformed_or_incomplete_dock_file_fails_at_configuration_time(tmp_path, content):
    """Incomplete ROS/YAML trees cannot silently fall back to ordinary POI routing."""
    path = tmp_path / 'dock.yaml'
    path.write_text(content)
    with pytest.raises(ConfigError):
        load_configuration(['--dock-config-file', str(path)], profile_path=PROFILE, environ={})


def test_missing_file_fails_at_configuration_time(tmp_path):
    """A requested configuration must exist even when it defines no Dock POI IDs."""
    with pytest.raises(ConfigError):
        load_configuration(['--dock-config-file', str(tmp_path / 'missing.yaml')],
                           profile_path=PROFILE, environ={})


def test_preexisting_stop_refuses_dock_before_ros_or_storage_io(tmp_path):
    """A stop already requested must not construct or expose the Dock host."""
    result = load_configuration(['--dock-config-file', str(_file(tmp_path))],
                                profile_path=PROFILE, environ={})

    def forbidden():
        pytest.fail('ROS host opened despite preexisting stop')

    async def scenario():
        stop = asyncio.Event()
        stop.set()
        await run(result, stop, host_factory=forbidden)

    asyncio.run(scenario())
