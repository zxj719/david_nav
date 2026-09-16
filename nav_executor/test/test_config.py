"""Configuration precedence, legacy CLI goldens and robot-independent task binding."""

from pathlib import Path
import runpy

import pytest
import yaml

from nav_tasks.api import ConfigError

from nav_executor.app.cli import build_help_text
from nav_executor.app.config import load_configuration, load_headless_executor_section

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'david_config' / 'robot' / 'david.yaml'
LEGACY = ROOT / 'david_nav/test/golden/legacy_entries/navigation_executor.py.txt'


def test_default_configuration_and_help_match_old_executor(monkeypatch):
    """New dataclass defaults and frozen help equal the pinned pre-shim implementation."""
    monkeypatch.delenv('NAV_WS_URL', raising=False)
    legacy = runpy.run_path(str(LEGACY))
    expected, remaining = legacy['_parse_cli_args']([])
    resolved = load_configuration([], profile_path=PROFILE, environ={})
    assert resolved.config.legacy_values() == expected
    assert list(resolved.ros_args) == remaining
    assert build_help_text() == legacy['_build_help_text']()


@pytest.mark.parametrize('arguments', [
    ['--server-uri', 'ws://other', '--action-name', '/other'],
    ['--poi-store-file', '~/example.yaml', '--pose-file', '~/poses.json'],
    ['--nav-retry-count', '3', '--nav-retry-delay', '0.2', '--reconnect-delay', '4'],
    ['--dock-poi-id', ' p ', '--dock-poi-id', 'p', '--dock-poi-id', '',
     '--dock-poi-id', 'q', '--dock-action-name', '/dock'],
    ['--ros-args', '-r', '__node:=example', '--server-uri', 'ws://changed', '-p', 'x:=2'],
    ['--server-uri', 'ws://first', '--unknown', 'value', '--server-uri', 'ws://last'],
])
def test_supported_old_arguments_remain_equivalent(monkeypatch, arguments):
    """Existing overrides, repetitions, paths and interspersed ROS flags preserve values."""
    monkeypatch.delenv('NAV_WS_URL', raising=False)
    legacy = runpy.run_path(str(LEGACY))
    old, ros = legacy['_parse_cli_args'](arguments)
    new = load_configuration(arguments, profile_path=PROFILE, environ={})
    assert new.config.legacy_values() == old
    assert list(new.ros_args) == ros


def test_configuration_precedence_and_provenance_are_explicit(tmp_path):
    """CLI wins over the one allowed environment variable, then YAML and defaults."""
    path = tmp_path / 'executor.yaml'
    path.write_text('server_uri: ws://yaml\nnav_retry_count: 2\nrobot_base_frame: chassis\n')
    environment = {'NAV_WS_URL': 'ws://environment', 'NAV_RETRY_COUNT': '99'}
    args = ['--config', str(path), '--nav-retry-count', '4', '--nav', 'untouched']
    result = load_configuration(args, profile_path=PROFILE, environ=environment)
    assert result.config.server_uri == 'ws://environment'
    assert result.config.nav_retry_count == 4
    assert result.config.robot_base_frame == 'chassis'
    assert result.sources['server_uri'] == 'env:NAV_WS_URL'
    assert result.sources['nav_retry_count'] == 'cli'
    assert result.sources['robot_base_frame'] == f'yaml:{path}'
    assert result.sources['heartbeat_interval'] == 'default'
    assert result.ros_args == ('--nav', 'untouched')
    assert result.config.ws_options().server_uri == 'ws://environment'
    override = load_configuration(args + ['--server-uri', 'ws://cli'],
                                  profile_path=PROFILE, environ=environment)
    assert override.config.server_uri == 'ws://cli'
    assert override.sources['server_uri'] == 'cli'
    assert 'NAV_RETRY_COUNT' not in result.log_line()
    with pytest.raises(TypeError):
        result.sources['server_uri'] = 'changed'


def test_new_robot_configuration_binds_task_values_without_code_changes(tmp_path):
    """Alternative robot data and CLI configure all frames/topics/thresholds explicitly."""
    profile = tmp_path / 'robot.yaml'
    profile.write_text(yaml.safe_dump({
        'frames': {'map': 'world', 'odom': 'local', 'poi_capture': 'chassis'},
        'nav2': {'odom_topic': '/other_odom'},
        'executor': {'grid_map_topic': '/other_grid', 'poi_store_file': '~/other_store.yaml'},
    }))
    result = load_configuration([
        '--robot-profile', str(profile), '--global-costmap-topic', '/other_cost',
        '--loop-closure-translation-m', '0.08', '--loop-closure-yaw-rad', '0.06',
        '--progress-min-interval-sec', '0.2', '--pose-stale-after-sec', '0.3',
        '--server-timeout', '4', '--cancel-timeout-sec', '2',
    ], profile_path=PROFILE, environ={})
    tasks = result.config.task_config()
    assert (tasks.map_frame, tasks.odom_frame, tasks.robot_base_frame) == (
        'world', 'local', 'chassis')
    assert (tasks.odom_topic, tasks.grid_map_topic, tasks.global_costmap_topic) == (
        '/other_odom', '/other_grid', '/other_cost')
    assert tasks.poi_store_file == Path('~/other_store.yaml').expanduser()
    assert tasks.server_timeout_sec == 4 and tasks.cancel_timeout_sec == 2
    assert tasks.loop_closure_translation_m == 0.08 and tasks.loop_closure_yaw_rad == 0.06
    assert tasks.progress_min_interval_sec == 0.2 and tasks.pose_stale_after_sec == 0.3


def test_headless_executor_section_preserves_format_and_ignores_launcher_only_keys(tmp_path):
    """Reading the shared old file neither changes it nor leaks launcher options into tasks."""
    path = tmp_path / 'headless.yaml'
    source = ('other: untouched\nexecutor:\n  server_uri: ws://headless\n'
              '  server_timeout: 20\n  show_logs: true\n  nav_event_timeout: 15\n'
              '  record_bag: true\n')
    path.write_text(source)
    assert load_headless_executor_section(path) == {
        'server_uri': 'ws://headless', 'server_timeout': 20}
    result = load_configuration(['--config', str(path)], profile_path=PROFILE, environ={})
    assert result.config.server_uri == 'ws://headless'
    assert result.config.server_timeout == 20
    assert path.read_text() == source


@pytest.mark.parametrize('content', [
    '[]', 'executor: []', 'unknown: 1', 'nav_retry_count: true', 'nav_retry_count: 1.2',
    'nav_retry_delay_sec: .nan', 'heartbeat_interval: 0', 'nav_goal_snap_step: 0',
    'odom_topic: null', 'dock_poi_ids: name', 'dock_poi_ids: [1]', 'server_uri: ""',
])
def test_invalid_yaml_values_fail_before_runtime_construction(tmp_path, content):
    """Bad configuration is rejected without constructing clients or touching POI stores."""
    path = tmp_path / 'config.yaml'
    path.write_text(content)
    with pytest.raises(ConfigError):
        load_configuration(['--config', str(path)], profile_path=PROFILE, environ={})


@pytest.mark.parametrize('args', [['--server-uri'], ['--nav-retry-count', 'bad'],
                                  ['--nav-retry-count', '-1']])
def test_known_invalid_cli_arguments_fail_explicitly(args):
    """Argparse reports incomplete/nonnumeric options instead of forwarding broken flags."""
    with pytest.raises(ConfigError):
        load_configuration(args, profile_path=PROFILE, environ={})
