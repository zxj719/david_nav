"""precise_nav_bringup.launch.py 的 server 模块接线（alias 0，2026-09-15）。"""
import importlib.util
import os
from pathlib import Path
import subprocess

import pytest

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch_ros.actions import Node
import yaml

LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'precise_nav_bringup.launch.py'


def _load_launch_module():
    spec = importlib.util.spec_from_file_location('precise_nav_bringup_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda package: str(LAUNCH_FILE.parents[2] / package)
    return module


def _context(**overrides):
    values = {
        'start_executor': 'true',
        'server_uri': '',
        'pose_file': '~/semantic_map/named_poses.json',
        'dock_poi_ids': 'POI_017',
        'nav_ready_timeout': '120',
        'start_chassis': 'false', 'chassis_startup_delay': '0',
        'dock_goal_topic': '/dock_goal', 'approach_dist': '0.5', 'xy_ok': '0.10',
        'yaw_ok': '0.4363', 'max_yaw_retries': '0',
    }
    values.update(overrides)
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


def _command_line(process, context):
    return ' '.join(''.join(part.perform(context) for part in item) for item in process.cmd)


def test_launch_declares_the_server_module_arguments():
    description = _load_launch_module().generate_launch_description()

    names = {
        entity.name for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }

    assert {'start_executor', 'server_uri', 'pose_file', 'dock_poi_ids',
            'nav_ready_timeout'} <= names


def test_executor_script_waits_for_nav2_before_starting_the_executor():
    script = _load_launch_module().build_executor_script(
        '/src/david_nav/david_nav/navigation_executor.py',
        '/src/david_nav/scripts/wait_nav_ready.py',
        ['--server-uri', 'ws://192.168.123.186:8101',
         '--pose-file', '/home/agx/semantic_map/named_poses.json',
         '--dock-poi-id', 'POI_017'],
        '120',
    )

    assert 'deadline=$((SECONDS + 120))' in script
    assert script.index('wait_nav_ready.py --timeout 5 --quiet') < script.index(
        'exec python3 /src/david_nav/david_nav/navigation_executor.py')
    assert script.endswith('--dock-poi-id POI_017')
    assert subprocess.run(['bash', '-n', '-c', script], check=False).returncode == 0


def test_executor_uses_the_alias_4_server_and_pose_file(monkeypatch):
    module = _load_launch_module()
    monkeypatch.setattr(module, '_pids_for', lambda patterns: [])
    context = _context()

    actions = module._executor_actions(context)

    processes = [action for action in actions if isinstance(action, ExecuteProcess)]
    assert len(processes) == 1
    command_line = _command_line(processes[0], context)
    headless = yaml.safe_load(
        (LAUNCH_FILE.parents[1] / 'config' / 'navigation_headless.yaml').read_text(
            encoding='utf-8'))
    assert '--server-uri ' + headless['executor']['server_uri'] in command_line
    assert '--pose-file ' + os.path.expanduser('~/semantic_map/named_poses.json') in command_line
    assert '--dock-poi-id POI_017' in command_line


def test_executor_is_not_started_twice_or_when_disabled(monkeypatch):
    module = _load_launch_module()
    monkeypatch.setattr(module, '_pids_for', lambda patterns: [4242])
    already_running = module._executor_actions(_context())
    monkeypatch.setattr(module, '_pids_for', lambda patterns: [])
    disabled = module._executor_actions(_context(start_executor='false'))

    for actions in (already_running, disabled):
        assert actions
        assert all(isinstance(action, LogInfo) for action in actions)


@pytest.mark.parametrize('websocket', ['true', 'false'])
def test_precise_launch_owns_only_one_manager(monkeypatch, websocket):
    """The compatibility launch must not turn its former two executables into two roots."""
    module = _load_launch_module()
    monkeypatch.setattr(module, '_pids_for', lambda patterns: [])
    context = _context(start_executor=websocket)
    processes = [action for action in module._setup(context) if isinstance(action, ExecuteProcess)]
    assert len(processes) == 1
    if websocket == 'true':
        assert not isinstance(processes[0], Node)
        command = _command_line(processes[0], context)
        assert '--dock-config-file ' in command
        assert '--websocket' in command and 'yaw_ok:=0.4363' in command
        assert 'max_yaw_retries:=0' in command and 'goal_topic:=/dock_goal' in command
    else:
        assert isinstance(processes[0], Node)
        assert processes[0].node_package == 'nav_executor'


def test_existing_manager_refuses_the_whole_legacy_launch(monkeypatch):
    """An already owned task runtime is not a reason to start another local Dock owner."""
    module = _load_launch_module()
    monkeypatch.setattr(module, '_pids_for', lambda patterns: (
        [4242] if patterns == module.EXECUTOR_PROC_PATTERNS else []))
    for websocket in ('true', 'false'):
        actions = module._setup(_context(start_executor=websocket))
        assert actions and all(isinstance(action, LogInfo) for action in actions)


def test_existing_manager_patterns_cover_new_and_retained_entries():
    """Compatibility startup recognizes every executable spelling without signaling it."""
    patterns = _load_launch_module().EXECUTOR_PROC_PATTERNS
    assert 'nav_task_manager' in patterns
    assert 'lib/nav_executor/navigation_executor' in patterns
    assert 'lib/david_nav/dock_navigator' in patterns
