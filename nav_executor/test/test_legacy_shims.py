"""Exercise retained executable paths through fake roots, never ROS or hardware."""

import ast
import importlib
import os
from pathlib import Path
import runpy
import sys
from types import SimpleNamespace

import pytest
import yaml

from nav_executor.app import cli
from nav_executor.app.config import load_configuration
from nav_tasks.api import declare_dock_parameters


ROOT = Path(__file__).resolve().parents[2]
DEFAULTS = ROOT / 'david_config/dock/standalone.yaml'


def test_no_argument_standalone_declares_dedicated_dock_topic(monkeypatch):
    """R8 resolves no-argument Dock CLI through configuration and real parameter binding."""
    module = importlib.import_module('nav_executor.app.dock_cli')
    monkeypatch.setattr(module, '_default_policy', lambda: DEFAULTS)
    declarations, configured = {}, []

    class Node:
        def declare_parameter(self, name, value):
            declarations[name] = value

        def get_parameter(self, name):
            return SimpleNamespace(value=declarations[name])

    def entry(args):
        resolved = load_configuration(args, profile_path=ROOT / 'david_config/robot/david.yaml',
                                      environ={})
        configured.append(declare_dock_parameters(Node(), resolved.dock_parameters))
        assert not resolved.config.websocket_enabled
        return 0

    monkeypatch.setattr(cli, 'main', entry)
    assert module.main([]) == 0
    assert configured[0].goal_topic == '/dock_goal'
    assert declarations['goal_topic'] == '/dock_goal'
    assert len(declarations) == 16
    assert configured[0].runtime.yaw_ok == 0.087
    assert configured[0].runtime.max_yaw_retries == 2
    assert configured[0].runtime.settle_max_rate_rad_s == 0.05


@pytest.mark.parametrize('name,target', [
    ('navigation_executor', 'nav_executor.app.cli'),
    ('dock_navigator', 'nav_executor.app.dock_cli'),
])
def test_retained_executable_delegates_argv_and_exit_status(monkeypatch, name, target):
    """The historical source path is executable and invokes exactly the new root."""
    module = importlib.import_module(target)
    calls = []

    def entry(args=None):
        calls.append(sys.argv[1:] if args is None else args)
        return 37

    monkeypatch.setattr(module, 'main', entry)
    path = ROOT / 'david_nav/david_nav' / (name + '.py')
    assert path.read_text().startswith('#!/usr/bin/env python3\n')
    assert os.access(path, os.X_OK)
    assert len(path.read_text().splitlines()) < 25
    args = ['--ros-args', '-p', 'goal_topic:=/test goal']
    monkeypatch.setattr(sys, 'argv', [str(path), *args])
    with pytest.raises(SystemExit) as result:
        runpy.run_path(str(path), run_name='__main__')
    assert result.value.code == 37
    assert calls == [args]
    loaded = runpy.run_path(str(path))
    assert loaded['main'] is entry and len(calls) == 1


def test_standalone_dock_data_preserves_frozen_defaults_except_r8_topic():
    """Only the documented R8 topic differs from the frozen standalone policy."""
    source = ROOT / 'david_nav/test/golden/legacy_entries/dock_navigator.py.txt'
    tree = ast.parse(source.read_text())
    footprint = next(ast.literal_eval(node.value) for node in tree.body
                     if isinstance(node, ast.Assign)
                     and any(isinstance(target, ast.Name) and target.id == 'DEFAULT_FOOTPRINT'
                             for target in node.targets))
    expected = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter'):
            value = node.args[1]
            expected[ast.literal_eval(node.args[0])] = (
                footprint if isinstance(value, ast.Name) else ast.literal_eval(value))
    assert len(expected) == 16
    assert expected['goal_topic'] == '/goal_pose'
    expected['goal_topic'] = '/dock_goal'
    assert yaml.safe_load(DEFAULTS.read_text()) == expected


def test_dock_cli_loads_explicit_policy_and_forces_local_only(monkeypatch):
    """Old Dock entry defaults to data, forwards ROS overrides, and cannot open WS."""
    module = importlib.import_module('nav_executor.app.dock_cli')
    monkeypatch.setattr(module, '_default_policy', lambda: DEFAULTS)
    calls = []
    monkeypatch.setattr(cli, 'main', lambda args: calls.append(args) or 11)
    args = ['--ros-args', '-p', 'yaw_ok:=0.2', '--websocket']
    assert module.main(args) == 11
    assert calls == [['--dock-config-file', str(DEFAULTS), *args, '--no-websocket']]
    resolved = load_configuration(calls[0], profile_path=ROOT / 'david_config/robot/david.yaml',
                                  environ={})
    assert not resolved.config.websocket_enabled
    assert resolved.ros_args == ('--ros-args', '-p', 'yaw_ok:=0.2')
    assert resolved.dock.runtime.yaw_ok == 0.087
    assert resolved.dock.runtime.max_yaw_retries == 2


@pytest.mark.parametrize('args', [['--help'], ['--dock-config-file', '/custom/policy.yaml']])
def test_dock_cli_help_and_explicit_policy_do_not_resolve_default(monkeypatch, args):
    """Help needs no robot resources; a custom policy never inherits David Dock tuning."""
    module = importlib.import_module('nav_executor.app.dock_cli')
    monkeypatch.setattr(module, '_default_policy', lambda: pytest.fail('unexpected lookup'))
    calls = []
    monkeypatch.setattr(cli, 'main', lambda value: calls.append(value) or 0)
    assert module.main(args) == 0
    assert calls == [args if '--help' in args else [*args, '--no-websocket']]


def test_sandbox_command_builders_use_one_owner_and_disposable_storage(tmp_path):
    """Inspect unexecuted acceptance recipes; never start their ROS fixtures or send goals."""
    path = ROOT / 'david_nav/sandbox_tests/sandbox_dock_checks.py'
    module = runpy.run_path(str(path))
    command = module['_executor_command'](tmp_path, tmp_path / 'poses.json')
    resolved = load_configuration(command[2:], profile_path=ROOT / 'david_config/robot/david.yaml',
                                  environ={})
    assert resolved.dock is not None and resolved.config.dock_poi_ids == ('POI_017',)
    assert resolved.config.poi_store_file == str(tmp_path / 'poi_store.yaml')
    assert 'yaw_ok:=0.4363' in resolved.ros_args and 'max_yaw_retries:=0' in resolved.ros_args
    local = module['_dock_navigator'](tmp_path)
    assert local[local.index('--poi-store-file') + 1] == str(tmp_path / 'poi_store.yaml')
    scenario = next(node for node in ast.parse(path.read_text()).body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == 'test_executor_docks_the_dock_poi_and_reports_arrival')
    starts = [node for node in ast.walk(scenario)
              if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
              and node.func.attr == 'start']
    assert [ast.literal_eval(node.args[0]) for node in starts] == ['fake_nav2']
