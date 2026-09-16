"""Unified task ownership and its frozen configuration files, without launching ROS."""

from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from nav_bringup.launching.bundle import load_options
from nav_bringup.launching.graph import build_graph, GraphOptions
from nav_bringup.navcfg import schema
from nav_bringup.navrun.preparation import prepare_bundle, write_bundle
from test_run_preparation import _inputs, SOURCE


@pytest.mark.parametrize('mode,websocket,count', [
    ('tour', True, 1), ('tour', False, 0), ('precise', True, 1), ('precise', False, 1),
    ('bench_slamgt', False, 0), ('chassis', False, 0), ('teleop', False, 0),
])
def test_each_mode_has_at_most_one_task_manager(tmp_path, mode, websocket, count):
    """ROS Dock and optional WS never create competing task-manager processes."""
    graph = build_graph(GraphOptions(mode, str(tmp_path), start_executor=websocket))
    managers = [spec for spec in graph.processes if spec.package == 'nav_executor']
    assert len(managers) == count
    assert not any(spec.package == 'david_nav' and spec.executable in (
        'dock_navigator', 'navigation_executor.py') for spec in graph.processes)
    if not count:
        return
    manager = managers[0]
    assert manager.executable == 'nav_task_manager'
    assert manager.wait_for_navigation == websocket
    assert manager.name == ('navigation_executor' if websocket else 'dock_navigator')
    assert ('--websocket' if websocket else '--no-websocket') in manager.arguments
    assert '--robot-profile' in manager.arguments
    assert ('--dock-config-file' in manager.arguments) == (mode == 'precise')
    assert len(manager.configuration_files) == (2 if mode == 'precise' else 1)


@pytest.mark.parametrize('websocket', [False, True])
def test_manager_snapshot_contains_effective_mode_not_raw_robot(tmp_path, websocket):
    """Mode overrides reach the runtime, with all referenced files present before launch."""
    inputs = _inputs(tmp_path, 'precise')
    mode = yaml.safe_load(inputs.mode.read_text())
    mode['profile'].setdefault('executor', {})['nav_retry_count'] = 3
    inputs.mode.write_text(yaml.safe_dump(mode))
    before = inputs.profile.read_bytes()
    options = GraphOptions('precise', str(tmp_path / 'run'), start_executor=websocket)
    prepared = prepare_bundle(options, inputs, schema.load_catalog(
        schema.find_source_schemas(SOURCE)))
    manager = next(spec for spec in build_graph(prepared.options).processes
                   if spec.package == 'nav_executor')
    artifacts = dict(prepared.artifacts)
    profile_path = Path(manager.arguments[manager.arguments.index('--robot-profile') + 1])
    effective = yaml.safe_load(artifacts[str(profile_path.relative_to(options.run_dir))])
    assert effective['executor']['nav_retry_count'] == 3
    assert inputs.profile.read_bytes() == before
    assert not Path(options.run_dir).exists()
    Path(options.run_dir).mkdir()
    write_bundle(prepared)
    assert all(Path(path).is_file() for path in manager.configuration_files)
    assert load_options(options.run_dir, {}) == prepared.options


@pytest.mark.parametrize('missing', ['executor_profile.yaml', 'dock.yaml'])
def test_missing_task_configuration_rejected_before_launch(tmp_path, missing):
    """Switching executable cannot bypass eager checking of plain CLI configuration files."""
    inputs = _inputs(tmp_path, 'precise')
    options = GraphOptions('precise', str(tmp_path / 'run'), start_executor=True)
    prepared = prepare_bundle(options, inputs, schema.load_catalog(
        schema.find_source_schemas(SOURCE)))
    Path(options.run_dir).mkdir()
    write_bundle(prepared)
    path = Path(options.run_dir) / 'params/rendered' / missing
    assert path.exists()
    path.unlink()
    with pytest.raises(ValueError, match='missing rendered|missing task'):
        load_options(options.run_dir, {})


def test_manager_keeps_explicit_arguments_and_backend_readiness_target(tmp_path):
    """Custom server/site argv survives, and ROS-only mode uses the selected Nav2 backend."""
    options = GraphOptions('precise', str(tmp_path), start_executor=True,
                           ready_action='/custom/navigation',
                           executor_arguments=('--pose-file', '/a path/poses.json'))
    manager = next(spec for spec in build_graph(options).processes
                   if spec.package == 'nav_executor')
    args = manager.arguments
    assert args[args.index('--pose-file') + 1] == '/a path/poses.json'
    assert args[args.index('--action-name') + 1] == '/custom/navigation'
    local = next(spec for spec in build_graph(replace(options, start_executor=False)).processes
                 if spec.package == 'nav_executor')
    assert '--no-websocket' in local.arguments and not local.wait_for_navigation
