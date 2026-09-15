"""Opt-in lightweight modes preserve default graphs and freeze explicit experiments."""

from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from nav_bringup.launching.bundle import load_options
from nav_bringup.launching.graph import build_graph, GraphOptions
from nav_bringup.navcfg.live_inputs import load_live_spec
from nav_bringup.navcfg import render, schema
from nav_bringup.navrun.cli import build_parser
from nav_bringup.navrun.preflight import ProcessInfo
from nav_bringup.navrun.preparation import write_bundle
from nav_bringup.navrun.up_planning import discover_inputs, plan_up, UpRequest


SOURCE = Path(__file__).resolve().parents[2]
DAVID = SOURCE / 'david_config'


def _plan(tmp_path, mode, experiment=None):
    path = DAVID / 'modes/experiments' / f'{mode}_{experiment}.yaml' if experiment else None
    request = UpRequest(mode, 'david', 'offline', tmp_path / 'run', no_executor=True,
                        mode_file=path)
    inputs = discover_inputs(request, lambda package: SOURCE / package)
    inputs = replace(inputs, site=SOURCE / 'example_diffdrive_config/sites/offline/site.yaml')
    own = ProcessInfo(100, 1, 1.0, ('navrun',), 1000)
    return plan_up(request, inputs, schema.load_catalog(schema.find_source_schemas(SOURCE)),
                   (own,), own.pid, lambda package, name: '/unused/' + name)


@pytest.mark.parametrize('mode', ['tour', 'precise'])
@pytest.mark.parametrize('experiment', ['l2', 'l3', 'l2_l3'])
def test_experiments_change_only_declared_parameters_and_graph(tmp_path, mode, experiment):
    """Opt-in deltas retain docking tolerances, exact hard limits and frozen source bytes."""
    original = _plan(tmp_path / 'original', mode)
    selected = _plan(tmp_path / 'selected', mode, experiment)
    baseline = dict(original.bundle.artifacts)
    actual = dict(selected.bundle.artifacts)
    l2, l3 = 'l2' in experiment, 'l3' in experiment
    assert selected.errors == ()
    assert selected.bundle.options.velocity_smoother is not l2
    names = [process.name for process in selected.graph.processes]
    assert ('velocity_smoother' in names) is not l2
    assert ('velocity_smoother' in selected.graph.lifecycle_nodes) is not l2
    original_adapter = yaml.safe_load(baseline['params/rendered/adapter.yaml'])
    adapter = yaml.safe_load(actual['params/rendered/adapter.yaml'])
    parameters = original_adapter['chassis_adapter']['ros__parameters']
    parameters.update(cmd_vel_timeout=1.0 if l2 else 2.0, publish_hz=20.0 if l3 else 50.0)
    assert adapter == original_adapter
    assert parameters['max_angular_z'] == 0.6
    assert parameters['min_stationary_angular_z'] == 0.55
    source_mode = DAVID / 'modes/experiments' / f'{mode}_{experiment}.yaml'
    assert actual['params/inputs/mode.yaml'] == source_mode.read_bytes()
    base_mode = render.parse_mode(render.load_yaml(DAVID / 'modes' / f'{mode}.yaml'), mode)
    new_mode = render.parse_mode(render.load_yaml(source_mode), source_mode)
    base = render.load_yaml(SOURCE / 'nav_bringup/config/nav2_base.yaml')
    bindings = render.parse_bindings(
        render.load_yaml(SOURCE / 'nav_bringup/config/nav2_bindings.yaml'), 'bindings')
    profile = render.load_yaml(DAVID / 'robot/david.yaml')
    expected = render.render(base, bindings, profile, base_mode)
    if l2:
        expected['behavior_server']['ros__parameters']['max_rotational_vel'] = 0.5
    assert render.render(base, bindings, profile, new_mode) == expected
    for process in selected.graph.processes:
        if process.name in ('controller_server', 'behavior_server'):
            assert ('cmd_vel', selected.bundle.options.smooth_command_topic if l2
                    else selected.bundle.options.nav_command_topic) in process.remappings
    root = Path(selected.bundle.options.run_dir)
    root.mkdir(parents=True)
    write_bundle(selected.bundle)
    assert load_options(str(root), {}) == selected.bundle.options
    live, _ = load_live_spec(root)
    assert ('/velocity_smoother' in live.parameters) is not l2
    topics = {rule.topic for rule in live.topics}
    assert ('/cmd_vel_nav' in topics) is not l2
    assert '/cmd_vel_smoothed' in topics


def test_default_graph_and_cli_leave_experiments_disabled(tmp_path):
    """An unmodified invocation retains smoother, 50 Hz and the two-second timeout."""
    plan = _plan(tmp_path, 'tour')
    assert plan.bundle.options.velocity_smoother
    args = build_parser().parse_args(['up', '--mode', 'tour'])
    assert args.mode_file is None
    path = DAVID / 'modes/experiments/tour_l2_l3.yaml'
    args = build_parser().parse_args(['up', '--mode', 'tour', '--mode-file', str(path)])
    assert args.mode_file == path


def test_wrong_mode_file_refuses_without_creating_run(tmp_path):
    """An experiment cannot silently switch the declared runtime mode."""
    request = UpRequest('tour', 'david', 'offline', tmp_path / 'run', no_executor=True,
                        mode_file=DAVID / 'modes/experiments/precise_l2.yaml')
    inputs = discover_inputs(request, lambda package: SOURCE / package)
    with pytest.raises(ValueError, match='mode'):
        plan_up(request, inputs, schema.load_catalog(schema.find_source_schemas(SOURCE)),
                (ProcessInfo(100, 1, 1.0, ('navrun',), 1000),), 100,
                lambda *args: pytest.fail('must reject before executable lookup'))
    assert not request.run_dir.exists()


@pytest.mark.parametrize('value', ['false', 0, None])
def test_smoother_switch_must_be_boolean(value):
    """Malformed graph controls cannot silently bypass velocity processing."""
    with pytest.raises(ValueError, match='velocity_smoother'):
        build_graph(GraphOptions('tour', '/unused/run', velocity_smoother=value))
