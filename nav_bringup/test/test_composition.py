"""Composition changes process ownership, not logical parameters or activation order."""

from dataclasses import replace
import json

import pytest

from nav_bringup.launching.bundle import load_options
from nav_bringup.launching.graph import build_graph, GraphOptions
from test_launch_bundle import _prepare


@pytest.mark.parametrize('mode', ['tour', 'precise', 'bench_slamgt'])
@pytest.mark.parametrize('smoother', [True, False])
@pytest.mark.parametrize('adapter', ['python', 'cpp'])
def test_composition_preserves_logical_graph_and_groups_only_eligible_nodes(
        mode, smoother, adapter):
    """Only explicitly eligible components share a no-respawn critical container."""
    from nav_bringup.launching.composition import component_specs, physical_graph

    options = GraphOptions(mode, '/unused/run', velocity_smoother=smoother,
                           adapter_implementation=adapter)
    baseline = build_graph(options)
    logical = build_graph(replace(options, composition=True))
    assert logical.processes == baseline.processes
    assert logical.lifecycle_nodes == baseline.lifecycle_nodes
    assert physical_graph(baseline) == baseline
    members = component_specs(logical)
    assert len(members) == 6 + int(smoother) + int(adapter == 'cpp')
    assert all(spec.name != 'dock_navigator' for spec, _ in members)
    physical = physical_graph(logical)
    container = next(spec for spec in physical.processes if spec.name == 'nav2_container')
    assert container.executable == 'component_container_isolated'
    assert container.critical and container.restart_limit == 0
    assert len(physical.processes) == len(logical.processes) - len(members) + 1
    selected = {spec.name for spec, _ in members}
    assert [spec for spec in physical.processes if spec != container] == [
        spec for spec in logical.processes if spec.name not in selected]


@pytest.mark.parametrize('mode', ['chassis', 'teleop'])
def test_composition_rejects_non_navigation_modes(mode):
    """An inapplicable opt-in cannot be silently ignored."""
    with pytest.raises(ValueError, match='composition'):
        build_graph(GraphOptions(mode, '/unused/run', composition=True))


def test_profile_selected_driver_keeps_its_independent_restart_contract():
    """Eligibility is a logical node identity, not just a package/executable match."""
    from nav_bringup.launching.composition import physical_graph

    graph = build_graph(GraphOptions(
        'tour', '/unused/run', composition=True, driver_package='chassis_adapter',
        driver_executable='chassis_adapter_cpp'))
    physical = physical_graph(graph)
    assert physical.processes[0] == graph.processes[0]
    assert physical.processes[0].restart_limit == 1


def test_composition_respects_optional_map_and_independent_scan_bridge():
    """An external map is not loaded, while perception remains its own process."""
    from nav_bringup.launching.composition import component_specs, physical_graph

    graph = build_graph(GraphOptions(
        'tour', '/unused/run', composition=True, map_server=False, scan_bridge=True))
    assert 'map_server' not in graph.lifecycle_nodes
    assert len(component_specs(graph)) == 6
    assert any(spec.name == 'pointcloud_to_laserscan' for spec in physical_graph(graph).processes)


def test_frozen_composition_default_and_explicit_override(tmp_path):
    """Absent selectors preserve the bundle; old bundles still mean false."""
    data = _prepare(tmp_path)
    assert not load_options(str(tmp_path), {}).composition
    assert load_options(str(tmp_path), {'composition': 'true'}).composition
    data['options']['composition'] = True
    (tmp_path / 'launch.json').write_text(json.dumps(data))
    assert load_options(str(tmp_path), {}).composition
    assert not load_options(str(tmp_path), {'composition': 'false'}).composition
    del data['options']['composition']
    (tmp_path / 'launch.json').write_text(json.dumps(data))
    assert not load_options(str(tmp_path), {}).composition


@pytest.mark.parametrize('available', [True, False])
def test_preflight_resolves_physical_executables_and_every_component(tmp_path, available):
    """Missing registrations aggregate before writes; unused standalones are not required."""
    from nav_bringup.navcfg import schema
    from nav_bringup.navrun import up_planning
    from test_up_planning import _request, SELF, SOURCE

    request = _request(tmp_path, composition=True)
    inputs = up_planning.discover_inputs(request, lambda package: SOURCE / package)
    catalog = schema.load_catalog(schema.find_source_schemas(SOURCE))
    checked = []

    def plugin(package, name):
        checked.append((package, name))
        if not available:
            raise LookupError('missing component')
        return '/unused/lib.so'

    plan = up_planning.plan_up(
        request, inputs, catalog, (SELF,), SELF.pid,
        lambda package, name: '/unused/' + name, plugin)
    report = up_planning.report_plan(plan)
    assert report['composition']
    assert len(checked) == len(report['components']) == 7
    assert len(plan.errors) == (0 if available else 7)
    assert 'controller_server' not in report['executables']
    assert 'nav2_container' in report['executables']
    assert not request.run_dir.exists()


def test_missing_component_library_fails_before_constructing_any_process(monkeypatch):
    """The runtime repeats read-only validation before starting a driver."""
    from nav_bringup.launching import runtime

    def missing(*_):
        raise ValueError('missing library')

    monkeypatch.setattr(runtime, 'resolve_component', missing)
    monkeypatch.setattr(runtime, 'Node', lambda **_: pytest.fail('must validate first'))
    with pytest.raises(ValueError, match='missing library'):
        runtime.launch_actions(build_graph(GraphOptions('tour', '/unused/run', composition=True)))


def test_composed_executor_phase_never_creates_a_second_container(monkeypatch):
    """The deferred executor phase preserves its gate and excludes component resolution."""
    from nav_bringup.launching import runtime

    monkeypatch.setattr(runtime, 'resolve_component', lambda *_: pytest.fail('no components'))
    monkeypatch.setattr(runtime, '_resolve', lambda package, name: '/unused/' + name)
    selected = []
    monkeypatch.setattr(runtime, 'managed_actions',
                        lambda graph, factory: selected.extend(graph.processes) or [])
    graph = build_graph(GraphOptions('tour', '/unused/run', composition=True, start_executor=True))
    runtime.launch_actions(graph, 'executor')
    assert len(selected) == 1 and selected[0].name == 'navigation_executor'
    assert selected[0].wait_for_navigation


def test_runtime_preserves_component_remaps_and_container_child_parameter_files(monkeypatch):
    """Nested costmap YAML and node-local overrides both reach the composed runtime."""
    from launch.actions import ExecuteProcess
    from nav_bringup.launching import runtime

    monkeypatch.setattr(runtime, 'resolve_component', lambda *_: '/unused/component.so')
    monkeypatch.setattr(runtime, '_resolve', lambda package, name: '/unused/' + name)
    nodes, components, loaders = [], [], []

    def node(**kwargs):
        nodes.append(kwargs)
        return ExecuteProcess(cmd=['not-executed'])

    def component(**kwargs):
        components.append(kwargs)
        return kwargs

    monkeypatch.setattr(runtime, 'Node', node)
    monkeypatch.setattr(runtime, 'ComposableNode', component)
    monkeypatch.setattr(runtime, 'CheckedComponentLoader',
                        lambda *args: loaders.append(args) or ExecuteProcess(cmd=['not-executed']))
    graph = build_graph(GraphOptions(
        'precise', '/unused/run', composition=True, adapter_implementation='cpp'))
    runtime.launch_actions(graph)
    container = next(node for node in nodes if node['name'] == 'nav2_container')
    assert container['parameters'] == [
        '/unused/run/params/rendered/adapter.yaml', '/unused/run/params/rendered/map_server.yaml',
        '/unused/run/params/rendered/nav2.yaml', '/unused/run/params/rendered/lifecycle.yaml']
    by_name = {item['name']: item for item in components}
    assert len(by_name) == 8
    for spec in graph.processes:
        if spec.name in by_name:
            assert by_name[spec.name]['parameters'] == [spec.parameter_file]
            assert by_name[spec.name]['remappings'] == list(spec.remappings)
    assert loaders[0][0] == '/nav2_container'
    assert [node['name'] for node in nodes] == [
        'wheeltec_robot', 'nav2_container', 'localization_monitor',
        'robot_state_publisher', 'dock_navigator']


@pytest.mark.parametrize('mode', ['tour', 'chassis', 'teleop'])
@pytest.mark.parametrize('name', ['nav2_container', 'unrelated_provider'])
def test_preflight_refuses_existing_nav_container_without_targeting_other_components(
        tmp_path, mode, name):
    """Shared containers are not added to kill patterns or confused with localization."""
    from nav_bringup.navrun.preflight import ProcessInfo
    from test_up_planning import _plan, _request, SELF

    container = ProcessInfo(200, 1, 2.0, (
        '/opt/ros/humble/lib/rclcpp_components/component_container_isolated',
        '--ros-args', '-r', '__node:=' + name), 1000)
    plan = _plan(_request(tmp_path, mode=mode), (SELF, container))
    assert not plan.preflight.targets
    assert bool(plan.errors) == (name == 'nav2_container')
    if plan.errors:
        assert 'automatic cleanup' in plan.errors[0]
    assert not plan.request.run_dir.exists()
