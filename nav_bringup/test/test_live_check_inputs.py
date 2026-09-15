"""Freeze live verification against prepared inputs, never the changing source profile."""

from pathlib import Path

import pytest

from nav_bringup.navcfg.live_inputs import load_live_spec, parameter_sections
from nav_bringup.navrun.preparation import write_bundle
from test_run_preparation import _prepare


def test_namespaced_node_and_nested_parameters_are_not_confused():
    """ROS namespaces use slashes, while nested parameter groups use dots."""
    tree = {'costmap': {'costmap': {'ros__parameters': {'layer': {'enabled': True}}}}}
    assert parameter_sections(tree) == {'/costmap/costmap': {'layer.enabled': True}}


@pytest.mark.parametrize('tree', [
    {'/**': {'ros__parameters': {}}}, {'node': {'ros__parameters': []}},
    {'ros__parameters': {}}, {'node': {'ros__parameters': {'a.b': 1, 'a': {'b': 2}}}},
    {'node': {'ros__parameters': {}}, '/node': {'ros__parameters': {}}}])
def test_ambiguous_or_invalid_parameter_document_is_rejected(tree):
    """The checker cannot silently select a winner for colliding or wildcard settings."""
    with pytest.raises(ValueError):
        parameter_sections(tree)


@pytest.mark.parametrize('mode', ['tour', 'precise', 'bench_slamgt'])
def test_live_inputs_use_exact_mode_bundle_without_modifying_sources(tmp_path, mode):
    """Alternate robot names and costmap children are resolved from the prepared graph."""
    prepared = _prepare(tmp_path, mode)
    root = Path(prepared.options.run_dir)
    root.mkdir()
    write_bundle(prepared)
    before = {path: path.read_bytes() for path in root.rglob('*') if path.is_file()}
    spec, warnings = load_live_spec(root)
    assert spec.map_frame == 'demo_world'
    assert spec.nav_frame == 'demo_axle'
    assert spec.topics[0].topic == '/demo/command'
    assert spec.topics[0].maximum == 1
    assert '/local_costmap/local_costmap' in spec.parameters
    assert '/global_costmap/global_costmap' in spec.parameters
    assert '/recoveries_server' not in spec.parameters
    assert any('/recoveries_server' in warning.message for warning in warnings)
    assert {path: path.read_bytes() for path in before} == before


@pytest.mark.parametrize('mode', ['chassis', 'teleop'])
def test_manual_chassis_modes_do_not_wait_for_localization(tmp_path, mode):
    """A navigation-only check cannot silently impose a localization dependency on teleop."""
    prepared = _prepare(tmp_path, mode)
    root = Path(prepared.options.run_dir)
    root.mkdir()
    write_bundle(prepared)
    with pytest.raises(ValueError, match='navigation modes'):
        load_live_spec(root)
