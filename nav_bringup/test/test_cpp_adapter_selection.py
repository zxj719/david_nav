"""The C++ implementation is explicitly selected without changing adapter contracts."""

from dataclasses import replace

import pytest
import yaml

from nav_bringup.launching.graph import build_graph, GraphOptions
from nav_bringup.navcfg import schema
from nav_bringup.navrun.preparation import prepare_bundle
from test_run_preparation import _inputs, SOURCE
from test_lightweight_modes import _plan


@pytest.mark.parametrize('mode', ['tour', 'precise', 'chassis', 'teleop'])
def test_profile_selects_cpp_entry_with_same_name_and_parameters(tmp_path, mode):
    """Only the executable changes; source profile defaults and Python remain intact."""
    inputs = _inputs(tmp_path, mode)
    catalog = schema.load_catalog(schema.find_source_schemas(SOURCE))
    options = GraphOptions(mode, str(tmp_path / 'run'), supervisor_keyboard=mode == 'teleop')
    baseline = prepare_bundle(options, inputs, catalog)
    overlay = yaml.safe_load(inputs.mode.read_text())
    overlay.setdefault('profile', {}).setdefault('chassis', {}).setdefault('adapter', {})[
        'implementation'] = 'cpp'
    inputs.mode.write_text(yaml.safe_dump(overlay))
    selected = prepare_bundle(options, inputs, catalog)
    before = next(spec for spec in build_graph(baseline.options).processes
                  if spec.name == 'chassis_adapter')
    after = next(spec for spec in build_graph(selected.options).processes
                 if spec.name == 'chassis_adapter')
    assert before.executable == 'chassis_adapter'
    assert after == replace(before, executable='chassis_adapter_cpp')
    assert dict(selected.artifacts)['params/rendered/adapter.yaml'] == dict(
        baseline.artifacts)['params/rendered/adapter.yaml']


def test_invalid_backend_rejects_before_graph_creation():
    """Unknown implementations cannot silently fall back to Python."""
    with pytest.raises(ValueError, match='implementation'):
        build_graph(GraphOptions('tour', '/unused/run', adapter_implementation='other'))


@pytest.mark.parametrize('mode', ['tour', 'precise'])
def test_david_cpp_mode_files_change_only_the_backend(tmp_path, mode):
    """Installed experiment data keeps all numerical settings and default mode behavior."""
    baseline = _plan(tmp_path / 'python', mode)
    selected = _plan(tmp_path / 'cpp', mode, 'cpp')
    assert selected.bundle.options.adapter_implementation == 'cpp'
    actual = dict(selected.bundle.artifacts)
    original = dict(baseline.bundle.artifacts)
    assert actual['params/rendered/adapter.yaml'] == original['params/rendered/adapter.yaml']
    # Nav2 paths are run-local; compare independent rendered inputs, not those paths.
    assert actual['params/inputs/nav2_base.yaml'] == original['params/inputs/nav2_base.yaml']
    assert selected.bundle.options.velocity_smoother
