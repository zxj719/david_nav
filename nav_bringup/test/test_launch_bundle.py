"""Validate prepared launch bundles without launching ROS or touching robot files."""

from dataclasses import asdict
import json
from pathlib import Path

import pytest
import yaml

from nav_bringup.launching.bundle import load_options
from nav_bringup.launching.graph import build_graph
from nav_bringup.launching.graph import GraphOptions


def _prepare(root, mode='tour'):
    options = GraphOptions(mode, str(root))
    values = asdict(options)
    values.pop('run_dir')
    data = {'schema_version': 1, 'robot': 'example', 'site': 'offline', 'options': values}
    (root / 'launch.json').write_text(json.dumps(data))
    graph = build_graph(options)
    for spec in graph.processes:
        paths = spec.configuration_files + ((spec.parameter_file,) if spec.parameter_file else ())
        for filename in paths:
            path = Path(filename)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('{}')
    if graph.lifecycle_nodes:
        (root / 'params/rendered/lifecycle.yaml').write_text(yaml.safe_dump({
            'lifecycle_manager_navigation': {'ros__parameters': {
                'node_names': list(graph.lifecycle_nodes)}}}))
    return data


@pytest.mark.parametrize('mode', ['tour', 'precise', 'chassis', 'teleop'])
def test_bundle_reads_all_four_modes_without_modifying_files(tmp_path: Path, mode: str) -> None:
    """Bundle selection is read-only and keeps supplied identities."""
    _prepare(tmp_path, mode)
    before = {path: path.read_bytes() for path in tmp_path.rglob('*') if path.is_file()}
    options = load_options(str(tmp_path), {'robot': 'example', 'site': 'offline', 'mode': mode})
    assert options.mode == mode
    assert before == {path: path.read_bytes() for path in tmp_path.rglob('*') if path.is_file()}


@pytest.mark.parametrize('selectors', [
    {'robot': 'wrong'}, {'site': 'wrong'}, {'mode': 'precise'}, {'chassis': 'maybe'},
    {'executor': 'yes'}, {'composition': 'yes'},
])
def test_bad_selectors_fail_before_launch(tmp_path: Path, selectors) -> None:
    """An explicit selector cannot silently choose a different prepared robot or mode."""
    _prepare(tmp_path)
    with pytest.raises(ValueError):
        load_options(str(tmp_path), selectors)


def test_missing_files_and_wrong_lifecycle_order_are_rejected(tmp_path: Path) -> None:
    """Node parameters and activation order must be prepared before starting a driver."""
    _prepare(tmp_path)
    lifecycle = tmp_path / 'params/rendered/lifecycle.yaml'
    lifecycle.write_text('lifecycle_manager_navigation: {ros__parameters: {node_names: []}}')
    with pytest.raises(ValueError, match='lifecycle order'):
        load_options(str(tmp_path), {})
    lifecycle.unlink()
    with pytest.raises(ValueError, match='missing rendered'):
        load_options(str(tmp_path), {})


def test_external_chassis_override_removes_driver_file_requirement(tmp_path: Path) -> None:
    """A manually owned driver is neither resolved nor launched by this bundle."""
    _prepare(tmp_path)
    (tmp_path / 'params/rendered/driver.yaml').unlink()
    options = load_options(str(tmp_path), {'chassis': 'off'})
    assert not options.start_chassis


@pytest.mark.parametrize('change', [
    {'schema_version': 2}, {'schema_version': True}, {'extra': 'unknown'}, {'options': []},
    {'options': {'mode': 'tour', 'run_dir': '/unrelated'}},
    {'options': {'mode': 'tour', 'executor_arguments': 'not a list'}},
])
def test_malformed_bundle_is_rejected(tmp_path: Path, change) -> None:
    """Unknown bundle layouts and command types do not reach process construction."""
    data = _prepare(tmp_path)
    data.update(change)
    (tmp_path / 'launch.json').write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_options(str(tmp_path), {})
