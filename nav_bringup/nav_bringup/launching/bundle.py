"""Read a supervisor-prepared launch bundle without rendering or changing files."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Mapping

import yaml

from nav_bringup.launching.graph import build_graph
from nav_bringup.launching.graph import GraphOptions


def load_options(run_dir: str, selectors: Mapping[str, str]) -> GraphOptions:
    """Validate an existing run bundle and apply explicit ownership switches.

    Args:
        run_dir: Absolute directory already prepared by navrun.
        selectors: Optional robot/site/mode identity checks, chassis on/off,
            executor true/false, and composition true/false switches from launch.

    Returns:
        Options whose referenced files exist and lifecycle order matches the graph.

    Raises:
        ValueError: If the bundle, selectors or required parameter files are invalid.
    """
    root = Path(run_dir).expanduser()
    if not root.is_absolute() or '..' in root.parts:
        raise ValueError('run_dir must be an absolute normalized directory')
    try:
        data = json.loads((root / 'launch.json').read_text(encoding='utf-8'))
    except (OSError, ValueError) as error:
        raise ValueError(f'cannot read prepared launch bundle: {error}') from error
    if not isinstance(data, dict) or set(data) != {'schema_version', 'robot', 'site', 'options'}:
        raise ValueError('launch bundle must contain schema_version, robot, site and options')
    if type(data['schema_version']) is not int or data['schema_version'] != 1 or not isinstance(
            data['options'], dict):
        raise ValueError('unsupported launch bundle version or options')
    values = dict(data['options'])
    if 'run_dir' in values:
        raise ValueError('bundle paths must be relative to the supplied run_dir')
    for identity in ('robot', 'site'):
        if not isinstance(data[identity], str) or not data[identity]:
            raise ValueError(f'bundle {identity} must be a nonempty string')
        if selectors.get(identity) and selectors[identity] != data[identity]:
            raise ValueError(f'{identity} selector does not match the prepared bundle')
    if selectors.get('mode') and selectors['mode'] != values.get('mode'):
        raise ValueError('mode selector does not match the prepared bundle')
    for switch, key, choices in (
            ('chassis', 'start_chassis', {'on': True, 'off': False}),
            ('executor', 'start_executor', {'true': True, 'false': False}),
            ('composition', 'composition', {'true': True, 'false': False})):
        value = selectors.get(switch, '')
        if value:
            if value not in choices:
                raise ValueError(f'invalid {switch} switch: {value!r}')
            values[key] = choices[value]
    if 'executor_arguments' in values:
        if not isinstance(values['executor_arguments'], list) or not all(
                isinstance(argument, str) for argument in values['executor_arguments']):
            raise ValueError('executor_arguments must be a list of strings')
        values['executor_arguments'] = tuple(values['executor_arguments'])
    try:
        options = GraphOptions(run_dir=str(root), **values)
        graph = build_graph(options)
    except (TypeError, ValueError) as error:
        raise ValueError(f'invalid launch options: {error}') from error
    for process in graph.processes:
        if process.parameter_file and not Path(process.parameter_file).is_file():
            raise ValueError(f'missing rendered parameters: {process.parameter_file}')
        for path in process.configuration_files:
            if not Path(path).is_file():
                raise ValueError(f'missing task configuration: {path}')
    if any(process.name == 'rviz2' for process in graph.processes) and not (
            root / 'inputs/navigation.rviz').is_file():
        raise ValueError('missing prepared RViz configuration')
    if graph.lifecycle_nodes:
        try:
            params = yaml.safe_load((root / 'params/rendered/lifecycle.yaml').read_text())
            names = params['lifecycle_manager_navigation']['ros__parameters']['node_names']
        except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as error:
            raise ValueError('invalid prepared lifecycle parameters') from error
        if names != list(graph.lifecycle_nodes):
            raise ValueError('prepared lifecycle order does not match the selected graph')
    return options
