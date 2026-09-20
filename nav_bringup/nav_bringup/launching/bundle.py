"""读取监督方已备好的启动包，不渲染也不改动任何文件.

只读校验：确认包内文件存在且彼此一致，不重新生成参数，也不修复发现的问题。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Mapping

import yaml

from nav_bringup.launching.graph import build_graph
from nav_bringup.launching.graph import GraphOptions


def load_options(run_dir: str, selectors: Mapping[str, str]) -> GraphOptions:
    """校验已存在的运行包，并套用显式的归属开关.

    校验只覆盖结构：文件存在、身份一致、生命周期顺序与图匹配；
    不检查参数取值是否适合这台车。

    Args:
        run_dir: navrun 已经准备好的绝对目录。
        selectors: 可选的机器人/场地/模式身份核对，以及来自 launch 的
            底盘开关、执行器开关与组合开关。

    Returns:
        其引用文件均存在、生命周期顺序与图一致的选项。

    Raises:
        ValueError: 运行包、选择子或所需参数文件非法。
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
