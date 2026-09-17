"""Read the exact prepared run inputs for runtime verification, without rendering anew."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Mapping, Tuple

from nav_bringup.launching.bundle import load_options
from nav_bringup.launching.graph import build_graph, NAVIGATION_MODES
from nav_bringup.navcfg import render
from nav_bringup.navcfg.live_check import Finding, LiveSpec, TopicRule


def parameter_sections(tree: Mapping[str, Any]) -> Dict[str, Dict[str, Any]]:
    """Resolve explicit ROS YAML node selectors and flatten nested parameter names.

    Args:
        tree: Parsed parameter document, including namespaced costmap sections.

    Returns:
        Qualified node names mapped to dot-separated parameter names.

    Raises:
        ValueError: A selector is ambiguous, malformed or a wildcard, or keys collide.
    """
    output: Dict[str, Dict[str, Any]] = {}

    def flatten(values: Mapping[str, Any], prefix: str, result: Dict[str, Any]) -> None:
        """递归展开参数映射为点分键，非映射叶子保留原对象引用.

        逐项写入 result，失败可能留下部分项；调用方不应把它视为事务。
        不设置递归深度或容器大小限制，也不深复制列表等叶子。

        Args:
            values: 当前参数映射。
            prefix: 已展开父参数前缀，非空时含末尾点号。
            result: 本节点正在累积的参数字典。

        Raises:
            ValueError: 参数键不是非空字符串，或最终点分键重复。
            RecursionError: 循环映射或嵌套过深。
        """
        for key, value in values.items():
            if not isinstance(key, str) or not key:
                raise ValueError('parameter keys must be nonempty strings')
            name = prefix + key
            if isinstance(value, Mapping):
                flatten(value, name + '.', result)
            elif name in result:
                raise ValueError(f'duplicate parameter name: {name}')
            else:
                result[name] = value

    def walk(values: Mapping[str, Any], prefix: str) -> None:
        """递归解析显式节点选择器，把每个节点参数加入外层 output.

        不接受通配符或嵌套绝对选择器；节点名按现有斜杠规则归一化，
        不在此执行完整 ROS 名称语法验证。空映射可以没有任何节点。

        Args:
            values: 选择器树或仅含 ros__parameters 的叶节点映射。
            prefix: 当前不带首尾斜杠的节点路径前缀。

        Raises:
            ValueError: 选择器、参数节、重复节点或展开后的参数键无效。
            RecursionError: 树循环或嵌套过深；此前部分 output 可能已构造。
        """
        if not isinstance(values, Mapping):
            raise ValueError('ROS parameter selectors must contain mappings')
        if 'ros__parameters' in values:
            if not prefix or set(values) != {'ros__parameters'} or not isinstance(
                    values['ros__parameters'], Mapping):
                raise ValueError('invalid ROS parameter section')
            node = '/' + prefix.strip('/')
            if node in output:
                raise ValueError(f'duplicate parameter selector: {node}')
            output[node] = {}
            flatten(values['ros__parameters'], '', output[node])
            return
        for key, value in values.items():
            if (not isinstance(key, str) or not key.strip('/') or '*' in key
                    or '?' in key or (prefix and key.startswith('/'))):
                raise ValueError('live checks require explicit unambiguous node selectors')
            walk(value, '/'.join(filter(None, (prefix, key.strip('/')))))

    walk(tree, '')
    return output


def load_live_spec(run_dir: Path) -> Tuple[LiveSpec, Tuple[Finding, ...]]:
    """Build live checks from a complete supervisor-prepared navigation run.

    Args:
        run_dir: Existing run record with launch.json and frozen rendered inputs.

    Returns:
        Expected observations and warnings for dormant parameter-file selectors.

    Raises:
        ValueError: Inputs are invalid or not a navigation mode.
        OSError: A prepared input cannot be read.
    """
    options = load_options(str(run_dir), {})
    if options.mode not in NAVIGATION_MODES:
        raise ValueError('localization live checks apply only to navigation modes')
    root = Path(options.run_dir)
    profile = render.load_yaml(root / 'params/inputs/robot.yaml')
    mode_path = root / 'params/inputs/mode.yaml'
    mode = render.parse_mode(render.load_yaml(mode_path), mode_path)
    if mode.name != options.mode or not isinstance(profile, dict):
        raise ValueError('prepared mode/profile does not match launch bundle')
    profile = render.deep_merge(profile, mode.profile)
    graph = build_graph(options)
    expected: Dict[str, Dict[str, Any]] = {}
    warnings = []
    paths = dict.fromkeys(process.parameter_file for process in graph.processes
                          if process.parameter_file)
    for path in paths:
        owned = {'/' + process.name for process in graph.processes
                 if process.parameter_file == path and process.ros_node}
        # A selector only applies to processes actually loading this specific file.
        if '/controller_server' in owned:
            owned.add('/local_costmap/local_costmap')
        if '/planner_server' in owned:
            owned.add('/global_costmap/global_costmap')
        tree = render.load_yaml(Path(path))
        if not isinstance(tree, dict):
            raise ValueError(f'invalid rendered parameters: {path}')
        for node, values in parameter_sections(tree).items():
            if node not in owned:
                warnings.append(Finding('warn', 'inactive_parameter_selector',
                                        f'{node}: no selected node consumes this section'))
                continue
            if node in expected and expected[node] != values:
                raise ValueError(f'conflicting rendered parameters for {node}')
            expected[node] = values
    required = {'/' + process.name for process in graph.processes if process.parameter_file}
    if not required.issubset(expected):
        raise ValueError('rendered files omit selected process parameter sections')

    def value(key: str) -> str:
        result = render.profile_value(profile, key)
        if not isinstance(result, str) or not result:
            raise ValueError(f'{key} must be a nonempty string')
        return result

    def topic(name: str) -> str:
        return '/' + name.lstrip('/')

    rules = {topic(options.chassis_command_topic): TopicRule(
        topic(options.chassis_command_topic), 1, 1, 'geometry_msgs/msg/Twist')}
    for name, message_type in (
            (options.chassis_odom_topic, 'nav_msgs/msg/Odometry'),
            (options.nav_command_topic if options.velocity_smoother else
             options.smooth_command_topic, 'geometry_msgs/msg/Twist'),
            (options.smooth_command_topic, 'geometry_msgs/msg/Twist'),
            (value('localization.odom_topic'), 'nav_msgs/msg/Odometry'),
            (value('nav2.odom_topic'), 'nav_msgs/msg/Odometry'),
            (value('nav2.map_topic'), 'nav_msgs/msg/OccupancyGrid')):
        rules.setdefault(topic(name), TopicRule(topic(name), message_type=message_type))
    if options.scan_bridge:
        for name, message_type in ((options.cloud_topic, 'sensor_msgs/msg/PointCloud2'),
                                   (options.scan_topic, 'sensor_msgs/msg/LaserScan')):
            rules.setdefault(topic(name), TopicRule(topic(name), message_type=message_type))
    health = profile.get('localization_health', {})
    if not isinstance(health, Mapping):
        raise ValueError('localization_health must be a mapping')
    spec = LiveSpec(value('frames.map'), value('frames.odom'), value('frames.nav_base'),
                    tuple(rules.values()), expected, health.get('max_tf_age_sec', 0.5))
    return spec, tuple(warnings)
