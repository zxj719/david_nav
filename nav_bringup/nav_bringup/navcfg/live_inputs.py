"""从已准备的运行记录构造在线校验规格，不重新渲染配置.

只读取冻结输入和所选进程实际引用的参数文件，不查询 ROS 或写入记录。
多文件读取不是原子快照；得到的是预期参数，不应标为已经观测的生效参数。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Mapping, Tuple

from nav_bringup.launching.bundle import load_options
from nav_bringup.launching.graph import build_graph, NAVIGATION_MODES
from nav_bringup.navcfg import render
from nav_bringup.navcfg.live_check import Finding, LiveSpec, TopicRule


def parameter_sections(tree: Mapping[str, Any]) -> Dict[str, Dict[str, Any]]:
    """解析显式 ROS 节点选择器并把嵌套参数展开为点分键.

    不接受通配符、混合参数/选择器节点或重复展开键；只校验现有选择器规则，
    不执行完整 ROS 名称语法检查。叶子对象保留引用，不深复制或限制递归深度。

    Args:
        tree: 解析后的参数树，可含带命名空间的 costmap 节点。

    Returns:
        带前导斜杠的节点名到参数字典的映射；参数键使用点分路径。

    Raises:
        ValueError: 选择器、参数节或键不合法，或归一化后出现冲突。
        RecursionError: 循环映射或嵌套过深。
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
    """从完整导航运行记录加载预期参数、话题和定位契约.

    只认所选进程实际使用的参数文件，额外选择器记警告，不当作活动节点。
    机器人与模式输入在内存中叠加；不改源文件，也不证明运行节点已采用这些值。

    Args:
        run_dir: 含 launch.json、冻结档案及渲染结果的现有运行目录。

    Returns:
        在线观察规格及未被所选进程消费的参数节警告元组。

    Raises:
        ValueError: 模式/输入不符、非导航模式、缺节点参数或参数内容冲突。
        OSError: 运行记录或准备好的输入无法读取。
        RecursionError: 参数树嵌套过深或递归结构无法展开。
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
    health = profile.get('localization_health', {})
    if not isinstance(health, Mapping):
        raise ValueError('localization_health must be a mapping')
    spec = LiveSpec(value('frames.map'), value('frames.odom'), value('frames.nav_base'),
                    tuple(rules.values()), expected, health.get('max_tf_age_sec', 0.5))
    return spec, tuple(warnings)
