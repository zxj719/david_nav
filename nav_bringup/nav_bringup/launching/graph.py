"""描述四种受管启动模式，不创建任何进程或文件.

参数文件由监督方预先渲染到本次运行目录；本模块只负责点名这些文件，并描述
归属、重启预算与就绪依赖，真正的启动交给 launch 适配层。本模块是纯描述：
不探测主机、不读参数内容、不启动任何东西，也从不探测或启动现场的定位栈。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import PurePath
from typing import Tuple


MODES = ('tour', 'precise', 'chassis', 'teleop')
NAVIGATION_MODES = MODES[:2]
CORE_NODES = (
    ('nav2_controller', 'controller_server'),
    ('nav2_planner', 'planner_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator'),
    ('nav2_velocity_smoother', 'velocity_smoother'),
)


@dataclass(frozen=True)
class GraphOptions:
    """navrun 解析后传入的启动图开关，不是机器人调参.

    这里只有"启动哪些进程、谁拥有它们"的决定；速度、容差等调参一律来自
    已渲染的参数文件，本结构不读取也不覆盖它们。

    Attributes:
        mode: 四种受支持模式之一。
        run_dir: 存放已渲染参数文件的绝对目录。
        start_chassis: 本次启动是否拥有一个新起的底盘驱动。
        start_executor: Nav2 就绪后是否启用上位机执行器。
        map_server: 本次启动是否拥有地图发布者。
        robot_state: 是否发布给定的机器人描述。
        rviz: 是否按给定配置启动 RViz。
        chassis_delay_sec: 自有驱动起来后，延迟多久再启动导航。
        driver_restarts: 整栈关停前允许的驱动重启次数。
        rviz_delay_sec: 导航启动后再延迟多久启动 RViz。
        driver_package: 由机器人档案选定的驱动包。
        driver_executable: 由档案选定的驱动入口。
        nav_command_topic: 启用平滑器时，控制器与恢复行为的输出话题。
        smooth_command_topic: 适配器输入，由平滑器或 Nav2 直接提供。
        chassis_command_topic: 适配器输出，由驱动消费。
        chassis_odom_topic: 驱动发布的轮式里程计。
        executor_arguments: 共享任务管理器的场地/传输命令行参数，必须显式给出。
        ready_timeout_sec: 拒绝启动执行器前的最长墙钟等待。
        ready_action: 用于判就绪的导航动作服务端；只检查存在，不下发任何目标。
        supervisor_keyboard: 遥控键盘由 navrun 直接持有，不进 launch。
        velocity_smoother: 除非显式关闭，保留原有的平滑中转。
        adapter_implementation: 显式选择 python 或 cpp 后端，默认仍为 python。
        composition: 显式把可组合的节点放进同一个隔离容器。
    """

    mode: str
    run_dir: str
    start_chassis: bool = True
    start_executor: bool = False
    map_server: bool = True
    robot_state: bool = True
    rviz: bool = False
    chassis_delay_sec: float = 0.0
    driver_restarts: int = 1
    rviz_delay_sec: float = 0.0
    driver_package: str = 'wheeltec_driver'
    driver_executable: str = 'wheeltec_driver_node'
    nav_command_topic: str = 'cmd_vel_nav'
    smooth_command_topic: str = 'cmd_vel_smoothed'
    chassis_command_topic: str = 'cmd_vel'
    chassis_odom_topic: str = 'odom'
    executor_arguments: Tuple[str, ...] = ()
    ready_timeout_sec: float = 90.0
    ready_action: str = '/navigate_to_pose'
    supervisor_keyboard: bool = False
    velocity_smoother: bool = True
    adapter_implementation: str = 'python'
    composition: bool = False


@dataclass(frozen=True)
class ProcessSpec:
    """一个逻辑程序/节点及其独立的运行约定.

    Attributes:
        name: 该进程/节点在图内的稳定身份。
        package: 提供可执行文件的 ROS 包。
        executable: 该包内的可执行文件名。
        parameter_file: 本次运行的 ROS 参数文件；普通程序为空串。
        remappings: 话题重映射，不含机器人专有的坐标系字面量。
        arguments: 参数文件之外的命令行参数。
        delay_sec: 相对 launch 起点的启动延迟。
        restart_limit: 触发整栈关停前允许的重启次数。
        critical: 非预期的最终退出是否导致整张图关停。
        wait_for_navigation: 进程执行前是否先过一道就绪门禁。
        ros_node: 是否允许追加 ROS 节点名/参数类命令行参数。
        ready_timeout_sec: 可选就绪门禁的期限。
        ready_action: 可选就绪门禁使用的动作名。
        configuration_files: 启动前必须已备好的其它命令行输入文件。
    """

    name: str
    package: str
    executable: str
    parameter_file: str = ''
    remappings: Tuple[Tuple[str, str], ...] = ()
    arguments: Tuple[str, ...] = ()
    delay_sec: float = 0.0
    restart_limit: int = 0
    critical: bool = True
    wait_for_navigation: bool = False
    ros_node: bool = True
    ready_timeout_sec: float = 90.0
    ready_action: str = '/navigate_to_pose'
    configuration_files: Tuple[str, ...] = ()


@dataclass(frozen=True)
class LaunchGraph:
    """自有的逻辑图，到 launch 边界才转成物理进程.

    Attributes:
        mode: 选定的模式。
        processes: 自有的程序/节点，按声明顺序排列。
        lifecycle_nodes: Nav2 的激活顺序；自有地图服务器排在最前。
        supervisor_keyboard: 把显式托管的键盘排除在 launch 动作之外。
        composition: 是否在 launch 边界把本逻辑图合成到同一容器里。
    """

    mode: str
    processes: Tuple[ProcessSpec, ...]
    lifecycle_nodes: Tuple[str, ...]
    supervisor_keyboard: bool = False
    composition: bool = False


def _validate(options: GraphOptions) -> None:
    """在生成启动图前验证已经解析的模式、开关、路径与秒制预算.

    Args:
        options: 完整 GraphOptions；不在此读取环境或替换默认值。

    Raises:
        ValueError: 类型、范围、名称、路径或模式与功能组合不合法。
    """
    if options.adapter_implementation not in ('python', 'cpp'):
        raise ValueError('adapter implementation must be python or cpp')
    if options.mode not in MODES:
        raise ValueError(f'unknown launch mode: {options.mode!r}')
    if not PurePath(options.run_dir).is_absolute() or '..' in PurePath(options.run_dir).parts:
        raise ValueError('run_dir must be an absolute normalized run directory')
    for name in ('start_chassis', 'start_executor', 'map_server',
                 'robot_state', 'rviz', 'supervisor_keyboard', 'velocity_smoother', 'composition'):
        if not isinstance(getattr(options, name), bool):
            raise ValueError(f'{name} must be a resolved boolean')
    for delay in (options.chassis_delay_sec, options.rviz_delay_sec):
        if isinstance(delay, bool) or not math.isfinite(delay) or delay < 0:
            raise ValueError('launch delays must be finite and nonnegative')
    if type(options.driver_restarts) is not int or options.driver_restarts < 0:
        raise ValueError('driver_restarts must be a nonnegative integer')
    if isinstance(options.ready_timeout_sec, bool) or not math.isfinite(
            options.ready_timeout_sec) or options.ready_timeout_sec <= 0:
        raise ValueError('ready_timeout_sec must be finite and positive')
    if not isinstance(options.executor_arguments, tuple) or not all(
            isinstance(argument, str) for argument in options.executor_arguments):
        raise ValueError('executor_arguments must be a tuple of strings')
    if options.start_executor and options.mode not in ('tour', 'precise'):
        raise ValueError('the executor is only valid in tour or precise mode')
    if options.supervisor_keyboard and options.mode != 'teleop':
        raise ValueError('supervisor keyboard ownership is only valid in teleop mode')
    if options.composition and options.mode not in NAVIGATION_MODES:
        raise ValueError('composition is only valid in a navigation mode')
    for name in ('driver_package', 'driver_executable', 'nav_command_topic',
                 'smooth_command_topic', 'chassis_command_topic', 'chassis_odom_topic',
                 'ready_action'):
        if not isinstance(getattr(options, name), str) or not getattr(options, name).strip():
            raise ValueError(f'{name} must be a nonempty string')


def build_graph(options: GraphOptions) -> LaunchGraph:
    """解析出逻辑节点图，全程不导入任何运行时实现.

    只做结构决策：不检查可执行文件是否装好、不读参数文件内容、不碰 ROS；
    因此返回的图能通过校验，不等于这台机器上真的能起来。

    Args:
        options: 已解析的模式开关与档案给出的话题名。

    Returns:
        不可变的进程图与生命周期激活顺序。

    Raises:
        ValueError: 开关、模式、路径或重启/延迟取值非法。
    """
    _validate(options)
    root = PurePath(options.run_dir)

    def params(name: str) -> str:
        return str(root / 'params' / 'rendered' / f'{name}.yaml')

    processes = []
    if options.start_chassis:
        processes.append(ProcessSpec(
            'wheeltec_robot', options.driver_package, options.driver_executable,
            params('driver'),
            (('cmd_vel', options.chassis_command_topic), ('odom', options.chassis_odom_topic)),
            restart_limit=options.driver_restarts))
    delay = options.chassis_delay_sec if options.start_chassis else 0.0
    processes.append(ProcessSpec(
        'chassis_adapter', 'chassis_adapter',
        'chassis_adapter_cpp' if options.adapter_implementation == 'cpp' else 'chassis_adapter',
        params('adapter'),
        delay_sec=delay))
    if options.mode == 'chassis':
        return LaunchGraph(options.mode, tuple(processes), ())
    if options.mode == 'teleop':
        processes.append(ProcessSpec(
            'chassis_teleop', 'chassis_teleop',
            'chassis_teleop_managed' if options.supervisor_keyboard else 'chassis_teleop',
            params('teleop'),
            delay_sec=delay))
        return LaunchGraph(options.mode, tuple(processes), (), options.supervisor_keyboard)

    tf = (('/tf', 'tf'), ('/tf_static', 'tf_static'))
    processes.append(ProcessSpec(
        'localization_monitor', 'nav_perception_iface', 'localization_monitor',
        params('localization_health'), tf, delay_sec=delay, critical=False))
    lifecycle = []
    if options.map_server:
        lifecycle.append('map_server')
        processes.append(ProcessSpec(
            'map_server', 'nav2_map_server', 'map_server', params('map_server'),
            tf, delay_sec=delay))
    for package, executable in CORE_NODES:
        if executable == 'velocity_smoother' and not options.velocity_smoother:
            continue
        lifecycle.append(executable)
        remappings = tf
        if executable != 'planner_server':
            remappings += (('cmd_vel', options.nav_command_topic if options.velocity_smoother
                            else options.smooth_command_topic),)
        if executable == 'velocity_smoother':
            remappings += (('cmd_vel_smoothed', options.smooth_command_topic),)
        processes.append(ProcessSpec(
            executable, package, executable, params('nav2'), remappings, delay_sec=delay))
    processes.append(ProcessSpec(
        'lifecycle_manager_navigation', 'nav2_lifecycle_manager', 'lifecycle_manager',
        params('lifecycle'), delay_sec=delay))
    if options.robot_state:
        processes.append(ProcessSpec(
            'robot_state_publisher', 'robot_state_publisher', 'robot_state_publisher',
            params('robot_state'), delay_sec=delay))
    if options.mode == 'precise' or options.start_executor:
        processes.append(_task_manager(options, params, delay))
    if options.rviz:
        processes.append(ProcessSpec(
            'rviz2', 'rviz2', 'rviz2', params('rviz'),
            arguments=('-d', str(root / 'inputs' / 'navigation.rviz')),
            delay_sec=delay + options.rviz_delay_sec, critical=False))
    return LaunchGraph(options.mode, tuple(processes), tuple(lifecycle),
                       composition=options.composition)


def _task_manager(options, params, delay):
    """构造唯一任务管理器描述，根据模式接入 Dock 并保留显式动作名覆盖.

    Args:
        options: 已验证的启动选项；start_executor 决定是否开启 WS。
        params: 将参数资产名解析为运行目录文件路径的函数。
        delay: 该进程启动延迟（秒）。

    Returns:
        未执行的 ProcessSpec，包含配置依赖与导航就绪条件；ROS-only precise
        入口和带 WS 的包装入口沿用各自的参数传入方式。
    """
    profile = params('executor_profile')
    files = (profile,)
    arguments = options.executor_arguments
    if not any(arg == '--action-name' or arg.startswith('--action-name=') for arg in arguments):
        arguments += ('--action-name', options.ready_action)
    arguments += ('--robot-profile', profile)
    precise = options.mode == 'precise'
    if precise:
        files += (params('dock'),)
        arguments += ('--dock-config-file', params('dock'))
    arguments += ('--websocket' if options.start_executor else '--no-websocket',)
    return ProcessSpec(
        'navigation_executor' if options.start_executor else 'dock_navigator',
        'nav_executor', 'nav_task_manager',
        parameter_file=params('dock') if precise and not options.start_executor else '',
        arguments=arguments, delay_sec=delay,
        wait_for_navigation=options.start_executor, ros_node=not options.start_executor,
        ready_timeout_sec=options.ready_timeout_sec, ready_action=options.ready_action,
        configuration_files=files)


def exit_decision(process: ProcessSpec, prior_restarts: int, shutting_down: bool) -> str:
    """为一个自有进程的退出决定重启还是关停.

    纯决策函数：不杀进程、不重启、不发信号，只返回该怎么做。

    Args:
        process: 已退出进程的运行约定。
        prior_restarts: 该进程谱系已经用掉的重启次数。
        shutting_down: 是否已经处于整体关停过程中。

    Returns:
        ignore、restart 或 shutdown 之一。已在关停中时一律返回 ignore：
        关停期的进程退出本就在预期之内，无需再做动作，更不会重启。
        正常期预算未用尽返回 restart；预算用尽后，关键进程返回 shutdown，
        非关键进程返回 ignore。

    Raises:
        ValueError: 重启计数非法。
    """
    if type(prior_restarts) is not int or prior_restarts < 0:
        raise ValueError('prior_restarts must be a nonnegative integer')
    if shutting_down:
        return 'ignore'
    if prior_restarts < process.restart_limit:
        return 'restart'
    return 'shutdown' if process.critical else 'ignore'
