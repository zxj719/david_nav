"""只读的启动规划，使用与 ROS launch 完全相同的运行包与启动图.

返回的计划只是预览，不构成改动主机的授权；真正的运行期接线仍是显式的集成工作。
特别地，dry-run 校验通过只说明配置与入口点齐备，从不意味着硬件被动过或验证过。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
import re
from typing import Any, Callable, Dict, Optional, Sequence, Tuple

import yaml

from nav_bringup.launching.graph import build_graph, GraphOptions, LaunchGraph, MODES
from nav_bringup.launching.graph import NAVIGATION_MODES
from nav_bringup.launching.composition import component_specs, physical_graph
from nav_bringup.launching.executables import resolve_component
from nav_bringup.navcfg import render, schema
from nav_bringup.navrun.platform import plan_platform, PlatformStep
from nav_bringup.navrun.preflight import plan_preflight, PreflightPlan, ProcessInfo
from nav_bringup.navrun.preflight import is_task_manager
from nav_bringup.navrun.preparation import BundleInputs, merge_overlay, OVERLAY_ARTIFACT
from nav_bringup.navrun.preparation import OVERLAY_NAME, PreparedBundle, prepare_bundle
from nav_bringup.navrun.recording import plan_recording, RecordingPlan
from nav_bringup.navrun.teleop_options import apply_options, TeleopOptions


@dataclass(frozen=True)
class UpRequest:
    """用户选择子，与 ROS 资源发现无关.

    Attributes:
        mode: 五种启动模式之一。
        robot: 机器人名，由对应的 name_config 包提供。
        site: 该包内的场地目录名；底盘类工具忽略。
        run_dir: 预期的绝对运行目录；规划阶段不会创建它。
        chassis: auto、on 或 off；auto 由一次只读进程快照决定。
        record: 请求的记录策略：light、full 或 off。
        no_executor: 关闭 WS 传输；精准模式仍保留本地入位入口。
        teleop_options: 显式的键位/档位，与源文件分开冻结。
        mode_file: 可选的完整模式差异文件路径；不会改变模式身份。
        composition: 显式选择把 Nav2 放进一个隔离容器。
        oa: 显式请求前向局部避障；叠加档由机器人自己的配置包提供，只在导航模式可用。
    """

    mode: str
    robot: str
    site: str
    run_dir: Path
    chassis: str = 'auto'
    record: str = 'light'
    no_executor: bool = False
    teleop_options: TeleopOptions = TeleopOptions()
    mode_file: Optional[Path] = None
    composition: bool = False
    oa: bool = False


@dataclass(frozen=True)
class UpPlan:
    """一个已校验的运行包，外加尚未满足的部署条件.

    errors 非空仍然是一个合法的计划对象：它如实列出缺什么，而不是抛异常，
    以便一次看全所有缺失项。

    Attributes:
        request: 原始选择子。
        bundle: 不可变的已渲染配置。
        graph: 依据已渲染选项选出的逻辑节点/程序。
        preflight: 底盘归属、待清理身份与受保护进程。
        platform: 拟执行的非交互平台命令；规划阶段不执行。
        executables: 物理进程名与解析出的已安装可执行文件路径。
        errors: 缺失的依赖或尚未完成的绑定要求。
        recording: 显式选定的本次运行 rosbag；规划阶段不启动它。
    """

    request: UpRequest
    bundle: PreparedBundle
    graph: LaunchGraph
    preflight: PreflightPlan
    platform: Tuple[PlatformStep, ...]
    executables: Tuple[Tuple[str, str], ...]
    errors: Tuple[str, ...]
    recording: RecordingPlan


def _validate(request: UpRequest) -> None:
    """校验启动请求及遥控覆盖，拒绝不明确路径或不支持的功能组合.

    Args:
        request: 未执行的模式、资源名称、底盘/记录选择和输出目录请求。

    Raises:
        ValueError: 标识、枚举、布尔开关、模式覆盖或运行目录无效。
    """
    request.teleop_options.values(request.mode)
    if request.mode not in MODES:
        raise ValueError('unknown mode')
    for name in ('robot', 'site'):
        value = getattr(request, name)
        if not isinstance(value, str) or not re.fullmatch(r'[a-z][a-z0-9_]*', value):
            raise ValueError(f'{name} must be a lowercase resource name')
    if request.chassis not in ('auto', 'on', 'off'):
        raise ValueError('chassis must be auto, on or off')
    if request.record not in ('light', 'full', 'off'):
        raise ValueError('record must be light, full or off')
    if type(request.no_executor) is not bool:
        raise ValueError('no_executor must be boolean')
    if type(request.composition) is not bool or (
            request.composition and request.mode not in NAVIGATION_MODES):
        raise ValueError('composition must be boolean and is only valid for navigation')
    if type(request.oa) is not bool or (request.oa and request.mode not in NAVIGATION_MODES):
        raise ValueError('obstacle avoidance must be boolean and needs a mode with a costmap')
    if request.mode_file is not None and not isinstance(request.mode_file, Path):
        raise ValueError('mode_file must be a path')
    if (not isinstance(request.run_dir, Path) or not request.run_dir.is_absolute()
            or '..' in request.run_dir.parts or request.run_dir == Path('/')):
        raise ValueError('run_dir must be an absolute normalized run directory')


def discover_inputs(
    request: UpRequest, package_share: Callable[[str], Path],
) -> BundleInputs:
    """挑选源码或已安装的资源，不导入任何机器人包的代码.

    只解析路径，不读取文件内容，因此返回的路径存在与否要由后续步骤负责。

    Args:
        request: 用户选择子，在任何资源查找之前先行校验。
        package_share: 只读解析器，返回某个包的资源目录。

    Returns:
        供共享离线渲染器使用的路径集合。非导航模式不会去找 Nav2、插件、场地、
        地图或 URDF 资源。

    Raises:
        ValueError: 选择子非法。
        LookupError: 缺少某个必需的资源包。
    """
    _validate(request)
    robot = Path(package_share(request.robot + '_config'))
    navigation = request.mode in NAVIGATION_MODES
    bringup = Path(package_share('nav_bringup')) if navigation else robot
    plugins = Path(package_share('nav_plugins')) if navigation else robot
    return BundleInputs(
        robot / 'robot' / (request.robot + '.yaml'),
        request.mode_file.expanduser().resolve(strict=True) if request.mode_file is not None
        else robot / 'modes' / (request.mode + '.yaml'),
        bringup / 'config/nav2_base.yaml', bringup / 'config/nav2_bindings.yaml',
        plugins / 'behavior_trees', plugins / 'config/bt_bindings.yaml',
        site=robot / 'sites' / request.site / 'site.yaml' if navigation else None,
        urdf=robot / 'urdf' / (request.robot + '.urdf') if navigation else None,
        overlay=robot / 'modes' / 'overlays' / (OVERLAY_NAME + '.yaml') if request.oa else None)


def plan_up(
    request: UpRequest, inputs: BundleInputs, catalog: schema.ParameterCatalog,
    processes: Sequence[ProcessInfo], own_pid: int,
    resolve: Callable[[str, str], str],
    resolve_plugin: Callable[[str, str], str] = resolve_component,
) -> UpPlan:
    """在改动主机之前先准备配置并逐个检查入口点.

    Args:
        request: 请求的启动选择子。
        inputs: 已解析的配置路径。
        catalog: 各包自带的参数 schema。
        processes: 完整的只读进程快照。
        own_pid: 快照中本监督进程自己的身份。
        resolve: 只读的包/可执行文件解析器，测试可注入。
        resolve_plugin: 只读的组件资源/库解析器。

    Returns:
        计划中会列出**全部**缺失可执行文件，而不是遇到第一个就停；
        过程中不做清理、不加锁、不碰 ROS、不写任何文件。

    Raises:
        ValueError: 输入非法、身份不匹配，或配置在读取期间发生变化。
        OSError: 某个配置文件读不到。
    """
    _validate(request)
    initial_profile = inputs.profile.read_bytes()
    initial_mode = inputs.mode.read_bytes()
    profile = yaml.safe_load(initial_profile)
    if not isinstance(profile, dict):
        raise ValueError('robot profile must be a mapping')
    mode = render.parse_mode(yaml.safe_load(initial_mode), inputs.mode)
    if mode.name != request.mode:
        raise ValueError('mode overlay does not match requested mode')
    initial_overlay = inputs.overlay.read_bytes() if inputs.overlay is not None else None
    mode = merge_overlay(
        mode, None if initial_overlay is None else yaml.safe_load(initial_overlay),
        inputs.overlay)
    effective = apply_options(render.deep_merge(profile, mode.profile),
                              request.mode, request.teleop_options)
    preflight = plan_preflight(effective, processes, own_pid, chassis=request.chassis)
    navigation = request.mode in NAVIGATION_MODES
    executor = request.mode in ('tour', 'precise') and not request.no_executor
    scan = effective.get('sensors', {}).get('scan_from_cloud', {}).get('enabled', False)
    options = GraphOptions(
        request.mode, str(request.run_dir), start_chassis=preflight.start_chassis,
        start_executor=executor, scan_bridge=scan if navigation else False,
        supervisor_keyboard=request.mode == 'teleop', composition=request.composition)
    bundle = prepare_bundle(options, inputs, catalog, request.teleop_options)
    artifacts = dict(bundle.artifacts)
    if (artifacts['params/inputs/robot.yaml'] != initial_profile
            or artifacts['params/inputs/mode.yaml'] != initial_mode
            or artifacts.get(OVERLAY_ARTIFACT) != initial_overlay):
        raise ValueError('configuration changed during startup planning; retry')
    if bundle.robot != request.robot or (navigation and bundle.site != request.site):
        raise ValueError('requested robot/site identity does not match the rendered bundle')
    graph = build_graph(bundle.options)
    errors = []
    for process in processes:
        if is_task_manager(process):
            errors.append(f'existing task manager pid {process.pid} requires inspection; '
                          'automatic cleanup of shared task managers is disabled')
        if (process.argv and Path(process.argv[0]).name in (
                'component_container', 'component_container_mt', 'component_container_isolated')
                and any(argument in ('__node:=nav2_container', '__name:=nav2_container')
                        for argument in process.argv[1:])):
            errors.append(f'existing nav2_container pid {process.pid} requires inspection; '
                          'automatic cleanup of shared containers is disabled')
    if request.mode == 'teleop' and preflight.targets:
        errors.append('teleop refuses existing navigation/residual processes; '
                      'stop and inspect them before starting keyboard control')
    executables = []
    requirements = [(spec.name, spec.package, spec.executable)
                    for spec in physical_graph(graph).processes]
    for spec, plugin in component_specs(graph):
        try:
            resolve_plugin(spec.package, plugin)
        except (ValueError, LookupError, OSError) as error:
            errors.append(f'{spec.package}/{plugin}: {error}')
    if navigation:
        requirements.append(('interface_probe', 'nav_perception_iface', 'nav_interface_check'))
    if executor:
        requirements.append(('ready_gate', 'nav_bringup', 'nav_ready_exec'))
    for name, package, executable in requirements:
        try:
            path = resolve(package, executable)
            if not isinstance(path, str) or not Path(path).is_absolute():
                raise ValueError('resolver must return an absolute executable path')
            executables.append((name, path))
        except (ValueError, LookupError, OSError) as error:
            errors.append(f'{package}/{executable}: {error}')
    return UpPlan(request, bundle, graph, preflight, plan_platform(effective),
                  tuple(executables), tuple(errors),
                  plan_recording(effective, request.record, request.run_dir))


def report_plan(plan: UpPlan) -> Dict[str, Any]:
    """把预览序列化出来，且不泄露观察到的其它进程命令行.

    进程只以身份出现，命令行内容不进报告，避免把别人的参数写进运行记录。

    Args:
        plan: 已准备好的启动计划。

    Returns:
        JSON 兼容的报告，其中 dry-run 与运行期能力标志都显式给出。
    """
    options = plan.bundle.options
    return {
        'dry_run': True, 'runtime_enabled': True,
        'robot': plan.bundle.robot, 'site': plan.bundle.site, 'mode': options.mode,
        'run_dir': options.run_dir, 'record': plan.request.record,
        'teleop_overrides': yaml.safe_load(dict(plan.bundle.artifacts).get(
            'params/inputs/teleop_options.yaml', b'{}')),
        'recording': asdict(plan.recording),
        'launch': ['ros2', 'launch', 'nav_bringup', 'nav.launch.py',
                   'run_dir:=' + options.run_dir, 'robot:=' + plan.bundle.robot,
                   'site:=' + plan.bundle.site, 'mode:=' + options.mode],
        'start_chassis': options.start_chassis,
        'composition': options.composition,
        'processes': [asdict(process) for process in physical_graph(plan.graph).processes],
        'components': [dict(asdict(spec), plugin=plugin)
                       for spec, plugin in component_specs(plan.graph)],
        'executables': dict(plan.executables),
        'platform': [asdict(step) for step in plan.platform],
        'preflight': {
            'target_pids': [process.pid for process in plan.preflight.targets],
            'protected_pids': list(plan.preflight.protected_pids),
            'warnings': list(plan.preflight.warnings)},
        'dds': 'not inspected or cleaned by dry-run; incomplete evidence retains all segments',
        'artifacts': [name for name, _ in plan.bundle.artifacts],
        'warnings': [asdict(issue) for issue in plan.bundle.issues],
        'errors': list(plan.errors),
        'pending_runtime': ['navigation acceptance and operator deployment checks'],
    }
