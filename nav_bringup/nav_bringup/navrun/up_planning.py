"""Read-only startup planning, using the same bundle and graph as ROS launch.

The returned plan is a preview, not authority to mutate the host. Runtime wiring
remains explicit integration work. In particular,
successful dry-run validation never claims that hardware was exercised.
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
from nav_bringup.navrun.preparation import BundleInputs, PreparedBundle, prepare_bundle
from nav_bringup.navrun.recording import plan_recording, RecordingPlan
from nav_bringup.navrun.teleop_options import apply_options, TeleopOptions


@dataclass(frozen=True)
class UpRequest:
    """User selectors independent of ROS resource discovery.

    Attributes:
        mode: One of the five launch modes.
        robot: Robot name, provided by the corresponding name_config package.
        site: Site directory name within that package; ignored for chassis tools.
        run_dir: Prospective absolute run directory, not created during planning.
        chassis: auto, on or off; resolved by a read-only process snapshot.
        record: Requested light, full or off recording policy.
        no_executor: Disable WS transport; precise mode retains local Dock ingress.
        teleop_options: Explicit layout/gears, frozen separately from source files.
        mode_file: Optional complete mode overlay path; never changes the mode identity.
        composition: Explicitly opt in to one isolated Nav2 container.
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


@dataclass(frozen=True)
class UpPlan:
    """A validated bundle plus unresolved deployment requirements.

    Attributes:
        request: Original selectors.
        bundle: Immutable rendered configuration.
        graph: Logical nodes/programs selected from the rendered options.
        preflight: Chassis ownership, cleanup identities and protected processes.
        platform: Proposed noninteractive platform commands, not executed.
        executables: Physical process name and resolved installed executable path.
        errors: Missing dependencies or unfinished binding requirements.
        recording: Explicit run-local rosbag selection, not started by planning.
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
    if request.mode_file is not None and not isinstance(request.mode_file, Path):
        raise ValueError('mode_file must be a path')
    if (not isinstance(request.run_dir, Path) or not request.run_dir.is_absolute()
            or '..' in request.run_dir.parts or request.run_dir == Path('/')):
        raise ValueError('run_dir must be an absolute normalized run directory')


def discover_inputs(
    request: UpRequest, package_share: Callable[[str], Path],
) -> BundleInputs:
    """Select source or installed assets without importing robot package code.

    Args:
        request: User selectors, validated before any resource lookup.
        package_share: Read-only resolver returning a package's resource directory.

    Returns:
        Paths consumed by the shared offline bundle renderer. Non-navigation
        modes do not look up Nav2, plugin, site, map or URDF resources.

    Raises:
        ValueError: A selector is invalid.
        LookupError: A required resource package is unavailable.
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
        urdf=robot / 'urdf' / (request.robot + '.urdf') if navigation else None)


def plan_up(
    request: UpRequest, inputs: BundleInputs, catalog: schema.ParameterCatalog,
    processes: Sequence[ProcessInfo], own_pid: int,
    resolve: Callable[[str, str], str],
    resolve_plugin: Callable[[str, str], str] = resolve_component,
) -> UpPlan:
    """Prepare configuration and check every entry point before host mutations.

    Args:
        request: Requested startup selectors.
        inputs: Resolved configuration paths.
        catalog: Package-owned parameter schemas.
        processes: Complete read-only process snapshot.
        own_pid: Supervisor identity present in the snapshot.
        resolve: Read-only package/executable resolver, injected by tests.
        resolve_plugin: Read-only component resource/library resolver.

    Returns:
        Plan containing all missing executable errors, rather than stopping at
        the first missing component. No cleanup, locks, ROS or writes occur.

    Raises:
        ValueError: Invalid inputs, mismatched identity or changing configuration.
        OSError: A configuration file cannot be read.
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
            or artifacts['params/inputs/mode.yaml'] != initial_mode):
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
    """Serialize a preview without disclosing observed process command lines.

    Args:
        plan: Prepared startup plan.

    Returns:
        JSON-compatible report with explicit dry-run and runtime capability flags.
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
