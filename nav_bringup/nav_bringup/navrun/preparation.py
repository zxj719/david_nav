"""Render an offline, validated launch bundle before any platform or ROS action.

Preparation reads configuration and returns immutable bytes. The separate writer
only adds missing artifacts to a recorder-owned directory; launch.json is written
last so a failed write cannot publish a launchable, partially prepared bundle.
Neither function discovers processes, starts localization, or opens hardware.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import hashlib
import json
import os
from pathlib import Path
import tempfile
from typing import Any, Dict, Mapping, Optional, Tuple

import yaml

from nav_bringup.launching.graph import build_graph, GraphOptions, NAVIGATION_MODES
from nav_bringup.navcfg import behavior_trees, render, schema, validation
from nav_bringup.navrun import prepared_parameters
from nav_bringup.navrun.executor_binding import bind_executor
from nav_bringup.navrun.teleop_options import apply_options, TeleopOptions


@dataclass(frozen=True)
class BundleInputs:
    """Configuration source paths, independent of source versus installed layout.

    Attributes:
        profile: Robot profile YAML.
        mode: Selected mode overlay YAML.
        nav2_base: Shared Nav2 parameter template; unused in chassis/teleop modes.
        nav2_bindings: Shared profile-to-Nav2 bindings.
        bt_templates: Directory of behavior tree templates.
        bt_bindings: Behavior tree profile bindings.
        site: Optional site profile; required for navigation modes.
        urdf: Robot description; required for navigation validation.
        rviz: Optional RViz configuration, required only when RViz is selected.
    """

    profile: Path
    mode: Path
    nav2_base: Path
    nav2_bindings: Path
    bt_templates: Path
    bt_bindings: Path
    site: Optional[Path] = None
    urdf: Optional[Path] = None
    rviz: Optional[Path] = None


@dataclass(frozen=True)
class PreparedBundle:
    """Validated bytes ready to add to one new run record.

    Attributes:
        options: Graph choices with profile-owned process and topic bindings.
        robot: Recorded robot identity.
        site: Recorded site identity, or none for site-free chassis tools.
        artifacts: Relative file names paired with immutable file contents.
        issues: Nonfatal offline validation warnings.
    """

    options: GraphOptions
    robot: str
    site: str
    artifacts: Tuple[Tuple[str, bytes], ...]
    issues: Tuple[validation.ValidationIssue, ...]


class _Reader:
    """Snapshot only configuration, and stream hashes of larger read-only assets."""

    def __init__(self) -> None:
        self.artifacts: Dict[str, bytes] = {}
        self.fingerprints: Dict[str, Dict[str, str]] = {}

    def read(self, label: str, path: Path, destination: str) -> bytes:
        path = path.expanduser().resolve(strict=True)
        data = path.read_bytes()
        self.artifacts[destination] = data
        self.fingerprints[label] = {
            'path': str(path), 'sha256': hashlib.sha256(data).hexdigest()}
        return data

    def mapping(self, label: str, path: Path, destination: str) -> Dict[str, Any]:
        data = yaml.safe_load(self.read(label, path, destination))
        if not isinstance(data, dict):
            raise ValueError(f'{label}: expected a YAML mapping')
        return data

    def fingerprint(self, label: str, path: Path) -> None:
        path = path.expanduser().resolve(strict=True)
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        self.fingerprints[label] = {'path': str(path), 'sha256': digest.hexdigest()}


def _identity(data: Mapping[str, Any], key: str) -> str:
    value = render.profile_value(data, key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'{key}: expected a nonempty identity')
    return value


def _version(data: Mapping[str, Any], label: str) -> None:
    if type(data.get('schema_version')) is not int or data['schema_version'] != 1:
        raise ValueError(f'{label}: requires schema_version: 1')


def _path(value: Any, parent: Path) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ValueError('asset path must be a nonempty string')
    path = Path(value).expanduser()
    return path if path.is_absolute() else parent / path


def _yaml(data: Any) -> bytes:
    return yaml.safe_dump(data, allow_unicode=True, sort_keys=True).encode('utf-8')


def _json(data: Any) -> bytes:
    return (json.dumps(data, ensure_ascii=False, allow_nan=False, indent=2) + '\n').encode('utf-8')


def _profile_options(options: GraphOptions, profile: Mapping[str, Any]) -> GraphOptions:
    """把机器人档案绑定到启动选项副本，并用图构建器验证其组合.

    Args:
        options: 调用方选择的模式与启动开关。
        profile: 已叠加模式的有效档案；时间配置单位为秒。

    Returns:
        填好驱动、话题、适配器实现和预算的 GraphOptions；
        bench_slamgt 保留不自建 map_server 的策略。

    Raises:
        ValueError: 驱动配置、标识、图选项或 teleop 输入话题一致性不合法。
    """
    driver = render.profile_value(profile, 'chassis.driver')
    if not isinstance(driver, dict):
        raise ValueError('chassis.driver must be a mapping')
    resolved = replace(
        options,
        driver_package=_identity(profile, 'chassis.driver.package'),
        driver_executable=_identity(profile, 'chassis.driver.executable'),
        driver_restarts=driver.get('max_restarts', 1),
        ready_timeout_sec=profile.get('startup', {}).get(
            'nav_ready_timeout_sec', options.ready_timeout_sec),
        chassis_delay_sec=driver.get('startup_delay_sec', 0.0),
        chassis_command_topic=_identity(profile, 'chassis.cmd_topic'),
        chassis_odom_topic=_identity(profile, 'chassis.odom_topic'),
        adapter_implementation=profile.get('chassis', {}).get(
            'adapter', {}).get('implementation', 'python'),
        velocity_smoother=profile.get('nav2', {}).get(
            'velocity_smoother', {}).get('enabled', True),
        map_server=options.map_server and options.mode != 'bench_slamgt')
    resolved = replace(resolved, smooth_command_topic=_identity(
        profile, 'chassis.adapter.input_topic'))
    if options.scan_bridge:
        resolved = replace(
            resolved, cloud_topic=_identity(profile, 'sensors.scan_from_cloud.cloud_topic'),
            scan_topic=_identity(profile, 'sensors.scan_from_cloud.scan_topic'))
    if options.mode == 'teleop' and resolved.smooth_command_topic.lstrip('/') != (
            _identity(profile, 'teleop.cmd_topic').lstrip('/')):
        raise ValueError('teleop.cmd_topic must match the teleop adapter input_topic')
    build_graph(resolved)
    return resolved


def _navigation(
    inputs: BundleInputs, reader: _Reader, profile: Mapping[str, Any],
    mode: render.ModeOverlay, options: GraphOptions, site: Mapping[str, Any],
    catalog: schema.ParameterCatalog,
) -> Tuple[Dict[str, Any], Tuple[validation.ValidationIssue, ...]]:
    """只读采集导航输入并渲染参数/行为树，验证后登记内存资产.

    Args:
        inputs: 地图、URDF、基础参数和模板等资源路径。
        reader: 持有输入快照、指纹与待写资产的读取器。
        profile: 源机器人档案。
        mode: 已解析模式叠加。
        options: 含运行目录的启动选项，供绑定输出路径。
        site: 场地档案，named_poses 仅采指纹。
        catalog: 完整参数 schema 目录。

    Returns:
        渲染 Nav2 参数与 URDF 的映射，以及配置诊断元组；没有写盘或启动 ROS。

    Raises:
        ValueError: 缺输入/模板、渲染或一致性验证失败。
        OSError: 只读输入或指纹采集失败。
    """
    if inputs.site is None or inputs.urdf is None:
        raise ValueError('navigation preparation requires site and urdf')
    effective = render.deep_merge(profile, mode.profile)
    map_path = _path(render.profile_value(site, 'map.nav_yaml'), inputs.site.resolve().parent)
    map_data = reader.mapping('map', map_path, 'params/inputs/map.yaml')
    image_path = _path(map_data.get('image'), map_path.resolve().parent)
    reader.fingerprint('map_image', image_path)
    urdf = reader.read('urdf', inputs.urdf, 'params/inputs/robot.urdf').decode('utf-8')
    base = reader.mapping('nav2_base', inputs.nav2_base, 'params/inputs/nav2_base.yaml')
    bindings = reader.mapping(
        'nav2_bindings', inputs.nav2_bindings, 'params/inputs/nav2_bindings.yaml')
    tree = render.render(
        base, render.parse_bindings(bindings, inputs.nav2_bindings), profile, mode)
    bt_bindings = behavior_trees.parse_bindings(reader.mapping(
        'bt_bindings', inputs.bt_bindings, 'params/inputs/bt_bindings.yaml'))
    templates = sorted(inputs.bt_templates.glob('*.xml.tmpl'))
    if not templates:
        raise ValueError('no behavior tree templates found')
    trees = {}
    for path in templates:
        text = reader.read('bt:' + path.name, path, 'params/inputs/bt/' + path.name)
        trees[path.name.removesuffix('.tmpl')] = behavior_trees.render_template(
            text.decode('utf-8'), effective, bt_bindings)
    issues = tuple(validation.validate_configuration(
        catalog, profile, mode, tree, map_data, trees, urdf))
    errors = [issue.message for issue in issues if issue.severity == 'error']
    if errors:
        raise ValueError('configuration rejected: ' + '; '.join(errors))
    for name, text in trees.items():
        reader.artifacts['params/rendered/bt/' + name] = text.encode('utf-8')
    behavior_trees.bind_output_paths(tree, trees, Path(options.run_dir) / 'params/rendered/bt')
    reader.artifacts['params/rendered/map.yaml'] = _yaml(
        dict(map_data, image=str(image_path.resolve())))
    named_poses = site.get('pois', {}).get('named_poses_file')
    if named_poses:
        reader.fingerprint('named_poses', _path(named_poses, inputs.site.resolve().parent))
    return {'nav2': tree, 'robot_description': urdf}, issues


def prepare_bundle(
    options: GraphOptions, inputs: BundleInputs, catalog: schema.ParameterCatalog,
    teleop_options: TeleopOptions = TeleopOptions(),
) -> PreparedBundle:
    """Read, bind and validate configuration entirely offline and without writes.

    The caller resolves chassis auto-detection and executor CLI arguments first.
    Profile-owned topics, driver identity and startup timing override generic
    graph defaults. Chassis and teleop never require map, URDF or localization.

    Args:
        options: Desired graph switches and final absolute run directory.
        inputs: Explicit configuration source paths.
        catalog: Aggregated package schemas used by offline validation.
        teleop_options: Explicit keyboard overrides; never modify source snapshots.

    Returns:
        Immutable file contents and resolved options, suitable for dry-run inspection.

    Raises:
        ValueError: If configuration is malformed, inconsistent or incomplete.
        OSError: If a required input cannot be read.
    """
    build_graph(options)
    overrides = teleop_options.values(options.mode)
    reader = _Reader()
    profile = reader.mapping('robot', inputs.profile, 'params/inputs/robot.yaml')
    _version(profile, 'robot')
    mode = render.parse_mode(
        reader.mapping('mode', inputs.mode, 'params/inputs/mode.yaml'), inputs.mode)
    if mode.name != options.mode:
        raise ValueError('mode overlay does not match the requested launch mode')
    effective = apply_options(render.deep_merge(profile, mode.profile),
                              options.mode, teleop_options)
    if overrides:
        reader.artifacts['params/inputs/teleop_options.yaml'] = _yaml(overrides)
    options = _profile_options(options, effective)
    site = (reader.mapping('site', inputs.site, 'params/inputs/site.yaml')
            if inputs.site is not None else {})
    if inputs.site is not None:
        _version(site, 'site')
        options = bind_executor(options, effective, site, inputs.site.resolve().parent)
    robot_name = _identity(effective, 'robot.name')
    site_name = _identity(site, 'site.name') if site else 'none'
    graph = build_graph(options)
    navigation: Dict[str, Any] = {}
    if options.mode in NAVIGATION_MODES:
        navigation, issues = _navigation(inputs, reader, profile, mode, options, site, catalog)
    else:
        issues = tuple(validation.ValidationIssue('schema', issue.severity, issue.message)
                       for issue in catalog.check_profile(effective))
        if options.mode == 'teleop':
            issues += tuple(validation.validate_teleop_limits(effective))
        errors = [issue.message for issue in issues if issue.severity == 'error']
        if errors:
            raise ValueError('configuration rejected: ' + '; '.join(errors))
    parameters = prepared_parameters.node_parameters(options, effective, navigation)
    for process in graph.processes:
        files = process.configuration_files
        if process.parameter_file:
            files += (process.parameter_file,)
        for path in dict.fromkeys(files):
            name = Path(path).stem
            reader.artifacts['params/rendered/' + name + '.yaml'] = _yaml(parameters[name])
    if options.rviz:
        if inputs.rviz is None:
            raise ValueError('rviz selected without a configuration file')
        reader.read('rviz', inputs.rviz, 'inputs/navigation.rviz')
    reader.artifacts['checks.yaml'] = _yaml({'offline': [asdict(issue) for issue in issues]})
    reader.artifacts['params/inputs/fingerprints.json'] = _json(reader.fingerprints)
    values = asdict(options)
    values.pop('run_dir')
    reader.artifacts['launch.json'] = _json({
        'schema_version': 1, 'robot': robot_name, 'site': site_name, 'options': values})
    return PreparedBundle(options, robot_name, site_name,
                          tuple(sorted(reader.artifacts.items())), issues)


def write_bundle(bundle: PreparedBundle) -> None:
    """Add prepared artifacts to an existing, newly created run record.

    Existing files and symlinked destinations are rejected before any write.
    A disk failure may leave diagnostic partial artifacts, but never launch.json.
    The caller owns the run directory and its single-instance lock throughout.

    Args:
        bundle: Previously validated contents for this run's exact directory.

    Raises:
        ValueError: If a destination is unsafe or artifacts are malformed.
        FileExistsError: If any artifact already exists, including a broken symlink.
        OSError: If a directory or exclusive file write fails.
    """
    build_graph(bundle.options)
    root = Path(bundle.options.run_dir)
    if root.is_symlink() or not root.is_dir():
        raise ValueError('run directory must exist and must not be a symlink')
    names = [name for name, _ in bundle.artifacts]
    if len(set(names)) != len(names) or names.count('launch.json') != 1:
        raise ValueError('bundle requires unique artifact paths and one launch.json')
    for name in names:
        relative = Path(name)
        if relative.is_absolute() or '..' in relative.parts or str(relative) != name:
            raise ValueError('artifact paths must be normalized and relative')
        path = root / relative
        for parent in path.parents:
            if parent == root:
                break
            if parent.is_symlink():
                raise ValueError(f'symlink in bundle destination: {parent}')
        if path.exists() or path.is_symlink():
            raise FileExistsError(path)
    ordered = sorted(bundle.artifacts, key=lambda entry: (entry[0] == 'launch.json', entry[0]))
    for name, content in ordered:
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        if name == 'launch.json':
            _publish_marker(path, content)
            continue
        with path.open('xb') as stream:
            stream.write(content)


def _publish_marker(path: Path, content: bytes) -> None:
    # A hard link publishes complete bytes exclusively: unlike replace/rename,
    # it cannot clobber a marker created since the preflight check.
    """通过同目录临时文件和排他硬链接发布完整标记，不覆盖已有目标.

    先 flush 和 fsync 临时文件，再链接到目标；并发创建的目标使发布失败，
    上下文负责删除临时文件。不宣称目录项已经具备断电持久性。

    Args:
        path: 已有父目录下待首次发布的标记路径。
        content: 已生成的完整标记字节。

    Raises:
        FileExistsError: 目标已存在或在预检查后被并发创建。
        OSError: 临时文件写入、同步、链接或回收失败。
    """
    with tempfile.NamedTemporaryFile(prefix='.launch-', dir=path.parent) as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())
        os.link(stream.name, path)
