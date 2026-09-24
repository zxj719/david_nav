"""在任何平台或 ROS 动作之前，离线渲染出一个已校验的启动包.

准备阶段只读配置并返回不可变字节；写入由单独的函数完成，且只往记录器自有的目录里
补齐缺失文件。launch.json 最后写，这样写盘中途失败不会留下一个"看起来可启动"的半成品。
两者都不做进程发现、不启动定位、不打开任何硬件；校验只覆盖配置本身，
不证明这台机器上装齐了对应的可执行文件。
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

# 附加叠加自报的身份，以及它在运行记录里被冻结成的资产名。
OVERLAY_NAME = 'oa'
OVERLAY_ARTIFACT = 'params/inputs/overlay.yaml'


@dataclass(frozen=True)
class BundleInputs:
    """配置源文件路径，与"源码布局还是安装布局"无关.

    Attributes:
        profile: 机器人档案 YAML。
        mode: 选定模式的差异 YAML。
        nav2_base: 共享的 Nav2 参数模板；底盘/遥控模式用不到。
        nav2_bindings: 档案到 Nav2 的共享绑定表。
        bt_templates: 行为树模板目录。
        bt_bindings: 行为树的档案绑定。
        site: 可选场地档案；导航模式必需。
        urdf: 机器人描述；导航校验必需。
        rviz: 可选的 RViz 配置，只有选了 RViz 才必需。
        overlay: 可选的附加叠加 YAML，合并到模式差异之上；只有显式选项
            （目前只有 --OA）才会选出它，没选时为 None 且不读任何文件。
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
    overlay: Optional[Path] = None


@dataclass(frozen=True)
class PreparedBundle:
    """已校验的字节内容，可直接加入某一条新的运行记录.

    Attributes:
        options: 启动图选择，含档案给出的进程与话题绑定。
        robot: 记录下来的机器人身份。
        site: 记录下来的场地身份；无场地的底盘工具为空。
        artifacts: 相对文件名与不可变文件内容的配对。
        issues: 离线校验发现的非致命警告；有警告仍可启动，需人工判断。
    """

    options: GraphOptions
    robot: str
    site: str
    artifacts: Tuple[Tuple[str, bytes], ...]
    issues: Tuple[validation.ValidationIssue, ...]


class _Reader:
    """只把配置完整快照下来，较大的只读资源改为流式取哈希.

    大文件只记哈希不记内容：能证明启动时用的是哪一份，不能据此重建该文件。
    """

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


def merge_overlay(
    mode: render.ModeOverlay, data: Any, source: Any,
) -> render.ModeOverlay:
    """把存在的附加叠加合并到模式叠加上.

    准备、规划与运行期绑定三处都要从同一批输入重建同一份有效档案，任何一处
    漏掉这次合并，绑定出来的命令就会和渲染出来的参数对不上。因此三处共用本函数，
    而不是各自合并一遍。

    Args:
        mode: 已解析的模式叠加。
        data: 已解析的附加叠加 YAML；本次没有选叠加时为 None。
        source: 出错消息里指明的来源，通常是路径。

    Returns:
        合并后的叠加；没有附加叠加时原样返回 mode。

    Raises:
        ValueError: 叠加自报的身份不是预期的那个。
        RenderError: 叠加本身的形状非法。
    """
    if data is None:
        return mode
    overlay = render.parse_mode(data, source)
    if overlay.name != OVERLAY_NAME:
        raise ValueError(f'{source}: expected the {OVERLAY_NAME} overlay, got {overlay.name!r}')
    return render.merge_overlays(mode, overlay)


def _identity(data: Mapping[str, Any], key: str) -> str:
    value = render.profile_value(data, key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'{key}: expected a nonempty identity')
    return value


def _version(data: Mapping[str, Any], label: str) -> None:
    if type(data.get('schema_version')) is not int or data['schema_version'] != 1:
        raise ValueError(f'{label}: requires schema_version: 1')


def _path(value: Any, parent: Path) -> Path:
    """展开资产路径中的用户目录，并把相对路径接到指定父目录.

    不 resolve、不检查存在性或安全归属，不去掉有效路径中的空白。
    与执行器绑定中的绝对路径解析助手不是同一种保证。

    Args:
        value: 非空资产路径字符串。
        parent: 解释相对路径时使用的父目录。

    Returns:
        展开后的绝对路径或拼接父目录后的路径。

    Raises:
        ValueError: 输入不是非空字符串。
    """
    if not isinstance(value, str) or not value.strip():
        raise ValueError('asset path must be a nonempty string')
    path = Path(value).expanduser()
    return path if path.is_absolute() else parent / path


def _yaml(data: Any) -> bytes:
    """以排序键与可读 Unicode 生成冻结配置的 UTF-8 YAML 字节.

    只序列化，不写文件；保持当前 YAML 库的既有输出格式。

    Args:
        data: safe_dump 支持的配置数据。

    Returns:
        排序键序列化后的 UTF-8 字节。

    Raises:
        yaml.YAMLError: 数据不能安全表示为 YAML。
    """
    return yaml.safe_dump(data, allow_unicode=True, sort_keys=True).encode('utf-8')


def _json(data: Any) -> bytes:
    """生成缩进两格、保留 Unicode 且禁止非有限数的配置 JSON 字节.

    保留输入对象的键顺序，末尾添加一个换行，不自行落盘。

    Args:
        data: 可由 JSON 序列化的配置数据。

    Returns:
        UTF-8 编码的带尾换行 JSON。

    Raises:
        ValueError: 含 NaN、无穷大或循环引用。
        TypeError: 含不支持的对象或键。
    """
    return (json.dumps(data, ensure_ascii=False, allow_nan=False, indent=2) + '\n').encode('utf-8')


def _profile_options(options: GraphOptions, profile: Mapping[str, Any]) -> GraphOptions:
    """把机器人档案绑定到启动选项副本，并用图构建器验证其组合.

    Args:
        options: 调用方选择的模式与启动开关。
        profile: 已叠加模式的有效档案；时间配置单位为秒。

    Returns:
        填好驱动、话题、适配器实现和预算的 GraphOptions。

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
        map_server=options.map_server)
    resolved = replace(resolved, smooth_command_topic=_identity(
        profile, 'chassis.adapter.input_topic'))
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
    """完全离线地读取、绑定并校验配置，过程中不写任何文件.

    底盘自动探测与执行器命令行参数由调用方先行解析。档案给出的话题、驱动身份与
    启动时序覆盖启动图的通用默认值。底盘与遥控模式从不要求地图、URDF 或定位。

    Args:
        options: 期望的启动图开关与最终的绝对运行目录。
        inputs: 显式的配置源路径。
        catalog: 离线校验所用的聚合包 schema。
        teleop_options: 显式的键盘覆盖项；不会改动源文件快照。

    Returns:
        不可变的文件内容与已解析选项，可直接用于 dry-run 审阅。

    Raises:
        ValueError: 配置格式错误、彼此不一致或缺项。
        OSError: 某个必需输入读不到。
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
    mode = merge_overlay(
        mode,
        reader.mapping('overlay', inputs.overlay, OVERLAY_ARTIFACT)
        if inputs.overlay is not None else None,
        inputs.overlay)
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
    """把已准备好的文件加入一条已存在且刚创建的运行记录.

    已存在的文件与指向别处的软链在写入之前就被拒绝，因此本函数不会覆盖任何东西。
    写盘失败可能留下用于诊断的半成品，但绝不会留下 launch.json。
    整个过程中运行目录与其单实例锁都由调用方持有。

    Args:
        bundle: 之前已校验、面向本次运行确切目录的内容。

    Raises:
        ValueError: 目标位置不安全，或内容格式错误。
        FileExistsError: 任一目标文件已存在，包括断掉的软链。
        OSError: 创建目录或排他写文件失败。
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
