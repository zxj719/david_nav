"""离线准备私有 Nav2 场景，不读取操作员地图或启动 ROS.

沿用源机器人几何及选定模式调参，只替换仿真接线、私有场地资产和进程策略。
生成场景不等于已执行，也不证明闭环测试通过。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Tuple
from urllib.parse import urlsplit

import yaml

from nav_bringup.launching.graph import build_graph, GraphOptions, LaunchGraph, ProcessSpec
from nav_bringup.navcfg import render, schema
from nav_bringup.navrun.preparation import BundleInputs, prepare_bundle, write_bundle
from nav_bringup.navrun.prepared_parameters import node_parameters
from nav_sim.selftest_evidence import ExpectedCase


@dataclass(frozen=True)
class Scene:
    """保存准备好的私有实验描述，明确不作为执行结论.

    Attributes:
        root: 排他创建的实验目录，用于本次可写资产。
        namespace: 本图 ROS 参与者使用的唯一命名空间，非 ROS 包装入口另以参数接线。
        graph: 真实 Nav2、任务管理器与假底盘/假定位组成的图。
        cases: 普通 POI 后两段 Dock 的验收预期。
        digest: 显式传给自有启动器的完整计划 SHA256。
    """

    root: Path
    namespace: str
    graph: LaunchGraph
    cases: Tuple[ExpectedCase, ExpectedCase]
    digest: str


def _yaml(value):
    """生成自测场景的 UTF-8 YAML 字节，排序键且保留 Unicode.

    只序列化，不创建目录或写文件，沿用当前 YAML 库的输出格式。

    Args:
        value: safe_dump 支持的场景数据。

    Returns:
        序列化后的 UTF-8 字节。

    Raises:
        yaml.YAMLError: 数据不能安全表示为 YAML。
    """
    return yaml.safe_dump(value, allow_unicode=True, sort_keys=True).encode()


def _json(value):
    """序列化自测场景 JSON，拒绝非有限数并保留输入键顺序.

    输出缩进为两格，保留 Unicode，末尾加一个换行；不在此落盘。

    Args:
        value: 可序列化的场景数据。

    Returns:
        UTF-8 JSON 字节。

    Raises:
        ValueError: 含非有限浮点值或循环引用。
        TypeError: 数据包含不支持的对象或键。
    """
    return (json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + '\n').encode()


def _write(root, name, value):
    """创建父目录并排他写入一个场景制品，已有文件不覆盖.

    根与相对名称必须由场景构建器选定；本函数不校验路径逃逸或祖先链接。
    写入失败可能留下部分文件，不在此回滚场景目录。

    Args:
        root: 调用方拥有的私有场景根目录。
        name: 构建器给定的制品相对路径。
        value: 要写入的完整字节内容。

    Raises:
        FileExistsError: 目标已存在。
        OSError: 目录创建、文件打开或写入失败。
    """
    path = root / name
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('xb') as stream:
        stream.write(value)


def _validate(root, robot, mode, namespace, uri):
    """在创建实验目录前校验身份、模式和无凭据的本地 WS 地址.

    Args:
        root: 尚不存在的绝对规范输出目录，不允许根目录或符号链接。
        robot: 机器人资源名称。
        mode: tour 或 precise。
        namespace: selftest_ 后接 32 位小写 UUID 十六进制的命名空间。
        uri: 仅允许带有效端口的 ws://127.0.0.1 地址，无路径或查询参数。

    Raises:
        ValueError: 身份、模式、路径或 URI 不符合私有实验约束。
        FileExistsError: 输出已存在，拒绝覆盖。
    """
    if mode not in ('tour', 'precise'):
        raise ValueError('selftest mode must be tour or precise')
    if re.fullmatch(r'[a-z][a-z0-9_]*', robot) is None:
        raise ValueError('robot must be a resource name')
    if re.fullmatch(r'selftest_[0-9a-f]{32}', namespace) is None:
        raise ValueError('selftest requires a unique selftest_<UUID hex> namespace')
    if not root.is_absolute() or '..' in root.parts or root == Path('/'):
        raise ValueError('output must be a new absolute normalized directory')
    if root.exists() or root.is_symlink():
        raise FileExistsError(root)
    parsed = urlsplit(uri)
    if (parsed.scheme != 'ws' or parsed.hostname != '127.0.0.1'
            or parsed.username or parsed.password or not parsed.port
            or parsed.path or parsed.query or parsed.fragment):
        raise ValueError('agent URI must be ws://127.0.0.1:<positive port>')


def _profile(original, mode, prefix, root):
    """保留机器人几何和模式调参，仅替换为私有仿真接线及无硬件策略.

    Args:
        original: 只读源机器人档案，深合并后修改副本。
        mode: 已解析的模式叠加对象。
        prefix: 本次实验的绝对 ROS 命名空间话题前缀。
        root: 私有输出目录，POI 可写缓存定位于其下。

    Returns:
        使用假底盘/假定位、禁用点云转扫描及平台提频/杀进程操作的新档案。

    Raises:
        ValueError: footprint 坐标不是有限数值或超出实验房间几何范围。
        KeyError: 源档案缺少所需的配置节。
    """
    profile = render.deep_merge(original, mode.profile)
    polygon = profile.get('footprint', [])
    if (not isinstance(polygon, list) or len(polygon) < 3
            or any(not isinstance(point, list) or len(point) != 2
                   or any(type(value) not in (int, float) or not math.isfinite(value)
                          or abs(value) >= 2 for value in point) for point in polygon)):
        raise ValueError('selftest room requires a finite footprint within two metres')
    profile['chassis'].update(cmd_topic=prefix + 'cmd_vel', odom_topic=prefix + 'odom',
                              odom_frame=profile['frames']['odom'])
    profile['chassis']['adapter']['input_topic'] = prefix + 'cmd_vel_smoothed'
    profile['chassis']['driver'] = {
        'package': 'nav_sim', 'executable': 'fake_chassis', 'parameters': {},
        'detect_patterns': [], 'startup_delay_sec': 0.0, 'max_restarts': 0}
    profile['localization'] = {'provider': 'fake', 'odom_topic': prefix + 'odom',
                               'aligned_cloud_topic': prefix + 'cloud'}
    profile['nav2'].update(odom_topic=prefix + 'odom', map_topic=prefix + 'map',
                           scan_topic=prefix + 'scan')
    profile['executor'].update(grid_map_topic=prefix + 'map',
                               poi_store_file=str(root / 'poi_store.yaml'))
    profile['sensors']['scan_from_cloud'].update(
        enabled=False, cloud_topic=prefix + 'cloud', scan_topic=prefix + 'scan')
    profile['platform'] = {'jetson': False, 'maxn': False, 'jetson_clocks': False}
    profile['preflight'] = {'kill_patterns': [], 'kill_foreign_stacks': False,
                            'kill_stale_robot_state_publishers': False}
    return profile


def _assets(root, profile, namespace, uri, kinds):
    """独占创建各用例点位、合成静态房间地图和私有场地档案.

    Args:
        root: 已创建的本次实验目录，目标资产不应存在。
        profile: 选定模式下的机器人档案，提供地图帧及米/弧度容差。
        namespace: 用于唯一请求 ID 的实验命名空间。
        uri: 已验证的本地 WS 对端地址，仅写入私有配置。
        kinds: 本次模式要跑的用例类别序列，首个必须是普通 POI。

    Returns:
        按 kinds 顺序的用例元组；地图为 12 米方形、0.05 米分辨率，
        边界占用两格。此步骤没有发送目标或启动仿真。

    Raises:
        OSError: 创建资产失败；可能保留部分文件供诊断，不覆盖现有文件。
        KeyError: 档案缺少所需帧、目标容差或入位阈值。
    """
    frame = profile['frames']['map']
    # 两类用例的达标口径不同：普通 POI 用 Nav2 的到点容差，Dock 用入位自己的阈值。
    # 两者自 2026-09-18 起就是彼此独立的档案项，裁判若混用会把已达标的入位判成失败。
    tolerances = {'poi': (profile['goal']['xy_tolerance'], profile['goal']['yaw_tolerance']),
                  'dock': (profile['dock']['xy_ok'], profile['dock']['yaw_ok'])}
    cases = tuple(ExpectedCase(
        namespace + '_' + kind, index, 'selftest_' + kind, kind, (float(index), 0.0, 0.0),
        xy_ok=tolerances[kind][0], yaw_ok=tolerances[kind][1])
        for index, kind in enumerate(kinds, 1))
    poses = [dict(id=case.target_id, name=case.target_id, frame_id=frame,
                  position=dict(x=case.goal[0], y=0.0, z=0.0), yaw=0.0) for case in cases]
    _write(root, 'scene/poses.json', _json(poses))
    # Twelve-metre square room, five-centimetre cells, two-cell occupied border.
    pixels = bytes(0 if min(x, y, 239 - x, 239 - y) < 2 else 255
                   for y in range(240) for x in range(240))
    _write(root, 'scene/map.pgm', b'P5\n240 240\n255\n' + pixels)
    _write(root, 'scene/map.yaml', _yaml(dict(
        image='map.pgm', mode='trinary', resolution=0.05, origin=[-6.0, -6.0, 0.0],
        negate=0, occupied_thresh=0.65, free_thresh=0.196)))
    _write(root, 'scene/site.yaml', _yaml(dict(
        schema_version=1, site={'name': 'selftest'}, map={'nav_yaml': 'map.yaml'},
        pois={'named_poses_file': 'poses.json', 'store_file': str(root / 'poi_store.yaml'),
              'dock_poi_ids': [case.target_id for case in cases if case.kind == 'dock']},
        agent_server={'uri': uri})))
    return cases


def _private_graph(bundle, profile, namespace, cases):
    """扩展准备包为私有用例图，保留所选模式调参并按用例接入 Dock.

    Args:
        bundle: 已准备的普通启动包，原对象不修改。
        profile: 私有仿真档案，明确区分机身和导航基准帧。
        namespace: 所有 ROS 参与者共用的独占实验命名空间。
        cases: 预期用例参数；当前实现未读取，身份由其它准备步骤绑定。

    Returns:
        新准备包与启动图；附加假机器人参数、TF/事件重映射及执行器 Dock
        配置。普通 ROS 节点和非 ROS 包装入口分别注入命名空间，不启动进程。

    Raises:
        ValueError: 现有配置路径不属于实验根目录或图构建校验失败。
        KeyError: 准备包缺少所需资产、帧或 Dock 参数。
        yaml.YAMLError: 已准备参数无法解析。
    """
    root, prefix = Path(bundle.options.run_dir), '/' + namespace + '/'
    artifacts = dict(bundle.artifacts)
    # 生产只在 precise 模式配置入位（见 prepared_parameters 的模式判断），导览模式没有
    # 入位节点。自测跟随该事实：只有带 Dock 用例的模式才渲染并注入 Dock 配置，
    # 否则会去验证一个现场从不存在的组合（2026-09-20 导览自测失败即源于此）。
    docking = any(case.kind == 'dock' for case in cases)
    if docking:
        # 沿用 precise 的 Dock 策略，但保留本模式实际选中的 Nav2 调参与容差。
        navigation = {'nav2': yaml.safe_load(artifacts['params/rendered/nav2.yaml']),
                      'robot_description': artifacts['params/inputs/robot.urdf'].decode()}
        parameters = node_parameters(
            replace(bundle.options, mode='precise'), profile, navigation)
        dock = parameters['dock']
        dock['dock_navigator']['ros__parameters'].update(
            goal_topic=prefix + 'dock_goal', action_name=prefix + 'dock_to_pose')
        artifacts['params/rendered/dock.yaml'] = _yaml(dock)
    graph = build_graph(bundle.options)
    frames = profile['frames']
    fake = {
        'fake_chassis': dict(odom_frame=frames['odom'], body_frame=frames['body'], dt=0.02,
                             odom_topic=prefix + 'odom', cmd_topic=prefix + 'cmd_vel'),
        'fake_localization': dict(map_frame=frames['map'], odom_frame=frames['odom'],
                                  body_frame=frames['body'], odom_topic=prefix + 'odom'),
    }
    processes = list(graph.processes)
    for name, values in fake.items():
        path = 'params/rendered/' + name + '.yaml'
        artifacts[path] = _yaml({name: {'ros__parameters': values}})
        processes.append(ProcessSpec(name, 'nav_sim', name, str(root / path)))
    remaps = (('/tf', prefix + 'tf'), ('/tf_static', prefix + 'tf_static'),
              ('/nav/events', prefix + 'nav/events'))
    private = []
    for process in processes:
        if process.ros_node:
            old_path = Path(process.parameter_file).relative_to(root).as_posix()
            path = 'params/selftest/' + Path(old_path).name
            artifacts[path] = _yaml({namespace: yaml.safe_load(artifacts[old_path])})
            mapping = dict(process.remappings)
            mapping.update(remaps)
            process = replace(process, parameter_file=str(root / path),
                              remappings=tuple(mapping.items()))
        else:
            args = process.arguments + ('--ros-args', '-r', '__ns:=/' + namespace)
            for source, target in remaps:
                args += ('-r', source + ':=' + target)
            files = process.configuration_files
            if docking:
                files = tuple(dict.fromkeys((
                    *files, str(root / 'params/rendered/dock.yaml'))))
            process = replace(process, arguments=args, configuration_files=files)
        private.append(process)
    return replace(bundle, artifacts=tuple(sorted(artifacts.items()))), replace(
        graph, processes=tuple(private))


def prepare_scene(
    root: Path, robot: str, mode: str, namespace: str, inputs: BundleInputs,
    catalog: schema.ParameterCatalog, server_uri: str,
) -> Scene:
    """完全离线创建排他私有资产和真实 Nav2 启动计划.

    源机器人、模式、URDF、共享模板及 schema 只读，不打开输入中的操作员场地。
    渲染失败可留下私有诊断文件，但不会执行最后的完整计划标记写入；
    标记写入本身失败仍可能留下部分文件，不把文件存在当作计划已验证。

    Args:
        root: 可信已有父目录下的新绝对输出目录。
        robot: 与源档案一致的机器人身份。
        mode: tour 或 precise；保留选定模式的控制器调参。
        namespace: selftest_ 后接 32 位小写 UUID 十六进制的唯一命名空间。
        inputs: 共享渲染输入，场地替换为私有场景。
        catalog: 常规离线校验使用的参数 schema 集合。
        server_uri: 自有回环对端地址，或不执行的预览地址。

    Returns:
        私有图及预期用例；不声称已经执行或通过。

    Raises:
        ValueError: 身份、隔离、几何或常规配置不合法。
        FileExistsError: 输出已存在，拒绝覆盖。
        OSError: 输入读取或排他私有写入失败。
    """
    _validate(root, robot, mode, namespace, server_uri)
    original_bytes, mode_bytes = inputs.profile.read_bytes(), inputs.mode.read_bytes()
    original = yaml.safe_load(original_bytes)
    overlay = render.parse_mode(yaml.safe_load(mode_bytes), inputs.mode)
    if (not isinstance(original, dict) or original.get('robot', {}).get('name') != robot
            or overlay.name != mode):
        raise ValueError('selected robot/mode does not match source identity')
    prefix = '/' + namespace + '/'
    profile = _profile(original, overlay, prefix, root)
    root.mkdir(mode=0o700)
    _write(root, 'scene/source_robot.yaml', original_bytes)
    _write(root, 'scene/source_mode.yaml', mode_bytes)
    _write(root, 'scene/robot.yaml', _yaml(profile))
    _write(root, 'scene/mode.yaml', _yaml(dict(
        schema_version=1, mode=mode, profile={}, nav2_overrides=overlay.nav2_overrides)))
    cases = _assets(root, profile, namespace, server_uri,
                    ('poi', 'dock') if mode == 'precise' else ('poi',))
    arguments = (
        '--server-uri', server_uri, '--reconnect-delay', '0.1',
        '--pose-file', str(root / 'scene/poses.json'),
        '--poi-store-file', str(root / 'poi_store.yaml'),
        '--global-costmap-topic', prefix + 'global_costmap/costmap')
    if len(cases) > 1:
        arguments += ('--dock-poi-id', cases[1].target_id,
                      '--dock-action-name', prefix + 'dock_to_pose')
    options = GraphOptions(
        mode, str(root), start_chassis=False, start_executor=True,
        nav_command_topic=prefix + 'cmd_vel_nav', ready_action=prefix + 'navigate_to_pose',
        executor_arguments=arguments)
    private_inputs = replace(inputs, profile=root / 'scene/robot.yaml',
                             mode=root / 'scene/mode.yaml', site=root / 'scene/site.yaml')
    bundle = prepare_bundle(options, private_inputs, catalog)
    bundle, graph = _private_graph(bundle, profile, namespace, cases)
    write_bundle(bundle)
    assets = {path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
              for path in sorted(root.rglob('*')) if path.is_file()}
    marker = _json(dict(
        schema_version=1, backend='real_nav2', executed=False, robot=robot, mode=mode,
        namespace=namespace, cases=[asdict(case) for case in cases], graph=asdict(graph),
        asset_hashes=assets,
        source_hashes={'robot': hashlib.sha256(original_bytes).hexdigest(),
                       'mode': hashlib.sha256(mode_bytes).hexdigest()}))
    _write(root, 'selftest.json', marker)
    return Scene(root, namespace, graph, cases, hashlib.sha256(marker).hexdigest())
