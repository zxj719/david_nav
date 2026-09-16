"""Prepare a private Nav2 experiment without reading operator maps or starting ROS.

Robot geometry and selected-mode tuning remain source configuration. Only
simulation wiring, private site assets and process ownership are overridden.
Preparation is not execution or a successful closed-loop test.
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
    """Prepared private experiment, explicitly not an execution verdict.

    Attributes:
        root: Exclusive experiment directory containing all writable assets.
        namespace: Unique ROS namespace applied to every process.
        graph: Real Nav2 and task manager plus fake chassis/localization.
        cases: Ordinary POI followed by two-stage Dock acceptance expectations.
        digest: Hash of the complete plan passed explicitly to its owned launcher.
    """

    root: Path
    namespace: str
    graph: LaunchGraph
    cases: Tuple[ExpectedCase, ExpectedCase]
    digest: str


def _yaml(value):
    return yaml.safe_dump(value, allow_unicode=True, sort_keys=True).encode()


def _json(value):
    return (json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + '\n').encode()


def _write(root, name, value):
    path = root / name
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('xb') as stream:
        stream.write(value)


def _validate(root, robot, mode, namespace, uri):
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


def _assets(root, profile, namespace, uri):
    frame = profile['frames']['map']
    cases = tuple(ExpectedCase(
        namespace + '_' + kind, index, 'selftest_' + kind, kind, (float(index), 0.0, 0.0),
        xy_ok=profile['goal']['xy_tolerance'], yaw_ok=profile['goal']['yaw_tolerance'])
        for index, kind in enumerate(('poi', 'dock'), 1))
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
              'dock_poi_ids': [cases[1].target_id]}, agent_server={'uri': uri})))
    return cases


def _private_graph(bundle, profile, namespace, cases):
    root, prefix = Path(bundle.options.run_dir), '/' + namespace + '/'
    artifacts = dict(bundle.artifacts)
    # Both modes exercise Dock. Reuse the existing precise-mode Dock policy,
    # but retain the actually selected tour/precise Nav2 tuning and tolerances.
    navigation = {'nav2': yaml.safe_load(artifacts['params/rendered/nav2.yaml']),
                  'robot_description': artifacts['params/inputs/robot.urdf'].decode()}
    parameters = node_parameters(replace(bundle.options, mode='precise'), profile, navigation)
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
            args = process.arguments
            if bundle.options.mode == 'tour':
                args += ('--dock-config-file', str(root / 'params/rendered/dock.yaml'))
            args += ('--ros-args', '-r', '__ns:=/' + namespace)
            for source, target in remaps:
                args += ('-r', source + ':=' + target)
            process = replace(process, arguments=args,
                              configuration_files=tuple(dict.fromkeys((
                                  *process.configuration_files,
                                  str(root / 'params/rendered/dock.yaml')))))
        private.append(process)
    return replace(bundle, artifacts=tuple(sorted(artifacts.items()))), replace(
        graph, processes=tuple(private))


def prepare_scene(
    root: Path, robot: str, mode: str, namespace: str, inputs: BundleInputs,
    catalog: schema.ParameterCatalog, server_uri: str,
) -> Scene:
    """Create exclusive private assets and a real-Nav2 launch plan, entirely offline.

    Source robot, mode, URDF, shared templates and schemas are read-only. The
    supplied operator site is deliberately not opened. Invalid rendering can
    leave partial private diagnostics, but never a runnable selftest marker.

    Args:
        root: New absolute output directory under a trusted existing parent.
        robot: Selected robot identity, matching its source profile.
        mode: Tour or precise; no controller tuning is replaced by this fixture.
        namespace: Unique selftest_ followed by 32 lowercase UUID hex digits.
        inputs: Shared renderer inputs; site is replaced with private scene data.
        catalog: Available parameter schemas for the normal offline validator.
        server_uri: Owned loopback peer, or a nonexecuted preview URI.

    Returns:
        Private graph and expected cases; no claim of execution or success.

    Raises:
        ValueError: Identity, isolation, geometry or normal configuration is invalid.
        FileExistsError: Output already exists; nothing is overwritten.
        OSError: An input read or exclusive private write fails.
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
    cases = _assets(root, profile, namespace, server_uri)
    arguments = (
        '--server-uri', server_uri, '--reconnect-delay', '0.1',
        '--pose-file', str(root / 'scene/poses.json'),
        '--poi-store-file', str(root / 'poi_store.yaml'),
        '--global-costmap-topic', prefix + 'global_costmap/costmap',
        '--dock-poi-id', cases[1].target_id, '--dock-action-name', prefix + 'dock_to_pose')
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
