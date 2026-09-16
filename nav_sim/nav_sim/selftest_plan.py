"""Verify a prepared experiment and its assets before importing any ROS runtime."""

import hashlib
import json
from pathlib import Path
import re

from nav_bringup.launching.graph import LaunchGraph, ProcessSpec
from nav_sim.selftest_evidence import check_evidence, ExpectedCase
from nav_sim.selftest_scene import Scene


_PROGRAMS = {
    'controller_server': ('nav2_controller', ('controller_server',)),
    'planner_server': ('nav2_planner', ('planner_server',)),
    'behavior_server': ('nav2_behaviors', ('behavior_server',)),
    'bt_navigator': ('nav2_bt_navigator', ('bt_navigator',)),
    'velocity_smoother': ('nav2_velocity_smoother', ('velocity_smoother',)),
    'map_server': ('nav2_map_server', ('map_server',)),
    'lifecycle_manager_navigation': ('nav2_lifecycle_manager', ('lifecycle_manager',)),
    'robot_state_publisher': ('robot_state_publisher', ('robot_state_publisher',)),
    'localization_monitor': ('nav_perception_iface', ('localization_monitor',)),
    'chassis_adapter': ('chassis_adapter', ('chassis_adapter', 'chassis_adapter_cpp')),
    'navigation_executor': ('nav_executor', ('nav_task_manager',)),
    'fake_chassis': ('nav_sim', ('fake_chassis',)),
    'fake_localization': ('nav_sim', ('fake_localization',)),
}


def _read(root, name):
    relative = Path(name)
    if relative.is_absolute() or '..' in relative.parts or relative.as_posix() != name:
        raise ValueError('non-private experiment artifact')
    path = root / relative
    if any(parent.is_symlink() for parent in (path, *path.parents)):
        raise ValueError('symlinked experiment artifact')
    with path.open('rb') as stream:
        data = stream.read(16 * 1024 * 1024 + 1)
    if len(data) > 16 * 1024 * 1024:
        raise ValueError('oversized experiment artifact')
    return data


def _graph(root, record, assets):
    namespace, value = record['namespace'], record['graph']
    if (value['mode'] != record['mode'] or value['composition'] is not False
            or value['supervisor_keyboard'] is not False):
        raise ValueError('invalid selftest graph mode or process ownership')
    processes = []
    prefix = '/' + namespace + '/'
    for fields in value['processes']:
        fields = dict(fields)
        for name in ('arguments', 'configuration_files'):
            fields[name] = tuple(fields[name])
        fields['remappings'] = tuple(tuple(pair) for pair in fields['remappings'])
        process = ProcessSpec(**fields)
        allowed = _PROGRAMS.get(process.name)
        if (allowed is None or process.package != allowed[0]
                or process.executable not in allowed[1]):
            raise ValueError('unexpected program in selftest; hardware is forbidden')
        if process.restart_limit != 0 or process.delay_sec != 0:
            raise ValueError('selftest processes must be single owned instances')
        for path in (*process.configuration_files, process.parameter_file):
            if path and Path(path).relative_to(root).as_posix() not in assets:
                raise ValueError('unverified process configuration')
        if process.ros_node:
            for topic in ('tf', 'tf_static', 'nav/events'):
                if ('/' + topic, prefix + topic) not in process.remappings:
                    raise ValueError('missing private ROS transport remapping')
            if any(not target.startswith(prefix) for _, target in process.remappings):
                raise ValueError('public topic in selftest process')
        elif (process.name != 'navigation_executor' or not process.wait_for_navigation
              or process.ready_action != prefix + 'navigate_to_pose'
              or '__ns:=/' + namespace not in process.arguments
              or '/nav/events:=' + prefix + 'nav/events' not in process.arguments):
            raise ValueError('executor is not privately gated')
        processes.append(process)
    names = [item.name for item in processes]
    if len(set(names)) != len(names) or set(names) not in (
            set(_PROGRAMS), set(_PROGRAMS) - {'velocity_smoother'}):
        raise ValueError('selftest requires the complete real Nav2 and fake robot graph')
    return LaunchGraph(record['mode'], tuple(processes), tuple(value['lifecycle_nodes']))


def load_scene(root: Path, digest: str) -> Scene:
    """Check the parent-held plan digest, all file hashes and the hardware-free graph.

    This is an integrity check in an exclusively owned directory, not protection
    against an administrator or concurrent same-user edits after verification.

    Args:
        root: Previously created private experiment directory, never an operator run.
        digest: Exact SHA256 returned by prepare_scene, not read from a marker file.

    Returns:
        Verified graph and expected evidence, still without execution authority.

    Raises:
        ValueError: Identity, hash, graph, artifact location or size is invalid.
        OSError: The private plan or an expected artifact cannot be read.
    """
    if not root.is_absolute() or '..' in root.parts or root.is_symlink():
        raise ValueError('expected an absolute private experiment directory')
    payload = _read(root, 'selftest.json')
    if hashlib.sha256(payload).hexdigest() != digest:
        raise ValueError('selftest plan changed after preparation')
    record = json.loads(payload)
    if (record['schema_version'] != 1 or record['backend'] != 'real_nav2'
            or record['executed'] is not False or record['mode'] not in ('tour', 'precise')
            or re.fullmatch(r'selftest_[0-9a-f]{32}', record['namespace']) is None):
        raise ValueError('invalid selftest identity')
    assets = record['asset_hashes']
    if not isinstance(assets, dict) or not assets or len(assets) > 128:
        raise ValueError('invalid selftest asset manifest')
    for name, expected in assets.items():
        if hashlib.sha256(_read(root, name)).hexdigest() != expected:
            raise ValueError('selftest artifact changed after preparation')
    cases = tuple(ExpectedCase(**dict(case, goal=tuple(case['goal']))) for case in record['cases'])
    check_evidence(cases, [], [])  # Validate expectations; absent observations are not success.
    return Scene(root, record['namespace'], _graph(root, record, assets), cases, digest)
