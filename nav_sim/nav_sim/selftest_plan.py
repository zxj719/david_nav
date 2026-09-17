"""在导入 ROS 运行时前核验准备计划、私有资产及无硬件启动图."""

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
    """读取私有实验内的规范相对路径文件，拒绝符号链接和超大内容.

    Args:
        root: 调用方独占的实验根目录。
        name: 必须保持规范拼写、无上级跳转的相对路径字符串。

    Returns:
        最多 16 MiB 的文件字节；不执行内容，不验证其哈希。

    Raises:
        ValueError: 路径越界/非规范、任一路径父级为符号链接或文件超限。
        OSError: 文件无法打开或读取。本检查不防御同用户并发替换路径。
    """
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
    """把已读取的计划重建为私有启动图，校验程序白名单与配置引用.

    Args:
        root: 实验根目录，配置路径必须属于此目录。
        record: 含模式、命名空间和进程图的计划映射。
        assets: 已在 load_scene 中逐项核对哈希的资产清单。

    Returns:
        普通单实例 Nav2、任务管理器和假机器人组成的 LaunchGraph；
        只构建描述，不启动进程，也不赋予发送目标的权限。

    Raises:
        ValueError: 模式、程序、重启/延迟策略、私有接线或进程集合不符。
        KeyError: 计划缺少所需结构字段。
        TypeError: 进程记录不能构造成 ProcessSpec。
    """
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
    """核对父进程持有的计划摘要、资产哈希和无硬件私有图.

    这是独占目录中的完整性检查，不防管理员或同用户在验证后的并发修改。
    结构缺失还可能传播 KeyError、TypeError；不把任意坏输入都统一为 ValueError。

    Args:
        root: 已创建的私有实验目录，不是操作员运行目录。
        digest: prepare_scene 返回并由父进程保留的精确 SHA256，不从标记文件自取。

    Returns:
        验证过的图和预期用例；没有启动进程，也不授予执行或运动权限。

    Raises:
        ValueError: 身份、哈希、图、资产位置或大小不合法。
        OSError: 私有计划或预期资产无法读取。
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
