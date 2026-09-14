"""nav2 参数文件的帧一致性不变量。

2026-09-10 实测：`behavior_server.global_frame: map` 而 `local_costmap.global_frame:
odom`，Spin 行为在 map 系算出模拟位姿、却拿去问 odom 系的 3×3m rolling 局部代价图，
坐标落在图外 -> behavior_server 报 `Pose Goes Off Grid` -> `Collision Ahead -
Exiting Spin` -> 每次 Spin 都在 ~100ms 内 ABORT。

也就是说这台车上的 Spin 恢复行为**从来没成功过** —— 而 dock_navigator 的末端朝向
补偿完全依赖它。这个不变量把这类"两处配置必须一致"的坑钉死。
"""

import glob
import os

import yaml

CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')


def _param_files():
    files = sorted(glob.glob(os.path.join(CONFIG_DIR, 'nav2_params_2d*.yaml')))
    assert files, '没找到任何 nav2_params_2d*.yaml'
    return files


def _load(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def test_behavior_server_global_frame_matches_local_costmap():
    """behavior_server 用哪个系算位姿，就必须用哪个系的代价图。

    behavior_server 的 costmap_topic 指向 local_costmap/costmap_raw，
    该话题发布在 local_costmap 的 global_frame 里。两者不一致 -> 位姿出图 -> 恢复行为全废。
    """
    bad = []
    for path in _param_files():
        cfg = _load(path)
        bs = cfg.get('behavior_server', {}).get('ros__parameters', {})
        lc = cfg.get('local_costmap', {}).get('local_costmap', {}).get(
            'ros__parameters', {})
        if not bs or not lc:
            continue
        # costmap_topic 必须确实指向 local_costmap，否则这条不变量不适用
        if 'local_costmap' not in str(bs.get('costmap_topic', '')):
            continue
        if bs.get('global_frame') != lc.get('global_frame'):
            bad.append('%s: behavior_server=%r local_costmap=%r'
                       % (os.path.basename(path), bs.get('global_frame'),
                          lc.get('global_frame')))
    assert not bad, '帧不一致（Spin/BackUp 等恢复行为会因位姿出图而全部 ABORT）:\n' \
                    + '\n'.join(bad)


def test_behavior_server_and_costmaps_agree_on_robot_base_frame():
    """robot_base_frame 三处必须一致 —— David 的 /base 与 base_footprint 不同姿态，
    混用会让 footprint 贴错位置（见 project_david_base_frame_imu_rotation）。"""
    bad = []
    for path in _param_files():
        cfg = _load(path)
        frames = {}
        bs = cfg.get('behavior_server', {}).get('ros__parameters', {})
        if bs.get('robot_base_frame'):
            frames['behavior_server'] = bs['robot_base_frame']
        for name in ('local_costmap', 'global_costmap'):
            node = cfg.get(name, {}).get(name, {}).get('ros__parameters', {})
            if node.get('robot_base_frame'):
                frames[name] = node['robot_base_frame']
        if len(set(frames.values())) > 1:
            bad.append('%s: %s' % (os.path.basename(path), frames))
    assert not bad, 'robot_base_frame 不一致:\n' + '\n'.join(bad)
