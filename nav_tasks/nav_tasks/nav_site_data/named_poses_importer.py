"""只读导入人工维护的命名点 JSON，不改源文件或重新解释坐标系."""

import json
from pathlib import Path
from typing import Callable

from nav_tasks.core.ports import PoiRepository
from nav_tasks.domain.types import PoseRecord, StoredPoi


def import_named_poses(
    repository: PoiRepository, path: Path, *, map_frame: str, odom_frame: str,
    log: Callable[[str], None] | None = None,
) -> int:
    """按旧命名点格式逐条导入，成功遍历后只裁剪 named_poses 会话.

    接受旧列表及 poses/named_poses 真值回退形式；非映射条目跳过。
    数值转换后原样复制进地图及里程计两种表示，只设置帧标签而不做 TF 变换。
    每条 upsert 独立写入，后续坏记录不撤回先前写入，失败时不执行尾部裁剪。
    日志在裁剪完成后调用，因此日志失败也可能发生在存储已经更新之后。

    Args:
        repository: 执行器持有的可写 POI 仓库，不应指向人工源文件。
        path: 只读命名点 JSON 路径。
        map_frame: 条目未包含 frame_id 时的回退帧；已有值沿用 str 转换。
        odom_frame: 旧复制坐标的里程计表示帧标签，不表示已完成坐标变换。
        log: 可选的无前缀裁剪日志接收器，输出冻结的陈旧 ID 日志。

    Returns:
        已导入映射条目数，重复 ID 重复计数；最终同 ID 记录由最后一项覆盖。

    Raises:
        OSError: 源读取或目标仓库写入失败。
        json.JSONDecodeError: 源文件不是合法 JSON。
        KeyError: 映射条目缺少必需 id。
        Exception: 旧格式转换、仓库操作或日志回调失败，不回滚已完成写入。
    """
    with Path(path).open('r', encoding='utf-8') as handle:
        payload = json.load(handle)
    items = (payload.get('poses') or payload.get('named_poses') or []) if (
        isinstance(payload, dict)) else payload
    count = 0
    imported = set()
    for entry in items:
        if not isinstance(entry, dict):
            continue
        position = entry.get('position', {}) or {}
        poi_id = str(entry['id'])
        name = str(entry.get('name', poi_id))
        frame = str(entry.get('frame_id', map_frame))
        x, y, z = (float(position.get(axis, 0.0)) for axis in ('x', 'y', 'z'))
        yaw = float(entry.get('yaw', 0.0))
        repository.upsert(StoredPoi(
            poi_id, name, PoseRecord(frame, x, y, z, yaw, 0.0),
            PoseRecord(odom_frame, x, y, z, yaw, 0.0), 'named_poses'))
        imported.add(poi_id)
        count += 1
    removed = repository.prune_session('named_poses', imported)
    if removed and log is not None:
        log(f'pruned stale named_poses POIs: {sorted(removed)}')
    return count
