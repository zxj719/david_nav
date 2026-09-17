"""以兼容旧格式的 YAML 持久化 POI，不依赖 ROS 或传输实现."""

from dataclasses import asdict, replace
import math
import os
from pathlib import Path
from typing import Iterable

import yaml

from nav_tasks.domain.types import PoseRecord, StoredPoi, Transform2D


class YamlPoiRepository:
    """提供单写入者 POI 仓库，保留旧文件结构及合并顺序.

    可选来源只读，本地文件覆盖同 ID；内存变更先于写入，失败可能使内存领先磁盘。
    同目录临时文件关闭后原子替换目标，不提供 fsync 持久化或多进程事务。
    调用方必须提供互不冲突的来源与缓存路径；构造器不校验路径别名或符号链接。

    Attributes:
        path: 供本仓库写入的 POI 缓存 YAML 路径。
        source_path: 可选的只读初始 POI YAML 路径。
    """

    def __init__(
        self, path: Path, source_path: Path | None = None, *, map_frame: str,
    ) -> None:
        """保存文件路径及全局回退帧并创建空缓存，不读写磁盘或加载记录.

        路径转为 Path，但不验证可写性、源/目标是否相同或帧名是否合法。

        Args:
            path: 调用方指定的可写缓存文件，必须避免覆盖人工源文件。
            source_path: 可选的只读初始 YAML 来源。
            map_frame: 重投影目标帧，也是旧记录缺失帧名时的回退值。
        """
        self.path = Path(path)
        self.source_path = Path(source_path) if source_path is not None else None
        self._map_frame = map_frame
        self._pois: dict[str, StoredPoi] = {}

    def load(self) -> None:
        """清空缓存后先加载来源再加载本地记录，缺失文件沿用忽略策略.

        本地同 ID 覆盖来源；非列表集合、非映射条目及缺 ID 条目沿用跳过行为。
        不修复或重写数据，不提供整次加载原子性：失败时缓存可只含部分新记录。

        Raises:
            OSError: 来源或缓存读取失败。
            yaml.YAMLError: YAML 内容解析失败。
            Exception: 单条旧格式转换失败，已经合并的记录不回滚。
        """
        self._pois = {}
        if self.source_path is not None:
            self._load_from(self.source_path)
        self._load_from(self.path)

    def _load_from(self, path: Path) -> None:
        """只读合并一个 YAML 文件，同名 POI 覆盖内存中的先前记录.

        Args:
            path: 初始来源或本地缓存路径；不存在时忽略，不自动创建文件。

        Raises:
            OSError: 文件读取失败。
            yaml.YAMLError: YAML 解析失败。
            Exception: 单条记录转换失败；已合并的记录不会回滚。
        """
        if not path.exists():
            return
        with path.open('r', encoding='utf-8') as handle:
            payload = yaml.safe_load(handle) or {}
        items = payload.get('pois', []) if isinstance(payload, dict) else payload
        if not isinstance(items, list):
            return
        for item in items:
            if not isinstance(item, dict):
                continue
            stored = self._stored_poi_from_mapping(item)
            if stored is not None:
                self._pois[stored.poi_id] = stored

    def get(self, poi_id: str) -> StoredPoi | None:
        """只读返回内存中保存的不可变记录，不读取磁盘或创建缺失点.

        Args:
            poi_id: 用于缓存字典查询的 POI 标识，不在此转换或校验。

        Returns:
            已存记录本身或 None，不创建副本。
        """
        return self._pois.get(poi_id)

    def upsert(self, poi: StoredPoi) -> StoredPoi:
        """先更新内存记录，再以临时文件替换完整缓存 YAML.

        不额外验证记录语义；写入失败不回滚已替换的内存值。

        Args:
            poi: 调用方提供的不可变记录。

        Returns:
            持久化调用成功后返回原输入记录。

        Raises:
            OSError: 目录创建、文件写入或原子替换失败。
            yaml.YAMLError: YAML 序列化失败。
            Exception: 输入记录不符合已有字段或序列化要求。
        """
        self._pois[poi.poi_id] = poi
        self._write()
        return poi

    def rename_many(self, id_to_name: Iterable[tuple[str, str]]) -> None:
        """按输入顺序修改已知点名称并写缓存，不创建未知点或改变几何.

        同 ID 的最后一次修改生效；即使没有记录变化，成功遍历后仍写文件。
        遍历或写入失败可留下部分内存修改，不提供整批事务回滚。

        Args:
            id_to_name: 按处理顺序提供的标识与名称二元组。

        Raises:
            OSError: 缓存持久化失败。
            yaml.YAMLError: 修改后的记录无法序列化。
            Exception: 输入遍历、解包或记录替换失败。
        """
        for poi_id, name in id_to_name:
            if poi_id in self._pois:
                self._pois[poi_id] = replace(self._pois[poi_id], name=name)
        self._write()

    def prune_session(self, session_id: str, keep_ids: set[str]) -> list[str]:
        """只移除指定会话内未保留的记录，有实际删除时才写缓存.

        不修改其它会话。删除先作用于内存，持久化失败不恢复已移除记录。

        Args:
            session_id: 允许裁剪的权威会话标识。
            keep_ids: 在该会话来源中仍存在的 ID 集合。

        Returns:
            按原仓库顺序返回已删除的 ID；无删除时为空列表。

        Raises:
            OSError: 变更后缓存持久化失败。
            yaml.YAMLError: 剩余记录无法序列化。
        """
        removed = [key for key, poi in self._pois.items()
                   if poi.slam_session_id == session_id and key not in keep_ids]
        for key in removed:
            del self._pois[key]
        if removed:
            self._write()
        return removed

    def reproject_session(
        self, session_id: str, map_from_odom: Transform2D,
        translation_threshold_m: float, yaw_threshold_rad: float,
    ) -> list[StoredPoi]:
        """重投影指定会话的里程计锚点，保留未归一化 yaw 及旧阈值语义.

        以里程计锚点重算地图 x/y/yaw，保留高度、时间与原始锚点。
        平面距离和原始 yaw 差都不超过阈值时跳过；角度差不归一。
        按仓库顺序更新内存，有变化才整体写盘。计算或写入失败不回滚先前更新，
        也不在此校验坐标帧、有限性或阈值合法性。

        Args:
            session_id: 允许重投影的会话。
            map_from_odom: 从里程计坐标系到配置全局坐标系的二维变换。
            translation_threshold_m: 含等号的平面不变阈值（米）。
            yaw_threshold_rad: 含等号的原始 yaw 差不变阈值（弧度）。

        Returns:
            按原仓库顺序返回已更新记录，无变化时为空列表。

        Raises:
            OSError: 变更后的缓存持久化失败。
            yaml.YAMLError: 更新后的记录无法序列化。
            Exception: 变换计算或输入字段处理失败，先前内存变更不回滚。
        """
        changed = []
        for key, poi in list(self._pois.items()):
            if poi.slam_session_id != session_id:
                continue
            pose = self._reproject_pose(map_from_odom, poi.odom_pose)
            dx, dy = pose.x - poi.map_pose.x, pose.y - poi.map_pose.y
            dyaw = pose.yaw - poi.map_pose.yaw
            if math.hypot(dx, dy) <= translation_threshold_m and abs(dyaw) <= yaw_threshold_rad:
                continue
            updated = replace(poi, map_pose=pose)
            self._pois[key] = updated
            changed.append(updated)
        if changed:
            self._write()
        return changed

    def _reproject_pose(self, transform: Transform2D, pose: PoseRecord) -> PoseRecord:
        # Do not use the normalized geometry helper: existing files and threshold
        # decisions retain yaw outside [-pi, pi). See site-data.md.
        """按二维变换重投影 POI，保留旧存储要求的未归一化 yaw.

        旋转平移 x/y，直接累加 yaw；不改 z 和采样时间，也不使用归一化几何助手。

        Args:
            transform: 平面旋转平移，平移为米、yaw 为弧度。
            pose: 待重投影的位姿记录，输入坐标应与变换语义一致。

        Returns:
            使用配置地图帧的新位姿，yaw 可超出 [-pi, pi)。
        """
        cosine, sine = math.cos(transform.yaw), math.sin(transform.yaw)
        return PoseRecord(
            self._map_frame, transform.x + cosine * pose.x - sine * pose.y,
            transform.y + sine * pose.x + cosine * pose.y, pose.z,
            transform.yaw + pose.yaw, pose.stamp_sec)

    def _write(self) -> None:
        """按内存顺序写同目录临时 YAML，再原子替换本地缓存文件.

        只用于单写入者；不写只读来源、不提供 fsync 持久化保证。上层已修改
        内存，写失败不回滚内存，也不主动删除可能留下的 .tmp 文件。

        Raises:
            OSError: 创建目录、写入文件或替换缓存失败。
            yaml.YAMLError: POI 数据无法序列化。
        """
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + '.tmp')
        payload = {'pois': [
            {'id': poi.poi_id, 'name': poi.name, 'map_pose': asdict(poi.map_pose),
             'odom_pose': asdict(poi.odom_pose), 'slam_session_id': poi.slam_session_id}
            for poi in self._pois.values()
        ]}
        with temporary.open('w', encoding='utf-8') as handle:
            yaml.safe_dump(payload, handle, allow_unicode=True, sort_keys=False)
        os.replace(temporary, self.path)

    def _stored_poi_from_mapping(self, item: dict) -> StoredPoi | None:
        """按旧 POI 映射格式恢复记录，缺少 ID 时跳过而不制造匿名点.

        map_pose 和 odom_pose 缺失或为空时各自回退到整个条目；
        名称回退到 ID，空会话标识回退到既有 semantic_map 标签。

        Args:
            item: 已由调用方确认可读取的 POI 字典。

        Returns:
            恢复的 StoredPoi；id 为 None 或不存在时返回 None。

        Raises:
            Exception: 嵌套位姿格式不合法或其字段不能转换。
        """
        poi_id = item.get('id')
        if poi_id is None:
            return None
        return StoredPoi(
            str(poi_id), str(item.get('name') or poi_id),
            self._pose_from_mapping(item.get('map_pose') or item),
            self._pose_from_mapping(item.get('odom_pose') or item),
            str(item.get('slam_session_id') or 'semantic_map'))

    def _pose_from_mapping(self, item: dict) -> PoseRecord:
        """按旧格式读取位姿，顶层坐标键优先于嵌套 position.

        缺失坐标、yaw 和时间使用零，空帧名使用配置地图帧。
        此处只转换数值，不校验有限性、不转换坐标帧，也不归一化 yaw。

        Args:
            item: 位姿字典，可含顶层 x/y/z 或嵌套 position。

        Returns:
            转为 float 的 PoseRecord，保留既有字段回退语义。

        Raises:
            TypeError: 数值字段不能转换，或嵌套结构类型不适用。
            ValueError: 数值文本不能转换为浮点数。
            AttributeError: 非映射对象缺少 get 方法。
        """
        position = item.get('position') or {}
        return PoseRecord(
            str(item.get('frame_id') or self._map_frame),
            float(item.get('x', position.get('x', 0.0))),
            float(item.get('y', position.get('y', 0.0))),
            float(item.get('z', position.get('z', 0.0))),
            float(item.get('yaw', 0.0)), float(item.get('stamp_sec', 0.0)))
