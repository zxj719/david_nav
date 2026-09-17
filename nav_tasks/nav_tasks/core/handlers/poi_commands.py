"""实现不获取或抢占运动所有权的 POI 管理操作."""

from dataclasses import dataclass
from typing import Callable, Iterable

from nav_tasks.core.ports import PoiRepository, PoseSource
from nav_tasks.domain.types import StoredPoi


@dataclass(frozen=True)
class MarkPoiResult:
    """保存点位标记结果，确认帧和线协议编码由入站层负责.

    数据类不校验 poi/error 互斥；处理器按成功或失败分支填写，并不自行发送 ACK。

    Attributes:
        error: 失败说明；成功时为 None。
        poi: 成功标记后的 POI，失败时为 None。
    """

    poi: StoredPoi | None = None
    error: str | None = None


class PoiCommands:
    """通过注入的位姿和仓库端口管理点位，不持有导航后端."""

    def __init__(
        self, poses: PoseSource, pois: PoiRepository, now: Callable[[], float],
        pose_stale_after_sec: float = 0.5,
    ) -> None:
        """注入点位管理端口及与位姿时间戳兼容的当前时间函数.

        不在构造时验证端口或阈值；实例不持有运动所有权，也不创建 ROS 资源。

        Args:
            poses: 位姿捕获来源，不是定位栈实现。
            pois: 按仓库契约负责写入的存储端口。
            now: 与捕获位姿时间戳使用同一时间基准的当前时间函数。
            pose_stale_after_sec: 地图位姿允许的最大年龄，单位秒。
        """
        self._poses = poses
        self._pois = pois
        self._now = now
        self._stale_after = pose_stale_after_sec

    async def mark(self, poi_id: str, name: str) -> MarkPoiResult:
        """捕获并写入点位，保持旧新鲜度与错误处理语义.

        只把捕获阶段的普通 Exception 转为可读失败；捕获后的时钟/字段/存储异常
        直接传播。仅检查地图位姿年龄是否严格大于阈值，不额外拒绝未来或 NaN
        时间，也不检查 odom 位姿年龄。输入身份应由入站解码。

        Args:
            poi_id: 已解码的点位标识。
            name: 已解码的显示名称。

        Returns:
            带仓库返回值的成功结果，或捕获/地图位姿过期错误。
            仓库正常返回不等同于本函数额外提供了持久化或传输 ACK 证明。

        Raises:
            asyncio.CancelledError: 位姿捕获被取消。
            Exception: 捕获后字段、时钟或仓库存储处理失败。
        """
        try:
            captured = await self._poses.capture_for_poi()
        except Exception as error:
            return MarkPoiResult(error=str(error))
        if self._now() - captured.map_pose.stamp_sec > self._stale_after:
            return MarkPoiResult(error='当前位置位姿数据过期')
        stored = self._pois.upsert(StoredPoi(
            poi_id, name, captured.map_pose, captured.odom_pose, captured.slam_session_id))
        return MarkPoiResult(poi=stored)

    def update_directory(self, id_to_name: Iterable[tuple[str, str]]) -> None:
        """直接委托仓库更新显示名称，不捕获位姿或创建运动请求.

        Args:
            id_to_name: 已解码且按输入顺序迭代的标识/名称对。

        Raises:
            Exception: 输入迭代或仓库更新失败。
        """
        self._pois.rename_many(id_to_name)
