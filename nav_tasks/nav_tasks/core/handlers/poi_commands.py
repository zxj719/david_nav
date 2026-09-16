"""Administrative POI operations that never acquire or preempt motion ownership."""

from dataclasses import dataclass
from typing import Callable, Iterable

from nav_tasks.core.ports import PoiRepository, PoseSource
from nav_tasks.domain.types import StoredPoi


@dataclass(frozen=True)
class MarkPoiResult:
    """POI mark completion; ingress supplies acknowledgement and wire encoding.

    Attributes:
        error: 失败说明；成功时为 None。
        poi: 成功标记后的 POI，失败时为 None。
    """

    poi: StoredPoi | None = None
    error: str | None = None


class PoiCommands:
    """Perform pose capture and directory updates without a navigation backend."""

    def __init__(
        self, poses: PoseSource, pois: PoiRepository, now: Callable[[], float],
        pose_stale_after_sec: float = 0.5,
    ) -> None:
        """Inject administrative ports and a pose-compatible timestamp clock.

        Args:
            poses: Pose capture source, not the localization implementation.
            pois: Repository for atomic persistence.
            now: Current time in the same time base as captured pose stamps.
            pose_stale_after_sec: Maximum tolerated global-pose age.
        """
        self._poses = poses
        self._pois = pois
        self._now = now
        self._stale_after = pose_stale_after_sec

    async def mark(self, poi_id: str, name: str) -> MarkPoiResult:
        """Capture and persist a POI, preserving existing freshness/error behavior.

        Args:
            poi_id: POI identifier already decoded by ingress.
            name: Display name already decoded by ingress.

        Returns:
            Stored POI, or a readable capture/freshness error. Persistence failures
            propagate rather than falsely acknowledging durable success.

        Raises:
            asyncio.CancelledError: Cancellation of pose capture propagates.
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
        """Update display names only; directory commands do not create poses.

        Args:
            id_to_name: Decoded POI identifier/name pairs, in input order.
        """
        self._pois.rename_many(id_to_name)
