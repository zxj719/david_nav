"""Dock-specific retained static map and bounded navigation-frame TF observations."""

import math
import threading
from typing import Callable

from nav_tasks.domain.errors import ClosedError
from nav_tasks.domain.footprint import FootprintCheck, OccupancyGridView
from nav_tasks.skills.docking.approach import check_approach_footprint
from nav_tasks.skills.docking.ports import DockLogger, PlanarPose


class DockObservations:
    """Own one static-map subscription, sharing the runtime's node and TF buffer.

    Unlike ordinary POI snap observations, Dock uses transient-local reliable
    depth-one QoS and an immutable full-grid snapshot retaining map-origin yaw.
    No localization resources, odometry subscription or executor are created.
    """

    def __init__(
        self, node: object, tf_buffer: object, *, global_frame: str,
        robot_base_frame: str, static_map_topic: str,
        footprint: tuple[tuple[float, float], ...], occupied_threshold: int,
        logger: DockLogger, pose_timeout_sec: float = 2.0,
        occupancy_type: type | None = None, qos: object | None = None,
        latest_time: Callable[[], object] | None = None,
        duration: Callable[..., object] | None = None,
    ) -> None:
        """Create the sole owned subscription with explicit navigation wiring.

        Args:
            node: Caller-owned node, kept alive until worker and observation close.
            tf_buffer: Caller-owned buffer, safe for reads from the Dock worker.
            global_frame: Target frame used by the old Dock pose lookup and map guard.
            robot_base_frame: Navigation rotation center, not the POI capture frame.
            static_map_topic: Static occupancy source, not a dynamic costmap.
            footprint: Immutable navigation-frame polygon.
            occupied_threshold: Inclusive occupied-cell cutoff; unknown remains allowed.
            logger: Worker-safe sink for the frozen TF-failure line.
            pose_timeout_sec: Old bounded lookup budget, default two seconds.
            occupancy_type: Optional fake message type for pure tests.
            qos: Optional fake QoS for tests; production uses retained depth-one data.
            latest_time: Optional fake latest-time factory.
            duration: Optional fake ROS duration factory.
        """
        if occupancy_type is None:
            from nav_msgs.msg import OccupancyGrid
            occupancy_type = OccupancyGrid
        if qos is None:
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        if latest_time is None:
            from rclpy.time import Time
            latest_time = Time
        if duration is None:
            from rclpy.duration import Duration
            duration = Duration
        self._node = node
        self._buffer = tf_buffer
        self._global_frame = global_frame
        self._robot_frame = robot_base_frame
        self._footprint = footprint
        self._threshold = occupied_threshold
        self._logger = logger
        self._timeout = duration(seconds=pose_timeout_sec)
        self._latest_time = latest_time
        self._lock = threading.Lock()
        self._closed = False
        self._grid: OccupancyGridView | None = None
        self._subscription = node.create_subscription(
            occupancy_type, static_map_topic, self._receive, qos)

    def _receive(self, message):
        with self._lock:
            if self._closed:
                return
        view = OccupancyGridView.from_message(message)
        with self._lock:
            if not self._closed:
                self._grid = view

    def pose(self) -> PlanarPose | None:
        """Read the latest navigation rotation center on the blocking worker.

        Returns:
            Unmodified legacy planar TF arithmetic, or None after a logged lookup failure.

        Raises:
            ClosedError: Observations are closing; the worker should already be joined.
        """
        with self._lock:
            if self._closed:
                raise ClosedError('Dock observations are closed')
        try:
            transform = self._buffer.lookup_transform(
                self._global_frame, self._robot_frame, self._latest_time(), timeout=self._timeout)
        except Exception as error:
            self._logger.warn('[dock] 取位姿失败: %s' % error)
            return None
        position = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return position.x, position.y, yaw

    def check_approach(self, pose: PlanarPose) -> FootprintCheck:
        """Check the entire footprint using one thread-safe immutable map snapshot.

        Args:
            pose: Computed approach pose in the configured global frame.

        Returns:
            Existing missing-map/frame/geometry/occupancy admission verdict.

        Raises:
            ClosedError: Observations are closing or closed.
        """
        with self._lock:
            if self._closed:
                raise ClosedError('Dock observations are closed')
            grid = self._grid
        return check_approach_footprint(
            grid, self._global_frame, self._footprint, pose, self._threshold)

    def close(self) -> None:
        """Seal callbacks and destroy only this subscription, retaining failures for retry.

        Raises:
            RuntimeError: The node reports unsuccessful subscription removal.
        """
        with self._lock:
            self._closed = True
        if self._subscription is not None:
            if self._node.destroy_subscription(self._subscription) is False:
                raise RuntimeError('failed to destroy Dock map subscription')
            self._subscription = None
