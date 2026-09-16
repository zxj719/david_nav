"""Navigation-owned TF/odometry observations; never controls localization."""

import threading
from typing import Callable

from nav_tasks.domain.errors import ClosedError, PoseUnavailableError
from nav_tasks.domain.geometry import legacy_yaw_from_quaternion
from nav_tasks.domain.types import CapturedPose, PoseRecord, Transform2D


def _yaw(rotation: object) -> float:
    return legacy_yaw_from_quaternion(
        float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w))


def _stamp(stamp: object) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


class TfPoseSource:
    """Read injected TF buffer and one depth-10 odometry subscription.

    Node, TF buffer/listener and executor are owned by the composition root.
    The only resource owned here is the created odometry subscription. Callback
    snapshots are protected across ROS threads; no worker or spin is started.
    """

    def __init__(
        self, node: object, tf_buffer: object, *, robot_base_frame: str,
        odom_topic: str, map_frame: str, odom_frame: str,
        session_id: Callable[[], str], odometry_type: type | None = None,
        latest_time: Callable[[], object] | None = None,
    ) -> None:
        """Inject ROS resources and explicit configured frame/topic names.

        Args:
            node: Node used only to create/destroy this subscription.
            tf_buffer: Existing buffer used for latest-time lookups.
            robot_base_frame: Exact configured robot observation frame.
            odom_topic: Existing odometry topic, not automatically rewired.
            map_frame: Global TF target and empty-header fallback.
            odom_frame: Odometry TF source and empty-message-frame fallback.
            session_id: Existing session identity supplier.
            odometry_type: Optional fake message class; lazily import ROS otherwise.
            latest_time: Optional latest-time factory; default is rclpy Time.
        """
        if odometry_type is None:
            from nav_msgs.msg import Odometry
            odometry_type = Odometry
        if latest_time is None:
            from rclpy.time import Time
            latest_time = Time
        self._node = node
        self._buffer = tf_buffer
        self._robot_frame = robot_base_frame
        self._odom_topic = odom_topic
        self._map_frame = map_frame
        self._odom_frame = odom_frame
        self._session_id = session_id
        self._latest_time = latest_time
        self._lock = threading.Lock()
        self._closed = False
        self._odom: PoseRecord | None = None
        self._subscription = node.create_subscription(
            odometry_type, odom_topic, self._on_odom, 10)

    def _check_open(self) -> None:
        with self._lock:
            if self._closed:
                raise ClosedError('pose source is closed')

    def _on_odom(self, message: object) -> None:
        with self._lock:
            if self._closed:
                return
        pose = message.pose.pose
        captured = PoseRecord(
            message.header.frame_id or self._odom_frame,
            float(pose.position.x), float(pose.position.y), float(pose.position.z),
            _yaw(pose.orientation), _stamp(message.header.stamp))
        with self._lock:
            if not self._closed:
                self._odom = captured

    def _lookup(self, source: str) -> object:
        self._check_open()
        try:
            return self._buffer.lookup_transform(self._map_frame, source, self._latest_time())
        except Exception as error:
            raise PoseUnavailableError(str(error)) from error

    def current_map_pose(self) -> PoseRecord:
        """Read the latest configured global-from-robot pose.

        Returns:
            Pose retaining TF header frame/stamp and legacy quaternion arithmetic.

        Raises:
            PoseUnavailableError: TF lookup failed; its cause and message are retained.
            ClosedError: Observation resources are closing or closed.
        """
        transform = self._lookup(self._robot_frame)
        position = transform.transform.translation
        return PoseRecord(
            transform.header.frame_id or self._map_frame,
            float(position.x), float(position.y), float(position.z),
            _yaw(transform.transform.rotation), _stamp(transform.header.stamp))

    async def capture_for_poi(self) -> CapturedPose:
        """Capture latest map and odometry observations with existing session identity.

        As before, map and odometry are latest independent samples, not a new
        timestamp-synchronization policy. Administrative freshness checks remain
        in PoiCommands.

        Returns:
            Detached immutable pose observations.

        Raises:
            PoseUnavailableError: TF lookup failed or no odometry was received.
            ClosedError: Observation resources are closing or closed.
        """
        map_pose = self.current_map_pose()
        with self._lock:
            odom_pose = self._odom
        if odom_pose is None:
            raise PoseUnavailableError(f'no odometry available on {self._odom_topic}')
        return CapturedPose(map_pose, odom_pose, self.current_slam_session_id())

    def current_slam_session_id(self) -> str:
        """Read the externally supplied session identifier.

        Returns:
            Current localization session identity.
        """
        self._check_open()
        return self._session_id()

    def current_map_from_odom(self) -> Transform2D:
        """Read latest global-from-odometry transform without changing localization.

        Returns:
            Planar transform retaining the legacy yaw calculation.

        Raises:
            PoseUnavailableError: Lookup failed, with original cause retained.
            ClosedError: Observation resources are closing or closed.
        """
        transform = self._lookup(self._odom_frame).transform
        return Transform2D(float(transform.translation.x), float(transform.translation.y),
                           _yaw(transform.rotation))

    def close(self) -> None:
        """Seal callbacks and destroy only the subscription owned by this adapter.

        Failed destruction is retryable; no node/buffer/executor is destroyed.

        Raises:
            RuntimeError: The node reports failure to remove the subscription.
        """
        with self._lock:
            self._closed = True
        if self._subscription is not None:
            if self._node.destroy_subscription(self._subscription) is False:
                raise RuntimeError('failed to destroy odometry subscription')
            self._subscription = None
