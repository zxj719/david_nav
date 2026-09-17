"""读取导航侧 TF 和里程计观察，不控制或修改定位栈."""

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
    """借用 TF 缓冲区并持有一个深度十里程计订阅.

    节点、TF listener、缓冲区与 executor 均由装配根持有。回调缓存受锁保护，
    不启动工作线程或 spin；唯一自有 ROS 资源是里程计订阅。
    """

    def __init__(
        self, node: object, tf_buffer: object, *, robot_base_frame: str,
        odom_topic: str, map_frame: str, odom_frame: str,
        session_id: Callable[[], str], odometry_type: type | None = None,
        latest_time: Callable[[], object] | None = None,
    ) -> None:
        """注入既有 ROS 资源及显式帧名和话题，仅创建里程计订阅.

        未注入消息或时间类型时惰性导入；不检查帧名的完整配置约束，
        不改接定位输出、不创建 TF listener，也不启停定位。

        Args:
            node: 创建和销毁本订阅所用的外部节点。
            tf_buffer: 用于最新时刻查询的既有缓冲区。
            robot_base_frame: 精确配置的机器人观察帧，不与 Dock 旋转中心混称。
            odom_topic: 既有里程计话题，不自动改接。
            map_frame: 全局 TF 目标与空 header 帧的后备值。
            odom_frame: 里程计 TF 源与空消息帧的后备值。
            session_id: 当前会话标识提供者。
            odometry_type: 可选测试消息类；省略时导入 ROS 消息。
            latest_time: 可选最新时刻构造器；默认 rclpy Time。

        Raises:
            Exception: 类型导入或订阅创建失败。
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
        """在 ROS 回调中保存里程计快照，关闭期间不再写入缓存.

        Args:
            message: 里程计消息；保留 header 帧与时间（秒），位置使用米，
                四元数转换为弧度 yaw。空帧使用配置值，不自动改接话题。

        Raises:
            Exception: 消息读取或数值转换失败，由 ROS 宿主处理回调故障。
        """
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
        """查询最新的全局帧到指定源帧变换，并统一包装查询错误.

        先检查观察源仍开放；关闭错误不被包装成位姿暂不可用。
        不在此 spin 或等待特定时刻的变换。

        Args:
            source: 要查询的源坐标帧名称。

        Returns:
            TF 缓冲区返回的变换消息。

        Raises:
            ClosedError: 位姿观察源已经关闭。
            PoseUnavailableError: 最新时刻构造或 TF 查询失败，原异常保留为原因。
        """
        self._check_open()
        try:
            return self._buffer.lookup_transform(self._map_frame, source, self._latest_time())
        except Exception as error:
            raise PoseUnavailableError(str(error)) from error

    def current_map_pose(self) -> PoseRecord:
        """读取最新全局帧到配置机器人观察帧的位姿.

        保留 TF header 帧和时间，空帧才使用配置值；保留旧四元数计算与高度，
        不增加有限性、归一化或新鲜度检查。成功查询后的解析异常不包装成查询失败。

        Returns:
            新建 PoseRecord，距离为米、yaw 为弧度、时间为秒。

        Raises:
            PoseUnavailableError: 最新时刻构造或 TF 查询失败，保留原异常原因。
            ClosedError: 观察源正在关闭或已关闭。
            Exception: 查询成功后的字段读取或数值转换失败。
        """
        transform = self._lookup(self._robot_frame)
        position = transform.transform.translation
        return PoseRecord(
            transform.header.frame_id or self._map_frame,
            float(position.x), float(position.y), float(position.z),
            _yaw(transform.transform.rotation), _stamp(transform.header.stamp))

    async def capture_for_poi(self) -> CapturedPose:
        """采集独立的最新地图和缓存里程计样本，并关联当前会话标识.

        协程内部没有异步等待，也不等待下一帧里程计。样本时间不对齐、不在此
        校验新鲜度；POI 管理端另行执行自己的校验。里程计在回调时转换为记录，
        不会因原消息后续修改而变化。

        Returns:
            不可变位姿记录及会话标识组成的采集结果，不保证跨来源同一时刻。

        Raises:
            PoseUnavailableError: TF 查询失败或尚无里程计。
            ClosedError: 观察源正在关闭或已经关闭。
            Exception: TF 解析或会话标识回调失败。
        """
        map_pose = self.current_map_pose()
        with self._lock:
            odom_pose = self._odom
        if odom_pose is None:
            raise PoseUnavailableError(f'no odometry available on {self._odom_topic}')
        return CapturedPose(map_pose, odom_pose, self.current_slam_session_id())

    def current_slam_session_id(self) -> str:
        """确认观察源仍开放后读取外部提供的定位会话标识.

        不查询、重启或更改定位进程，不校验回调返回值。

        Returns:
            外部提供的当前会话标识。

        Raises:
            ClosedError: 观察源已封闭。
            Exception: 会话标识提供者失败。
        """
        self._check_open()
        return self._session_id()

    def current_map_from_odom(self) -> Transform2D:
        """读取最新全局帧到里程计帧的平面变换，不改变定位接线.

        只提取 x/y 和旧算法 yaw，不返回时间戳，也不增加新鲜度或有限性检查。

        Returns:
            当前平面 Transform2D。

        Raises:
            PoseUnavailableError: 最新时刻构造或 TF 查询失败，保留原异常原因。
            ClosedError: 观察源正在关闭或已经关闭。
            Exception: 查询成功后的字段或数值转换失败。
        """
        transform = self._lookup(self._odom_frame).transform
        return Transform2D(float(transform.translation.x), float(transform.translation.y),
                           _yaw(transform.rotation))

    def close(self) -> None:
        """封闭回调并销毁唯一自有里程计订阅，失败句柄可重试.

        不销毁节点、缓冲区、listener 或 executor；外层需先停止使用这些资源。
        持锁只保护封闭标志，生命周期调用仍须由外层顺序安排。

        Raises:
            RuntimeError: 节点明确返回 False 表示订阅删除失败。
            Exception: 节点销毁抛错，订阅句柄与封闭状态保留。
        """
        with self._lock:
            self._closed = True
        if self._subscription is not None:
            if self._node.destroy_subscription(self._subscription) is False:
                raise RuntimeError('failed to destroy odometry subscription')
            self._subscription = None
