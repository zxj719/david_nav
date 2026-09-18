"""提供 Dock 专用保留静态图快照与导航旋转中心的有界 TF 查询."""

import math
import threading
from typing import Callable

from nav_tasks.domain.errors import ClosedError
from nav_tasks.domain.footprint import FootprintCheck, OccupancyGridView
from nav_tasks.skills.docking.approach import check_approach_footprint
from nav_tasks.skills.docking.ports import DockLogger, PlanarPose


class DockObservations:
    """持有一个静态图订阅，借用运行时节点和 TF 缓冲区.

    与普通 POI 栅格源不同，使用可靠、瞬态本地、深度一 QoS，并在回调中
    复制完整不可变栅格及原点 yaw。不创建定位资源、里程计订阅或 executor。
    """

    def __init__(
        self, node: object, tf_buffer: object, *, global_frame: str,
        robot_base_frame: str, static_map_topic: str,
        footprint: tuple[tuple[float, float], ...], occupied_threshold: int,
        logger: DockLogger, pose_timeout_sec: float = 2.0, map_wait_sec: float = 1.0,
        occupancy_type: type | None = None, qos: object | None = None,
        latest_time: Callable[[], object] | None = None,
        duration: Callable[..., object] | None = None,
    ) -> None:
        """根据显式导航接线创建唯一自有地图订阅，不改变定位栈.

        未注入的消息、QoS、时间或 Duration 类型在构造时惰性导入。
        构造器不替代配置工厂的完整校验；外层须先停止并加入消费者线程再关闭。

        Args:
            node: 外部节点，存活至工作线程与本观察源收尾。
            tf_buffer: 外部缓冲区，须允许 Dock 工作线程读取。
            global_frame: TF 目标帧及地图准入要求的帧。
            robot_base_frame: 导航旋转中心，不是 POI 标记用的另一基座帧。
            static_map_topic: 静态占用地图来源，不是动态代价图。
            footprint: 配置提供的不可变导航轮廓，不在此复制。
            occupied_threshold: 含边界的占用阈值，未知格继续沿用允许策略。
            logger: 工作线程安全的 TF 失败日志接收器。
            pose_timeout_sec: TF 查询预算，默认两秒，传给 Duration。
            map_wait_sec: 首次静态地图的等待上限，默认一秒。只在快照仍为空时等待，且由 Dock
                工作线程调用，不会阻塞 ROS 回调线程；等不到仍按原策略拒绝。
            occupancy_type: 可选测试消息类型。
            qos: 可选测试 QoS；默认使用保留数据的深度一配置。
            latest_time: 可选最新时刻构造器。
            duration: 可选 ROS Duration 构造器。

        Raises:
            Exception: 类型导入、时间额度构造或订阅创建失败。
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
        self._map_wait = float(map_wait_sec)
        self._map_ready = threading.Event()
        self._closed = False
        self._grid: OccupancyGridView | None = None
        self._subscription = node.create_subscription(
            occupancy_type, static_map_topic, self._receive, qos)

    def _receive(self, message):
        """在锁外复制静态地图，再持锁复查关闭状态后替换当前快照.

        第一次检查避免关闭后无谓解析，第二次检查阻止解析期间发生关闭后再写入。
        此处只复制消息，不证明地图或预备点可用；准入检查在 check_approach 执行。

        Args:
            message: 含完整 OccupancyGrid 字段的静态地图消息。

        Raises:
            Exception: 消息字段缺失或快照数值转换失败，旧快照尚未被替换。
        """
        with self._lock:
            if self._closed:
                return
        view = OccupancyGridView.from_message(message)
        with self._lock:
            if not self._closed:
                self._grid = view
                self._map_ready.set()

    def pose(self) -> PlanarPose | None:
        """在阻塞工作线程查询最新导航旋转中心的平面位姿.

        保留旧四元数运算，不验证归一化、有限性或时间新鲜度。
        只捕获查询本身的普通异常；查询成功后的字段解析异常不转为 None。

        Returns:
            x/y/yaw，或查询失败且警告成功记录后的 None。

        Raises:
            ClosedError: 已开始关闭；工作线程本应先完成退出。
            Exception: 结果字段解析或失败日志接收器出错。
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
        """用同一份不可变静态图快照检查预备点的完整车身轮廓.

        取快照后不持锁做几何计算；无图、错帧、越界或占用按独立 Dock
        准入策略拒绝，未知格 -1 仍允许，不证明整片区域已知空闲。
        首次快照为空时在本线程内有界等待一次首图（默认一秒）：目标可能比锁存地图先到，
        等到即照常判定，等不到仍然拒绝。

        Args:
            pose: 配置全局帧内已计算出的预备位姿，单位为米与弧度。

        Returns:
            既有地图、帧、几何与轮廓占用的准入判定，不证明实车可达。

        Raises:
            ClosedError: 观察源正在关闭或已经关闭。
        """
        with self._lock:
            if self._closed:
                raise ClosedError('Dock observations are closed')
            grid = self._grid
        if grid is None:
            self._map_ready.wait(self._map_wait)
            with self._lock:
                if self._closed:
                    raise ClosedError('Dock observations are closed')
                grid = self._grid
        return check_approach_footprint(
            grid, self._global_frame, self._footprint, pose, self._threshold)

    def close(self) -> None:
        """封闭地图回调并销毁唯一自有订阅，失败句柄保留供重试.

        外层须先加入工作线程；这里不能中止已经在进行的 TF 查询。
        不销毁借用的节点、缓冲区或任何定位资源。

        Raises:
            RuntimeError: 节点明确返回 False 表示订阅未删除。
            Exception: 订阅销毁抛错，封闭状态不回退。
        """
        with self._lock:
            self._closed = True
        if self._subscription is not None:
            if self._node.destroy_subscription(self._subscription) is False:
                raise RuntimeError('failed to destroy Dock map subscription')
            self._subscription = None
