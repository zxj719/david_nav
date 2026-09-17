"""把 ROS 栅格订阅和纯目标解析策略分离，不承担 Dock 轮廓准入."""

import threading

from nav_tasks.domain.errors import ClosedError, RuntimeAssemblyError
from nav_tasks.domain.grid import GridMapView


class RosGridSource:
    """持有两个深度十栅格订阅，不拥有节点、executor 或定位进程."""

    def __init__(
        self, node: object, *, map_topic: str, costmap_topic: str,
        occupancy_type: type | None = None,
    ) -> None:
        """沿用旧导航目标解析器的 QoS 订阅静态图和全局代价图.

        回调只保存最新消息引用，snapshot 才复制格值；不增加保留 QoS 或
        地图新鲜度策略。第二项创建失败时顺序回收已创建订阅。

        Args:
            node: 外部持有的 ROS 节点。
            map_topic: 配置的静态占用图话题。
            costmap_topic: 配置的全局代价图话题。
            occupancy_type: 可选测试类型，否则惰性导入 OccupancyGrid。

        Raises:
            RuntimeAssemblyError: 部分构造后的回滚也失败，附带可重试清理句柄。
            Exception: 类型导入或订阅创建失败，成功回滚后保留原异常。
        """
        if occupancy_type is None:
            from nav_msgs.msg import OccupancyGrid
            occupancy_type = OccupancyGrid
        self._node = node
        self._lock = threading.Lock()
        self._closed = False
        self._messages = [None, None]
        self._subscriptions = []
        try:
            for index, topic in enumerate((map_topic, costmap_topic)):
                subscription = node.create_subscription(
                    occupancy_type, topic,
                    lambda message, slot=index: self._receive(slot, message), 10)
                self._subscriptions.append(subscription)
        except Exception as error:
            try:
                self.close()
            except Exception:
                raise RuntimeAssemblyError(error, self._retry_close) from error
            raise

    async def _retry_close(self) -> None:
        self.close()

    def _receive(self, index: int, message: object) -> None:
        """持锁保存对应网格槽位的最新消息引用，关闭后的迟到回调被忽略.

        这里只保存引用，不复制或解析栅格；快照转换由 snapshot 负责。

        Args:
            index: 构造订阅时绑定的槽位，零为静态图、一为全局代价图。
            message: 当前收到的占用栅格消息。
        """
        with self._lock:
            if not self._closed:
                self._messages[index] = message

    def snapshot(self) -> tuple[GridMapView | None, GridMapView | None]:
        """持锁取得当前两张消息的引用，再按旧轴对齐规则复制栅格.

        两张消息不是时间同步的一对，解析在锁外完成；原消息之后的修改会影响
        下一次快照，但已生成的格值副本不随消息改变。GridMapView 本身仍可变。
        不解释原点旋转、不检查帧/时间，也不在此决定缺图是否放行。

        Returns:
            静态图和全局代价图的旧视图，任一可为 None；准入由调用方策略决定。

        Raises:
            ClosedError: 已开始关闭或已关闭。
            Exception: 消息字段或栅格转换失败。
        """
        with self._lock:
            if self._closed:
                raise ClosedError('grid source is closed')
            grid, costmap = self._messages
        return (GridMapView.from_occupancy_grid(grid),
                GridMapView.from_occupancy_grid(costmap))

    def close(self) -> None:
        """先封闭两个回调，再顺序移除自有订阅，失败时保留未完成尾部.

        成功移除的句柄不重复；首个失败中断后续销毁，外部节点不在这里销毁。
        封闭后回调不再读取消息，但本方法不为并发 close 调用提供生命周期串行化。

        Raises:
            RuntimeError: 节点明确返回 False 表示订阅未删除。
            Exception: 销毁抛错，失败项和未尝试项继续保留。
        """
        with self._lock:
            self._closed = True
        while self._subscriptions:
            subscription = self._subscriptions[0]
            if self._node.destroy_subscription(subscription) is False:
                raise RuntimeError('failed to destroy grid subscription')
            self._subscriptions.pop(0)
