"""ROS occupancy subscriptions separated from pure goal-resolution policy."""

import threading

from nav_tasks.domain.errors import ClosedError, RuntimeAssemblyError
from nav_tasks.domain.grid import GridMapView


class RosGridSource:
    """Own two depth-10 subscriptions, without a ROS node or executor of its own."""

    def __init__(
        self, node: object, *, map_topic: str, costmap_topic: str,
        occupancy_type: type | None = None,
    ) -> None:
        """Subscribe using the same QoS as the old navigation goal resolver.

        Args:
            node: Externally owned ROS node.
            map_topic: Configured static occupancy topic.
            costmap_topic: Configured global costmap topic.
            occupancy_type: Optional fake type, otherwise lazily import OccupancyGrid.

        Raises:
            RuntimeAssemblyError: Partial creation cleanup failed; retain and retry its handle.
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
        with self._lock:
            if not self._closed:
                self._messages[index] = message

    def snapshot(self) -> tuple[GridMapView | None, GridMapView | None]:
        """Capture the current pair and apply the original axis-aligned conversion.

        No missing-map safety policy or map-origin rotation is added here; pure
        SnapSearch and the separate dock footprint checker retain their policies.

        Returns:
            Static grid and global costmap views, either possibly absent.

        Raises:
            ClosedError: The adapter is closing or closed.
        """
        with self._lock:
            if self._closed:
                raise ClosedError('grid source is closed')
            grid, costmap = self._messages
        return (GridMapView.from_occupancy_grid(grid),
                GridMapView.from_occupancy_grid(costmap))

    def close(self) -> None:
        """Seal callbacks and remove owned subscriptions, retaining failed cleanup.

        Raises:
            RuntimeError: The node reports a failed subscription removal.
        """
        with self._lock:
            self._closed = True
        while self._subscriptions:
            subscription = self._subscriptions[0]
            if self._node.destroy_subscription(subscription) is False:
                raise RuntimeError('failed to destroy grid subscription')
            self._subscriptions.pop(0)
