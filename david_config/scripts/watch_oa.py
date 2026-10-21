"""实时数地图里没有、激光却看到的障碍格，用来当场验动态避障.

用法：起 3 + 21 之后直接跑，然后拿箱子/人走到车前 1.5 m 以内，看 新增 那一列跳。
Ctrl-C 退出。
"""
import math
import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
import tf2_ros

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)


def main():
    """跑一次检查并打印结论.

    Returns:
        0 通过，1 未通过，2 数据不全。
    """
    rclpy.init()
    node = rclpy.create_node('watch_oa')
    seen = {}
    node.create_subscription(OccupancyGrid, '/map', lambda m: seen.setdefault('map', m), LATCHED)
    node.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                             lambda m: seen.__setitem__('grid', m), LATCHED)
    node.create_subscription(OccupancyGridUpdate, '/local_costmap/costmap_updates',
                             lambda m: seen.__setitem__('update', m), 10)
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)
    deadline = node.get_clock().now().nanoseconds + 20e9
    while node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if len(seen) >= 3 and buffer.can_transform('map', 'odom', rclpy.time.Time()):
            break
    if len(seen) < 3:
        print('收不到：', sorted({'map', 'grid', 'update'} - set(seen)))
        return 2
    static, grid = seen['map'], seen['grid']
    width, height, res = grid.info.width, grid.info.height, grid.info.resolution
    print('局部代价图 %dx%d，只覆盖机器人周围 ±%.1f m —— 障碍要走进这个范围才算数'
          % (width, height, width * res / 2))
    print('%-10s %8s %8s %8s   %s' % ('时间', '新障碍格', '新增', '消失', '最近的一个(相对车,米)'))
    previous = set()
    while rclpy.ok():
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.2)
        update = seen.get('update')
        cells = (list(update.data)
                 if update and (update.width, update.height) == (width, height)
                 else list(grid.data))
        try:
            link = buffer.lookup_transform('map', grid.header.frame_id, rclpy.time.Time(),
                                           Duration(seconds=1.0)).transform
        except tf2_ros.TransformException:
            continue
        rot, tra = link.rotation, link.translation
        yaw = math.atan2(2 * (rot.w * rot.z + rot.x * rot.y),
                         1 - 2 * (rot.y * rot.y + rot.z * rot.z))
        fresh = set()
        nearest = None
        for row in range(height):
            for col in range(width):
                if cells[row * width + col] != 100:
                    continue
                ox = grid.info.origin.position.x + (col + 0.5) * res
                oy = grid.info.origin.position.y + (row + 0.5) * res
                wx = tra.x + ox * math.cos(yaw) - oy * math.sin(yaw)
                wy = tra.y + ox * math.sin(yaw) + oy * math.cos(yaw)
                mx = int((wx - static.info.origin.position.x) / static.info.resolution)
                my = int((wy - static.info.origin.position.y) / static.info.resolution)
                inside = 0 <= mx < static.info.width and 0 <= my < static.info.height
                if inside and 0 <= static.data[my * static.info.width + mx] < 50:
                    fresh.add((col, row))
                    span = math.hypot(ox, oy)
                    if nearest is None or span < nearest[0]:
                        nearest = (span, ox, oy)
        stamp = node.get_clock().now().nanoseconds * 1e-9
        where = ('%.2f m @ (%+.2f, %+.2f)' % nearest) if nearest else '-'
        print('%-10s %8d %8d %8d   %s'
              % (time_text(stamp), len(fresh), len(fresh - previous),
                 len(previous - fresh), where), flush=True)
        previous = fresh
    return 0


def time_text(stamp):
    """把 Unix 秒格式化成时分秒.

    Args:
        stamp: Unix 秒。

    Returns:
        HH:MM:SS 文本。
    """
    import datetime
    return datetime.datetime.fromtimestamp(stamp).strftime('%H:%M:%S')


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        pass
