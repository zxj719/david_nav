"""判断点云有没有真的进局部代价图，起 3 + 21 之后跑.

判据一（确定性）：obstacle_layer 的高度门限必须容得下扫描所在的接地平面。
  该门限在局部代价图的 global_frame(odom) 下解读，而 odom 通常锚在机身帧上，
  接地平面的 z 是负的；nav2 的每源默认下沿是 0.0，会静默丢掉全部点。
判据二（经验性）：量程内的扫描端点所在格应当被标记。落在代价 0 上的端点越多，
  越说明标记没发生。注意"端点落在已标记格上"本身不算证据——静态地图里的墙
  本来就被标记，激光当然也打在那些墙上。
"""
import collections
import math
import rclpy
from rclpy.duration import Duration
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf2_ros

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
COSTMAP = '/local_costmap/local_costmap'
GATE = ('obstacle_layer.scan.min_obstacle_height', 'obstacle_layer.scan.max_obstacle_height')
UNMARKED_BUDGET = 0.10


def _gate(node):
    """读运行中代价图的每源高度门限，读不到就返回 None（不猜默认值）.

    Args:
        node: 已初始化的只读节点。

    Returns:
        (下沿, 上沿) 两个浮点数；服务不在或超时返回 None。
    """
    client = node.create_client(GetParameters, COSTMAP + '/get_parameters')
    if not client.wait_for_service(timeout_sec=10.0):
        return None
    future = client.call_async(GetParameters.Request(names=list(GATE)))
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if not future.done() or len(future.result().values) != len(GATE):
        return None
    return [value.double_value for value in future.result().values]


def main():
    """跑一次检查并打印结论.

    Returns:
        0 通过，1 未通过，2 数据不全。
    """
    rclpy.init()
    node = rclpy.create_node('check_oa')
    seen = {}
    node.create_subscription(OccupancyGrid, COSTMAP.rsplit('/', 1)[0] + '/costmap',
                             lambda m: seen.__setitem__('grid', m), LATCHED)
    node.create_subscription(OccupancyGridUpdate, COSTMAP.rsplit('/', 1)[0] + '/costmap_updates',
                             lambda m: seen.__setitem__('update', m), 10)
    node.create_subscription(LaserScan, '/scan',
                             lambda m: seen.__setitem__('scan', m), qos_profile_sensor_data)
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)
    deadline = node.get_clock().now().nanoseconds + 20e9
    while node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if len(seen) >= 3 and buffer.can_transform('odom', 'base_footprint', rclpy.time.Time()):
            break
    missing = {'grid', 'update', 'scan'} - set(seen)
    if missing:
        print('收不到：', sorted(missing), '—— 栈没起全，先看 21 的终端')
        return 2
    grid, update, scan = seen['grid'], seen['update'], seen['scan']
    width, height, res = grid.info.width, grid.info.height, grid.info.resolution
    matched = (update.width, update.height) == (width, height)
    cells = list(update.data) if matched else list(grid.data)
    ground = buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time(),
                                     Duration(seconds=3.0)).transform.translation.z
    finite = [r for r in scan.ranges if math.isfinite(r)]
    age = node.get_clock().now().nanoseconds * 1e-9 - (
        scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9)
    print('接地帧在 odom 里的高度 z = %+.3f m  （扫描点进 obstacle_layer 时就是这个 z）' % ground)
    print('/scan  有效束 %d/%d  量程 %.2f-%.2f m  时延 %.2f s'
          % (len(finite), len(scan.ranges), min(finite), max(finite), age))

    gate = _gate(node)
    gate_ok = None
    if gate is None:
        print('高度门限：读不到参数（节点没起或参数服务超时）')
    else:
        gate_ok = gate[0] <= ground <= gate[1]
        print('高度门限：min %.2f <= z %+.3f <= max %.2f  ->  %s'
              % (gate[0], ground, gate[1], '容得下' if gate_ok else '把全部点挡掉了'))

    pose = buffer.lookup_transform('odom', scan.header.frame_id, rclpy.time.Time(),
                                   Duration(seconds=3.0)).transform
    rot, tra = pose.rotation, pose.translation
    yaw = math.atan2(2 * (rot.w * rot.z + rot.x * rot.y),
                     1 - 2 * (rot.y * rot.y + rot.z * rot.z))
    tally = collections.Counter()
    for index, reach in enumerate(scan.ranges):
        if not math.isfinite(reach):
            continue
        angle = scan.angle_min + index * scan.angle_increment
        bx, by = reach * math.cos(angle), reach * math.sin(angle)
        ox = tra.x + bx * math.cos(yaw) - by * math.sin(yaw)
        oy = tra.y + bx * math.sin(yaw) + by * math.cos(yaw)
        col = int((ox - grid.info.origin.position.x) / res)
        row = int((oy - grid.info.origin.position.y) / res)
        if not (0 <= col < width and 0 <= row < height):
            tally['窗口外(被裁掉)'] += 1
            continue
        value = cells[row * width + col]
        tally['已标记' if value != 0 else '未标记(代价0)'] += 1
    inside = tally['已标记'] + tally['未标记(代价0)']
    print('扫描端点：窗口内 %d（已标记 %d / 代价0 %d），窗口外被裁掉 %d'
          % (inside, tally['已标记'], tally['未标记(代价0)'], tally['窗口外(被裁掉)']))

    unmarked = tally['未标记(代价0)'] / inside if inside else 1.0
    if gate_ok is False:
        print('\n结论：未通过 —— 高度门限把扫描全挡在代价图之外。')
        print('      修法：在 <robot>_config/modes/overlays/oa.yaml 里把'
              ' obstacle_layer/scan/min_obstacle_height 放到 %+.2f 以下。' % ground)
        return 1
    if unmarked > UNMARKED_BUDGET:
        print('\n结论：未通过 —— 窗口内有 %.0f%% 的端点落在代价 0 的格上（容许 %.0f%%）。'
              % (unmarked * 100, UNMARKED_BUDGET * 100))
        return 1
    print('\n结论：通过 —— 高度门限容得下接地平面，且窗口内 %.0f%% 的端点所在格已被标记。'
          % ((1 - unmarked) * 100))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
