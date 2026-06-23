#!/usr/bin/env python3
"""self_filter —— 抠掉机器人本体点云的 box 过滤(替代粗暴的径向 range_min)。

为什么不用 range_min：
  雷达在机身正前 0.352m 处，机身主要在它【后面】(后 0.65m、前仅 0.07m)。
  以雷达为心的 0.5m 圆会把正前方空地白白挡掉、又够不到机身后部。
  所以按 URDF 实测的本体包围盒做 box 过滤，只抠真正的机身。

本体盒(base_link 干净前向落地系，由 david.urdf + base_link.STL bbox 反算)：
  X[前] -0.30..+0.42   Y[侧] ±0.30   Z[高] 0..0.45    (轮子也在 ±0.30 内)
  留 ~0.03m 余量见下面默认参数。

实现：点云在 laser_link 系；base_link->laser_link 是纯平移(无旋转)，
  故 p_base = p_laser + sensor_xyz，逐点判是否落在本体盒内，在内则删。
  字段无关(用 PointField 动态拼 numpy dtype)，保留 intensity/ring/time 等所有列。
  仅适用于传感器相对 base 无旋转的安装(本包 base_link 即如此)。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

# PointField datatype -> numpy
_DT = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
       5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


class SelfFilter(Node):
    def __init__(self):
        super().__init__('self_filter')
        self.declare_parameter('input_topic', '/c16/lslidar_point_cloud')
        self.declare_parameter('output_topic', '/c16/points_self_filtered')
        # 雷达原点在 base_link 中的位置(纯平移)
        self.declare_parameter('sensor_xyz', [0.352, 0.0, 0.3775])
        # 本体盒(base_link 系，含 ~0.03m 余量)
        self.declare_parameter('min_x', -0.33)
        self.declare_parameter('max_x', 0.45)
        self.declare_parameter('min_y', -0.33)
        self.declare_parameter('max_y', 0.33)
        self.declare_parameter('min_z', -0.10)
        self.declare_parameter('max_z', 0.55)

        g = self.get_parameter
        self.sx, self.sy, self.sz = [float(v) for v in g('sensor_xyz').value]
        self.bx = (float(g('min_x').value), float(g('max_x').value))
        self.by = (float(g('min_y').value), float(g('max_y').value))
        self.bz = (float(g('min_z').value), float(g('max_z').value))
        out_topic = g('output_topic').value

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(PointCloud2, out_topic, qos)
        self.sub = self.create_subscription(
            PointCloud2, g('input_topic').value, self.cb, qos)
        self._dtype = None
        self._fields_sig = None
        self.get_logger().info(
            f"self_filter: 删本体盒 base_link X{self.bx} Y{self.by} Z{self.bz} "
            f"(雷达@{self.sx},{self.sy},{self.sz}) -> {out_topic}")

    def _build_dtype(self, msg):
        names, formats, offsets = [], [], []
        for f in msg.fields:
            if f.datatype not in _DT:
                continue
            names.append(f.name)
            formats.append(_DT[f.datatype])
            offsets.append(f.offset)
        self._dtype = np.dtype(
            {'names': names, 'formats': formats, 'offsets': offsets,
             'itemsize': msg.point_step})
        self._fields_sig = (msg.point_step, tuple((f.name, f.offset, f.datatype)
                                                  for f in msg.fields))

    def cb(self, msg: PointCloud2):
        sig = (msg.point_step, tuple((f.name, f.offset, f.datatype) for f in msg.fields))
        if sig != self._fields_sig:
            self._build_dtype(msg)

        arr = np.frombuffer(bytes(msg.data), dtype=self._dtype)
        if arr.size == 0:
            self.pub.publish(msg)
            return

        # laser 系 -> base_link 系(纯平移)
        bx = arr['x'].astype(np.float64) + self.sx
        by = arr['y'].astype(np.float64) + self.sy
        bz = arr['z'].astype(np.float64) + self.sz
        inside = ((bx >= self.bx[0]) & (bx <= self.bx[1]) &
                  (by >= self.by[0]) & (by <= self.by[1]) &
                  (bz >= self.bz[0]) & (bz <= self.bz[1]))
        # 删本体点 + 顺手删 NaN
        finite = np.isfinite(bx) & np.isfinite(by) & np.isfinite(bz)
        keep = (~inside) & finite
        out = arr[keep]

        msg.data = out.tobytes()
        msg.height = 1
        msg.width = int(out.size)
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
