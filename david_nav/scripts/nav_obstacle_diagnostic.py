#!/usr/bin/env python3
"""
Live diagnostic for david_nav navigation stack.

Prints (default 1 Hz) a single line capturing the three signals that decide
whether DWB will reject every trajectory:

  - /local_costmap/published_footprint -> count of LETHAL cells inside
    footprint polygon + nearest lethal cell distance/angle (in costmap frame).
    Counterpart of david_nav's RobotEmbeddedInObstacle BT node, computed in
    Python so it requires no rebuild.
  - /scan -> min range, near-distance return count, sector distribution
    (Front/Left/Right/Back). Catches ground-return rings injected into /scan.
  - /follow_path/_action/status -> latest controller_server action status and
    a short rolling history so we can correlate "Aborting handle" with the
    above.

Run:
    python3 src/david_nav/scripts/nav_obstacle_diagnostic.py

Options (all optional):
    --scan-topic /scan
    --costmap-topic /local_costmap/costmap
    --footprint-topic /local_costmap/published_footprint
    --action-status /follow_path/_action/status
    --period 1.0
    --near 1.0
"""

import argparse
import math
import sys
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan


LETHAL_OCCUPANCYGRID = 100  # nav2 downsamples 254 -> 100 on the OccupancyGrid topic
INSCRIBED_OCCUPANCYGRID = 99

STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNK",
    GoalStatus.STATUS_ACCEPTED: "ACC",
    GoalStatus.STATUS_EXECUTING: "EXE",
    GoalStatus.STATUS_CANCELING: "CXL",
    GoalStatus.STATUS_SUCCEEDED: "OK",
    GoalStatus.STATUS_CANCELED: "CNC",
    GoalStatus.STATUS_ABORTED: "ABRT",
}


def point_in_polygon(px: np.ndarray, py: np.ndarray,
                     vx: np.ndarray, vy: np.ndarray) -> np.ndarray:
    """Vectorised even-odd ray-cast point-in-polygon test."""
    n = vx.shape[0]
    inside = np.zeros(px.shape, dtype=bool)
    j = n - 1
    eps = 1e-12
    for i in range(n):
        cond = (vy[i] > py) != (vy[j] > py)
        denom = vy[j] - vy[i]
        safe_denom = np.where(np.abs(denom) < eps, eps, denom)
        x_at_py = (vx[j] - vx[i]) * (py - vy[i]) / safe_denom + vx[i]
        inside ^= cond & (px < x_at_py)
        j = i
    return inside


@dataclass
class ScanSnap:
    stamp_sec: float
    min_range: float
    min_angle: float
    near_count: int
    sectors: dict


@dataclass
class CostmapSnap:
    stamp_sec: float
    frame_id: str
    lethal_in_footprint: int
    inscribed_in_footprint: int
    nearest_lethal_m: float
    nearest_lethal_angle: float
    footprint_frame: str
    footprint_present: bool


@dataclass
class StatusEntry:
    when: float
    status: int


class Diagnostic(Node):
    def __init__(self, args):
        super().__init__("nav_obstacle_diagnostic")
        self.args = args

        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        default_qos = QoSProfile(depth=10)

        self.create_subscription(LaserScan, args.scan_topic, self.on_scan, sensor_qos)
        self.create_subscription(OccupancyGrid, args.costmap_topic, self.on_costmap, latched_qos)
        self.create_subscription(PolygonStamped, args.footprint_topic, self.on_footprint, default_qos)
        self.create_subscription(GoalStatusArray, args.action_status, self.on_status, default_qos)

        self.latest_footprint: Optional[PolygonStamped] = None
        self.scan_snap: Optional[ScanSnap] = None
        self.costmap_snap: Optional[CostmapSnap] = None
        self.status_history: Deque[StatusEntry] = deque(maxlen=10)
        self.frame_mismatch_logged = False

        self.start_time = self._now()
        self.create_timer(args.period, self.print_status)

        self.get_logger().info(
            f"nav_obstacle_diagnostic started (period={args.period}s, near={args.near}m)\n"
            f"  scan={args.scan_topic}  costmap={args.costmap_topic}\n"
            f"  footprint={args.footprint_topic}  status={args.action_status}"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------- subscribers ----------

    def on_footprint(self, msg: PolygonStamped) -> None:
        self.latest_footprint = msg

    def on_scan(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if not valid.any():
            self.scan_snap = ScanSnap(stamp, float("inf"), 0.0, 0,
                                      {"F": 0, "L": 0, "R": 0, "B": 0})
            return

        idx = np.where(valid)[0]
        angles = msg.angle_min + idx.astype(np.float32) * msg.angle_increment
        rs = ranges[idx]

        near_mask = rs < self.args.near
        near_angles = angles[near_mask]

        sectors = {"F": 0, "L": 0, "R": 0, "B": 0}
        if near_angles.size:
            abs_a = np.abs(near_angles)
            front = abs_a < math.pi / 4
            back = abs_a >= 3 * math.pi / 4
            left = (~front) & (~back) & (near_angles > 0)
            right = (~front) & (~back) & (near_angles < 0)
            sectors["F"] = int(front.sum())
            sectors["L"] = int(left.sum())
            sectors["R"] = int(right.sum())
            sectors["B"] = int(back.sum())

        k = int(np.argmin(rs))
        self.scan_snap = ScanSnap(
            stamp_sec=stamp,
            min_range=float(rs[k]),
            min_angle=float(angles[k]),
            near_count=int(near_mask.sum()),
            sectors=sectors,
        )

    def on_costmap(self, msg: OccupancyGrid) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        fp = self.latest_footprint
        if fp is None or len(fp.polygon.points) < 3:
            self.costmap_snap = CostmapSnap(
                stamp, msg.header.frame_id, 0, 0,
                float("nan"), float("nan"), "", False,
            )
            return

        if fp.header.frame_id and fp.header.frame_id != msg.header.frame_id and not self.frame_mismatch_logged:
            self.get_logger().warn(
                f"footprint frame={fp.header.frame_id!r} != costmap frame={msg.header.frame_id!r} "
                f"(diagnostic assumes same frame, no TF lookup performed)"
            )
            self.frame_mismatch_logged = True

        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        res = msg.info.resolution
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0 or res <= 0.0:
            return

        try:
            data = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
        except ValueError:
            return

        vx = np.array([p.x for p in fp.polygon.points], dtype=np.float64)
        vy = np.array([p.y for p in fp.polygon.points], dtype=np.float64)

        i0 = max(0, int(math.floor((vx.min() - ox) / res)))
        i1 = min(w - 1, int(math.floor((vx.max() - ox) / res)))
        j0 = max(0, int(math.floor((vy.min() - oy) / res)))
        j1 = min(h - 1, int(math.floor((vy.max() - oy) / res)))

        lethal_in = 0
        inscribed_in = 0
        if i1 >= i0 and j1 >= j0:
            sub = data[j0:j1 + 1, i0:i1 + 1]
            ix, iy = np.meshgrid(
                np.arange(i0, i1 + 1, dtype=np.float64),
                np.arange(j0, j1 + 1, dtype=np.float64),
            )
            cx = ox + (ix + 0.5) * res
            cy = oy + (iy + 0.5) * res
            inside = point_in_polygon(cx.ravel(), cy.ravel(), vx, vy).reshape(cx.shape)
            lethal_in = int(((sub >= LETHAL_OCCUPANCYGRID) & inside).sum())
            inscribed_in = int(
                ((sub >= INSCRIBED_OCCUPANCYGRID) & (sub < LETHAL_OCCUPANCYGRID) & inside).sum()
            )

        cent_x = float(vx.mean())
        cent_y = float(vy.mean())
        lethal_mask = data >= LETHAL_OCCUPANCYGRID
        if lethal_mask.any():
            jys, ixs = np.where(lethal_mask)
            wxs = ox + (ixs.astype(np.float64) + 0.5) * res
            wys = oy + (jys.astype(np.float64) + 0.5) * res
            d = np.hypot(wxs - cent_x, wys - cent_y)
            k = int(np.argmin(d))
            nearest = float(d[k])
            nearest_angle = math.atan2(float(wys[k] - cent_y), float(wxs[k] - cent_x))
        else:
            nearest = float("nan")
            nearest_angle = float("nan")

        self.costmap_snap = CostmapSnap(
            stamp_sec=stamp,
            frame_id=msg.header.frame_id,
            lethal_in_footprint=lethal_in,
            inscribed_in_footprint=inscribed_in,
            nearest_lethal_m=nearest,
            nearest_lethal_angle=nearest_angle,
            footprint_frame=fp.header.frame_id,
            footprint_present=True,
        )

    def on_status(self, msg: GoalStatusArray) -> None:
        now = self._now()
        for s in msg.status_list:
            if self.status_history and self.status_history[-1].status == s.status:
                # Same status reasserted within one array; skip the duplicate.
                continue
            self.status_history.append(StatusEntry(when=now, status=s.status))

    # ---------- printer ----------

    def print_status(self) -> None:
        now = self._now()
        t = now - self.start_time

        # /scan summary
        if self.scan_snap is None:
            scan_str = "(no /scan yet)"
        else:
            s = self.scan_snap
            age_ms = (now - s.stamp_sec) * 1000.0
            min_str = "inf" if math.isinf(s.min_range) else f"{s.min_range:.2f}m@{s.min_angle:+.2f}"
            scan_str = (
                f"min={min_str} near<{self.args.near:.1f}m={s.near_count} "
                f"F/L/R/B={s.sectors['F']}/{s.sectors['L']}/{s.sectors['R']}/{s.sectors['B']} "
                f"age={age_ms:.0f}ms"
            )

        # footprint / costmap summary
        if self.costmap_snap is None:
            cm_str = "(no local_costmap yet)"
        elif not self.costmap_snap.footprint_present:
            cm_str = f"(no footprint yet, costmap frame={self.costmap_snap.frame_id})"
        else:
            c = self.costmap_snap
            nearest = "nan" if math.isnan(c.nearest_lethal_m) else f"{c.nearest_lethal_m:.2f}m@{c.nearest_lethal_angle:+.2f}"
            cm_str = (
                f"lethal_in_fp={c.lethal_in_footprint} inscribed_in_fp={c.inscribed_in_footprint} "
                f"nearest_lethal={nearest} (frames cm={c.frame_id} fp={c.footprint_frame})"
            )

        # action status
        if not self.status_history:
            st_str = "(no follow_path status yet)"
        else:
            last = self.status_history[-1]
            age = now - last.when
            recent = ",".join(
                STATUS_NAMES.get(e.status, str(e.status))
                for e in list(self.status_history)[-6:]
            )
            st_str = f"last={STATUS_NAMES.get(last.status, str(last.status))}({age:.1f}s ago) recent=[{recent}]"

        self.get_logger().info(
            f"[diag t={t:6.1f}s]\n"
            f"  footprint: {cm_str}\n"
            f"  /scan:     {scan_str}\n"
            f"  follow_path: {st_str}"
        )


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scan-topic", default="/scan")
    p.add_argument("--costmap-topic", default="/local_costmap/costmap")
    p.add_argument("--footprint-topic", default="/local_costmap/published_footprint")
    p.add_argument("--action-status", default="/follow_path/_action/status")
    p.add_argument("--period", type=float, default=1.0, help="print period in seconds")
    p.add_argument("--near", type=float, default=1.0, help="/scan 'near' threshold in metres")
    return p.parse_args(argv)


def main():
    args = parse_args(sys.argv[1:])
    rclpy.init()
    node = Diagnostic(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
