#!/usr/bin/env python3
r"""Record nav2 cmd_vel pipeline + /odom feedback for drift diagnosis.

Subscribes to every step of the cmd_vel chain plus encoder feedback, and
reports per-stage statistics so we can see where (if anywhere) a persistent
angular-velocity bias creeps in.

Pipeline (for the david/wheeltec stack):

    DWB ── /cmd_vel_nav ── velocity_smoother ── /cmd_vel_smoothed
        ── collision_monitor ── /cmd_vel_safe
        ── david_move ── /cmd_vel ── wheeltec_robot_node

Plus ``/odom`` from the wheel encoders.

Usage::

    source /opt/ros/humble/setup.bash
    source /home/agx/ros2_ws/install/setup.bash
    python3 /home/agx/ros2_ws/src/david_cmd/scripts/record_nav_drift.py \
        --duration 30 --label drift_run1

Run a Nav2 goal while the script is recording. After ``--duration`` seconds
the script writes::

    /tmp/nav_drift/<label>_summary.txt
    /tmp/nav_drift/<label>_<topic>.csv  (one per recorded topic)
"""

from __future__ import annotations

import argparse
import csv
import statistics
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


TWIST_TOPICS = [
    ("/cmd_vel_nav", "DWB raw output"),
    ("/cmd_vel_smoothed", "after velocity_smoother (CLOSED_LOOP)"),
    ("/cmd_vel_safe", "after collision_monitor"),
    ("/cmd_vel", "final to wheeltec (after david_move)"),
]


class NavDriftRecorder(Node):
    """Subscribe to every Twist in the chain + /odom; write CSVs on shutdown."""

    def __init__(self, output_dir: Path, label: str, motion_threshold: float):
        super().__init__("nav_drift_recorder")
        self.output_dir = output_dir
        self.label = label
        self.motion_threshold = motion_threshold
        self.start_mono = time.monotonic()

        # topic -> list[(t_rel, linear_x, linear_y, angular_z)]
        self.twist_records: dict[str, list[tuple[float, float, float, float]]] = {}
        self.twist_descriptions: dict[str, str] = {}
        for topic, desc in TWIST_TOPICS:
            self.twist_records[topic] = []
            self.twist_descriptions[topic] = desc
            self.create_subscription(
                Twist,
                topic,
                self._make_twist_cb(topic),
                QoSProfile(depth=200,
                           reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST),
            )

        # /odom -> list[(t_rel, linear_x, angular_z)]
        self.odom_records: list[tuple[float, float, float]] = []
        self.create_subscription(
            Odometry,
            "/odom",
            self._odom_cb,
            QoSProfile(depth=200,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST),
        )

    def _make_twist_cb(self, topic):
        def cb(msg: Twist):
            t = time.monotonic() - self.start_mono
            self.twist_records[topic].append(
                (t,
                 float(msg.linear.x),
                 float(msg.linear.y),
                 float(msg.angular.z))
            )
        return cb

    def _odom_cb(self, msg: Odometry):
        t = time.monotonic() - self.start_mono
        self.odom_records.append(
            (t,
             float(msg.twist.twist.linear.x),
             float(msg.twist.twist.angular.z))
        )

    # ------------------------------------------------------------------
    def _stats_active(self, values, threshold):
        """Mean / std / count restricted to samples where |linear_x| > threshold."""
        keep = [v for v in values if abs(v[1]) > threshold]
        if not keep:
            return None
        ang = [v[3] if len(v) == 4 else v[2] for v in keep]
        return {
            "count": len(keep),
            "ang_mean": sum(ang) / len(ang),
            "ang_std": statistics.pstdev(ang) if len(ang) > 1 else 0.0,
            "ang_min": min(ang),
            "ang_max": max(ang),
        }

    def write_outputs(self):
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # Per-topic CSVs.
        for topic, rows in self.twist_records.items():
            csv_path = self.output_dir / f"{self.label}_{topic.strip('/').replace('/', '_')}.csv"
            with csv_path.open("w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["t_rel", "linear_x", "linear_y", "angular_z"])
                for row in rows:
                    writer.writerow([f"{v:.6f}" for v in row])
        odom_path = self.output_dir / f"{self.label}_odom.csv"
        with odom_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["t_rel", "lin_x", "ang_z"])
            for row in self.odom_records:
                writer.writerow([f"{v:.6f}" for v in row])

        # Summary text.
        summary_path = self.output_dir / f"{self.label}_summary.txt"
        lines = []
        lines.append(f"nav drift recording label={self.label}")
        lines.append(f"motion_threshold={self.motion_threshold} m/s "
                     f"(stats only on samples with |linear_x| > threshold)")
        lines.append("")
        lines.append(f"{'topic':<22} {'desc':<40} {'n_total':>8} "
                     f"{'n_active':>8} {'ang_mean':>11} {'ang_std':>10} "
                     f"{'ang_min':>10} {'ang_max':>10}")
        lines.append("-" * 130)

        for topic in [t for t, _ in TWIST_TOPICS]:
            rows = self.twist_records[topic]
            desc = self.twist_descriptions[topic]
            s = self._stats_active(rows, self.motion_threshold)
            if s is None:
                lines.append(f"{topic:<22} {desc:<40} {len(rows):>8d} "
                             f"{'(none active)':>40}")
            else:
                lines.append(f"{topic:<22} {desc:<40} {len(rows):>8d} "
                             f"{s['count']:>8d} {s['ang_mean']:>+11.5f} "
                             f"{s['ang_std']:>10.5f} "
                             f"{s['ang_min']:>+10.5f} {s['ang_max']:>+10.5f}")

        # odom is (t, lin_x, ang_z), 3 fields — pad to 4 for _stats_active.
        odom_padded = [(t, lx, 0.0, az) for (t, lx, az) in self.odom_records]
        s = self._stats_active(odom_padded, self.motion_threshold)
        if s is None:
            lines.append(f"{'/odom':<22} {'wheel encoder feedback':<40} "
                         f"{len(self.odom_records):>8d} {'(none active)':>40}")
        else:
            lines.append(f"{'/odom':<22} {'wheel encoder feedback':<40} "
                         f"{len(self.odom_records):>8d} "
                         f"{s['count']:>8d} {s['ang_mean']:>+11.5f} "
                         f"{s['ang_std']:>10.5f} "
                         f"{s['ang_min']:>+10.5f} {s['ang_max']:>+10.5f}")

        lines.append("")
        lines.append("Interpretation:")
        lines.append("- ang_mean ≈ 0 on /cmd_vel_nav with non-trivial std")
        lines.append("    => DWB is bouncing left/right, not biased. Probably a")
        lines.append("       smoother / odom feedback issue downstream.")
        lines.append("- ang_mean significantly < 0 (e.g. -0.05) on /cmd_vel_nav")
        lines.append("    => DWB is consistently turning right. Robot drifts left")
        lines.append("       because actual motion lags command (anti-windup-like).")
        lines.append("- ang_mean ≈ 0 on /cmd_vel_nav but > 0 (or < 0) on")
        lines.append("    /cmd_vel_smoothed => CLOSED_LOOP smoother is adding bias.")
        lines.append("- /odom ang_z ang_mean significantly different from /cmd_vel")
        lines.append("    ang_mean => physical drift not reflected in command;")
        lines.append("    likely wheel calibration (but teleop ruled this out).")
        lines.append("")
        lines.append(f"CSV files: {self.output_dir}/{self.label}_*.csv")

        summary_path.write_text("\n".join(lines) + "\n")
        # Also print to stdout for convenience.
        for line in lines:
            print(line)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0,
                        help="Recording duration in seconds (default 30)")
    parser.add_argument("--label", default="run",
                        help="Label prepended to output files")
    parser.add_argument("--output-dir", default="/tmp/nav_drift",
                        help="Output directory (default /tmp/nav_drift)")
    parser.add_argument("--motion-threshold", type=float, default=0.02,
                        help=("|linear_x| > threshold to count a sample as 'active' "
                              "(default 0.02 m/s)"))
    args = parser.parse_args()

    rclpy.init()
    node = NavDriftRecorder(Path(args.output_dir), args.label, args.motion_threshold)
    print(f"=== nav drift recording for {args.duration}s as label='{args.label}' ===")
    print("Start your Nav2 goal NOW. Recording...")
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        print("\nInterrupted, writing partial output.")
    finally:
        node.write_outputs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
