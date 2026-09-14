"""Velocity-loop step + frequency-sweep evaluation using wheel encoders.

The orchestrator subscribes to ``/odom`` (Wheeltec encoder-derived twist) and
publishes square-wave angular velocity commands to a configurable target
topic. It records every odom sample plus every publish instant and feeds
them to the pure metric helpers in this module to produce start/stop latency
and steady-state statistics per segment.

The pure helpers are intentionally framework-free so they can be unit tested
without ROS.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence


@dataclass
class Sample:
    """One ``/odom`` reading captured by the orchestrator."""

    t_mono: float
    odom_lin_x: float
    odom_ang_z: float


@dataclass
class SteadyState:
    """Mean / standard deviation / sample count of a steady-state window."""

    mean: Optional[float]
    std: Optional[float]
    count: int


@dataclass
class SegmentSummary:
    """One segment's metrics in a form that's easy to print or serialise."""

    name: str
    target_w: float
    publish_hz: float
    start_latency: Optional[float]
    stop_latency: Optional[float]
    ss_mean: Optional[float]
    ss_std: Optional[float]
    ss_error: Optional[float]
    ss_count: int
    n_samples: int
    threshold: float


def _signal_value(sample: Sample, axis: str) -> float:
    """Pick the odom signal corresponding to the test axis.

    ``angular`` reads ``odom_ang_z`` (rad/s); ``linear`` reads ``odom_lin_x``
    (m/s). Unknown axis falls back to angular for back-compat with callers
    that omit the argument.
    """
    if axis == "linear":
        return float(sample.odom_lin_x)
    return float(sample.odom_ang_z)


def compute_start_latency(
    samples: Sequence[Sample],
    t_step_on: float,
    threshold: float,
    axis: str = "angular",
) -> Optional[float]:
    """Time from step-on to first odom |signal| >= threshold on ``axis``.

    Only samples whose monotonic timestamp is at or after ``t_step_on`` are
    considered. Returns ``None`` if no sample crosses the threshold.
    """
    threshold = abs(float(threshold))
    for sample in samples:
        if sample.t_mono < t_step_on:
            continue
        if abs(_signal_value(sample, axis)) >= threshold:
            return sample.t_mono - t_step_on
    return None


def compute_stop_latency(
    samples: Sequence[Sample],
    t_step_off: float,
    threshold: float,
    axis: str = "angular",
) -> Optional[float]:
    """Time from step-off to first odom |signal| < threshold on ``axis``.

    Only samples whose monotonic timestamp is at or after ``t_step_off`` are
    considered. Returns ``None`` if the magnitude never falls below the
    threshold within the captured samples.
    """
    threshold = abs(float(threshold))
    for sample in samples:
        if sample.t_mono < t_step_off:
            continue
        if abs(_signal_value(sample, axis)) < threshold:
            return sample.t_mono - t_step_off
    return None


def compute_steady_state(
    samples: Sequence[Sample],
    t_from: float,
    t_to: float,
    axis: str = "angular",
) -> SteadyState:
    """Mean and population std of odom signal on ``axis`` in [t_from, t_to]."""
    window = [
        _signal_value(sample, axis)
        for sample in samples
        if t_from <= sample.t_mono <= t_to
    ]
    if not window:
        return SteadyState(mean=None, std=None, count=0)
    mean = sum(window) / len(window)
    std = math.sqrt(sum((x - mean) ** 2 for x in window) / len(window))
    return SteadyState(mean=mean, std=std, count=len(window))


@dataclass
class RepeatabilityStats:
    """Cross-run statistics for repeated segments at the same (target_w, hz)."""

    target_w: float
    publish_hz: float
    n: int
    n_responded: int
    response_rate: float
    ss_mean_avg: Optional[float]
    ss_mean_std: Optional[float]
    start_lat_avg: Optional[float]
    start_lat_std: Optional[float]
    stop_lat_avg: Optional[float]
    stop_lat_std: Optional[float]


def response_classification(
    summary: "SegmentSummary",
    min_ss_fraction: float = 0.3,
) -> bool:
    """Decide whether a segment counts as "the robot responded".

    Response is acknowledged when:
      - ``start_latency`` was detected (transient motion seen), unless
        ``ss_mean`` is significantly the **wrong** sign (motor went the
        opposite way at steady state — counted as failure), OR
      - ``start_latency`` is None but ``ss_mean`` magnitude reaches at least
        ``min_ss_fraction * |target_w|`` with the correct sign.

    Wrong-sign responses (e.g. commanded -0.5 but encoder reports +0.3 at
    steady state) count as **non-responses** because the velocity loop did
    not deliver the requested direction.
    """
    target = float(summary.target_w)
    if target == 0.0:
        return False

    ss = summary.ss_mean

    if ss is not None:
        wrong_sign_significant = (
            (ss * target) < 0.0
            and abs(ss) >= abs(target) * float(min_ss_fraction)
        )
        if wrong_sign_significant:
            return False

    if summary.start_latency is not None:
        return True

    if ss is None:
        return False

    return (ss * target) > 0.0 and abs(ss) >= abs(target) * float(min_ss_fraction)


def _mean_std(values: Sequence[float]) -> tuple[Optional[float], Optional[float]]:
    if not values:
        return None, None
    n = len(values)
    mean = sum(values) / n
    std = math.sqrt(sum((x - mean) ** 2 for x in values) / n)
    return mean, std


def aggregate_repetitions(
    summaries: Sequence["SegmentSummary"],
    min_ss_fraction: float = 0.3,
) -> RepeatabilityStats:
    """Aggregate N segments with identical (target_w, publish_hz)."""
    if not summaries:
        raise ValueError("aggregate_repetitions requires at least one summary")

    targets = {round(s.target_w, 6) for s in summaries}
    rates = {round(s.publish_hz, 3) for s in summaries}
    if len(targets) != 1 or len(rates) != 1:
        raise ValueError(
            f"summaries must share target_w and publish_hz; "
            f"got targets={targets}, rates={rates}"
        )

    target_w = summaries[0].target_w
    publish_hz = summaries[0].publish_hz
    n = len(summaries)

    # ss_mean across all runs: None → 0 to capture "no motion" honestly.
    ss_vals = [(s.ss_mean if s.ss_mean is not None else 0.0) for s in summaries]
    ss_avg, ss_std = _mean_std(ss_vals)

    responded = [s for s in summaries if response_classification(s, min_ss_fraction)]
    n_responded = len(responded)

    start_lats = [s.start_latency for s in responded if s.start_latency is not None]
    stop_lats = [s.stop_latency for s in responded if s.stop_latency is not None]
    start_avg, start_std = _mean_std(start_lats)
    stop_avg, stop_std = _mean_std(stop_lats)

    return RepeatabilityStats(
        target_w=target_w,
        publish_hz=publish_hz,
        n=n,
        n_responded=n_responded,
        response_rate=n_responded / n,
        ss_mean_avg=ss_avg,
        ss_mean_std=ss_std,
        start_lat_avg=start_avg,
        start_lat_std=start_std,
        stop_lat_avg=stop_avg,
        stop_lat_std=stop_std,
    )


def summarize_segment(
    samples: Sequence[Sample],
    target_w: float,
    publish_hz: float,
    t_step_on: float,
    t_step_off: float,
    threshold: float,
    ss_window: float,
    name: str = "",
    axis: str = "angular",
) -> SegmentSummary:
    """Combine latency and steady-state metrics for one motion segment.

    The steady-state window is the last ``ss_window`` seconds before
    ``t_step_off``. ``axis`` selects which odom signal to read: ``angular``
    (default, rad/s) or ``linear`` (m/s on x).
    """
    ss = compute_steady_state(
        samples,
        t_from=t_step_off - ss_window,
        t_to=t_step_off,
        axis=axis,
    )
    ss_error = None if ss.mean is None else ss.mean - target_w
    return SegmentSummary(
        name=name,
        target_w=target_w,
        publish_hz=publish_hz,
        start_latency=compute_start_latency(samples, t_step_on, threshold, axis=axis),
        stop_latency=compute_stop_latency(samples, t_step_off, threshold, axis=axis),
        ss_mean=ss.mean,
        ss_std=ss.std,
        ss_error=ss_error,
        ss_count=ss.count,
        n_samples=len(samples),
        threshold=threshold,
    )


# ---------- ROS orchestrator ----------
# Imported lazily so unit tests for the pure helpers do not require rclpy.


def _build_node_class():  # pragma: no cover - exercised on hardware only
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    class VelocityLoopEval(Node):
        """Send angular square waves and record odom for offline analysis."""

        def __init__(
            self,
            *,
            cmd_topic: str,
            odom_topic: str,
            publish_hz: float,
            hold_seconds: float,
            rest_seconds: float,
            motion_threshold: float,
            ss_window: float,
            axis: str = "angular",
        ):
            super().__init__("wheeltec_velocity_loop_eval")
            self.cmd_topic = cmd_topic
            self.odom_topic = odom_topic
            self.publish_hz = float(publish_hz)
            self.hold_seconds = float(hold_seconds)
            self.rest_seconds = float(rest_seconds)
            self.motion_threshold = float(motion_threshold)
            self.ss_window = float(ss_window)
            if axis not in ("angular", "linear"):
                raise ValueError(f"axis must be 'angular' or 'linear', got {axis!r}")
            self.axis = axis

            self._cmd_twist = Twist()
            self._current_cmd_w = 0.0
            self.samples: List[Sample] = []

            self.cmd_publisher = self.create_publisher(Twist, cmd_topic, 10)
            self.create_subscription(
                Odometry, odom_topic, self._odom_cb, 50,
            )

        def _odom_cb(self, msg):
            self.samples.append(
                Sample(
                    t_mono=time.monotonic(),
                    odom_lin_x=float(msg.twist.twist.linear.x),
                    odom_ang_z=float(msg.twist.twist.angular.z),
                )
            )

        def _publish_zero(self):
            self._current_cmd_w = 0.0
            twist = Twist()
            self.cmd_publisher.publish(twist)

        def _publish_cmd(self):
            twist = Twist()
            if self.axis == "linear":
                twist.linear.x = float(self._current_cmd_w)
            else:
                twist.angular.z = float(self._current_cmd_w)
            self.cmd_publisher.publish(twist)

        def _wait_until(self, deadline_mono: float, pub_period: float):
            while rclpy.ok() and time.monotonic() < deadline_mono:
                self._publish_cmd()
                slice_end = min(deadline_mono, time.monotonic() + pub_period)
                while rclpy.ok() and time.monotonic() < slice_end:
                    rclpy.spin_once(
                        self,
                        timeout_sec=max(slice_end - time.monotonic(), 0.0),
                    )

        def stop_burst(self, repetitions: int = 5, period_sec: float = 0.02):
            self._current_cmd_w = 0.0
            for _ in range(max(int(repetitions), 1)):
                self._publish_zero()
                time.sleep(max(float(period_sec), 0.0))

        def run_segment(
            self,
            *,
            target_w: float,
            publish_hz: float,
            name: str,
        ) -> SegmentSummary:
            pub_period = 1.0 / max(float(publish_hz), 1.0)
            self.samples = []

            self._current_cmd_w = 0.0
            t0 = time.monotonic()
            self._wait_until(t0 + self.rest_seconds, pub_period)

            t_step_on = time.monotonic()
            self._current_cmd_w = float(target_w)
            self._wait_until(
                t_step_on + self.hold_seconds, pub_period,
            )

            t_step_off = time.monotonic()
            self._current_cmd_w = 0.0
            self._wait_until(
                t_step_off + self.rest_seconds, pub_period,
            )

            return summarize_segment(
                samples=list(self.samples),
                target_w=float(target_w),
                publish_hz=float(publish_hz),
                t_step_on=t_step_on,
                t_step_off=t_step_off,
                threshold=self.motion_threshold,
                ss_window=self.ss_window,
                name=name,
                axis=self.axis,
            )

    return VelocityLoopEval


def _format_summary(summary: SegmentSummary) -> str:
    def fmt(x, unit=""):
        return "n/a" if x is None else f"{x:.4f}{unit}"

    return (
        f"{summary.name}: target_w={summary.target_w:.3f}rad/s, "
        f"hz={summary.publish_hz:.1f}, "
        f"start_lat={fmt(summary.start_latency, 's')}, "
        f"stop_lat={fmt(summary.stop_latency, 's')}, "
        f"ss_mean={fmt(summary.ss_mean)}rad/s, "
        f"ss_std={fmt(summary.ss_std)}rad/s, "
        f"ss_err={fmt(summary.ss_error)}rad/s, "
        f"n={summary.n_samples}"
    )


def _write_segment_csv(path: str, summary: SegmentSummary, samples: Iterable[Sample]):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "t_mono", "odom_lin_x", "odom_ang_z",
            "target_w", "publish_hz",
        ])
        for s in samples:
            writer.writerow([
                f"{s.t_mono:.6f}",
                f"{s.odom_lin_x:.6f}",
                f"{s.odom_ang_z:.6f}",
                f"{summary.target_w:.6f}",
                f"{summary.publish_hz:.3f}",
            ])


def _summary_to_dict(summary: SegmentSummary) -> dict:
    return {
        "name": summary.name,
        "target_w": summary.target_w,
        "publish_hz": summary.publish_hz,
        "start_latency": summary.start_latency,
        "stop_latency": summary.stop_latency,
        "ss_mean": summary.ss_mean,
        "ss_std": summary.ss_std,
        "ss_error": summary.ss_error,
        "ss_count": summary.ss_count,
        "n_samples": summary.n_samples,
        "threshold": summary.threshold,
    }


def _parse_float_list(text: str) -> List[float]:
    return [float(x.strip()) for x in text.split(",") if x.strip()]


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Wheeltec encoder closed-loop step + sweep evaluation.",
    )
    parser.add_argument(
        "--mode",
        choices=["step", "amp", "freq"],
        default="amp",
        help="Test mode: single step, amplitude sweep, or frequency sweep.",
    )
    parser.add_argument(
        "--axis",
        choices=["angular", "linear"],
        default="angular",
        help="Which axis to drive and measure: angular.z (rad/s) or linear.x (m/s).",
    )
    parser.add_argument(
        "--cmd-topic", default="/cmd_vel",
        help="Topic to publish command Twist (default: /cmd_vel, bypasses david_move).",
    )
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--publish-hz", type=float, default=50.0)
    parser.add_argument("--target-w", type=float, default=0.35)
    parser.add_argument(
        "--amplitudes", type=str, default="0.20,0.30,0.35,0.40,0.50",
        help="Comma-separated angular speeds for --mode amp",
    )
    parser.add_argument(
        "--frequencies", type=str, default="5,10,20,50,100,200",
        help="Comma-separated publish rates for --mode freq",
    )
    parser.add_argument("--hold-seconds", type=float, default=2.0)
    parser.add_argument("--rest-seconds", type=float, default=1.0)
    parser.add_argument("--motion-threshold", type=float, default=0.05)
    parser.add_argument("--ss-window", type=float, default=0.5)
    parser.add_argument(
        "--output-dir",
        default="/tmp/wheeltec_velocity_loop_eval",
        help="Directory for per-segment CSV and run summary JSON.",
    )
    parser.add_argument(
        "--label", default="run",
        help="Run label prepended to output filenames.",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None):  # pragma: no cover - hardware
    parser = _build_arg_parser()
    args = parser.parse_args(argv)

    import rclpy

    VelocityLoopEval = _build_node_class()
    rclpy.init()
    node = VelocityLoopEval(
        cmd_topic=args.cmd_topic,
        odom_topic=args.odom_topic,
        publish_hz=args.publish_hz,
        hold_seconds=args.hold_seconds,
        rest_seconds=args.rest_seconds,
        motion_threshold=args.motion_threshold,
        ss_window=args.ss_window,
        axis=args.axis,
    )

    os.makedirs(args.output_dir, exist_ok=True)
    run_summaries: List[dict] = []

    def run_one(target_w: float, publish_hz: float, name: str):
        node.publish_hz = publish_hz
        node.get_logger().info(
            f"running segment: {name} target_w={target_w:.3f} hz={publish_hz:.1f}"
        )
        summary = node.run_segment(
            target_w=target_w,
            publish_hz=publish_hz,
            name=name,
        )
        node.get_logger().info(_format_summary(summary))
        csv_path = os.path.join(args.output_dir, f"{args.label}_{name}.csv")
        _write_segment_csv(csv_path, summary, list(node.samples))
        run_summaries.append(_summary_to_dict(summary))

    try:
        # Always do a sign-flip pair (+w, -w) so we exercise both wheels.
        if args.mode == "step":
            run_one(args.target_w, args.publish_hz, "pos")
            node.stop_burst()
            run_one(-args.target_w, args.publish_hz, "neg")
        elif args.mode == "amp":
            for w in _parse_float_list(args.amplitudes):
                run_one(w, args.publish_hz, f"amp_pos_{w:.2f}")
                node.stop_burst()
                run_one(-w, args.publish_hz, f"amp_neg_{w:.2f}")
                node.stop_burst()
        elif args.mode == "freq":
            for hz in _parse_float_list(args.frequencies):
                run_one(args.target_w, hz, f"freq_pos_{hz:.0f}")
                node.stop_burst()
                run_one(-args.target_w, hz, f"freq_neg_{hz:.0f}")
                node.stop_burst()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted, sending stop burst")
    finally:
        node.stop_burst(repetitions=10, period_sec=0.02)
        json_path = os.path.join(args.output_dir, f"{args.label}_summary.json")
        with open(json_path, "w") as f:
            json.dump({
                "args": vars(args),
                "segments": run_summaries,
            }, f, indent=2)
        node.get_logger().info(f"wrote {json_path}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
