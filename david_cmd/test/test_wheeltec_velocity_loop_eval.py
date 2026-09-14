"""Unit tests for the pure metric helpers used by the velocity loop eval node.

These metrics are extracted from the test orchestrator so they can be verified
without a running ROS graph. The orchestrator simply hands its recorded
samples to these functions to compute start/stop latency and steady-state
statistics for each angular-velocity step.
"""

import math
import pytest

from david_cmd.wheeltec_velocity_loop_eval import (
    Sample,
    SegmentSummary,
    aggregate_repetitions,
    compute_start_latency,
    compute_stop_latency,
    compute_steady_state,
    response_classification,
    summarize_segment,
)


def make_samples(pairs):
    """Build a list of Sample objects from (t, angular_z) pairs."""
    return [Sample(t_mono=t, odom_lin_x=0.0, odom_ang_z=w) for t, w in pairs]


def make_linear_samples(pairs):
    """Build a list of Sample objects from (t, linear_x) pairs (ang_z=0)."""
    return [Sample(t_mono=t, odom_lin_x=v, odom_ang_z=0.0) for t, v in pairs]


# ---------- compute_start_latency ----------

def test_start_latency_returns_first_crossing_after_step_on():
    samples = make_samples([
        (0.00, 0.00),
        (0.05, 0.01),
        (0.10, 0.02),
        (0.15, 0.06),  # first crossing above 0.05
        (0.20, 0.20),
    ])

    latency = compute_start_latency(
        samples, t_step_on=0.10, threshold=0.05,
    )

    assert latency == pytest.approx(0.05, abs=1e-9)


def test_start_latency_ignores_samples_before_step_on():
    samples = make_samples([
        (0.00, 0.30),   # spurious noise before step
        (0.05, 0.00),
        (0.10, 0.00),   # step issued here
        (0.15, 0.06),
    ])

    latency = compute_start_latency(
        samples, t_step_on=0.10, threshold=0.05,
    )

    assert latency == pytest.approx(0.05, abs=1e-9)


def test_start_latency_uses_absolute_value_for_negative_steps():
    samples = make_samples([
        (0.00, 0.00),
        (0.10, -0.01),
        (0.15, -0.08),  # |w| crosses threshold
    ])

    latency = compute_start_latency(
        samples, t_step_on=0.05, threshold=0.05,
    )

    assert latency == pytest.approx(0.10, abs=1e-9)


def test_start_latency_returns_none_when_no_crossing():
    samples = make_samples([
        (0.00, 0.00),
        (0.10, 0.02),
        (0.20, 0.03),
    ])

    latency = compute_start_latency(
        samples, t_step_on=0.05, threshold=0.10,
    )

    assert latency is None


# ---------- compute_stop_latency ----------

def test_stop_latency_returns_first_below_threshold_after_step_off():
    samples = make_samples([
        (0.00, 0.30),
        (0.10, 0.25),   # step_off issued here
        (0.20, 0.20),
        (0.30, 0.04),   # first sample below 0.05
        (0.40, 0.01),
    ])

    latency = compute_stop_latency(
        samples, t_step_off=0.10, threshold=0.05,
    )

    assert latency == pytest.approx(0.20, abs=1e-9)


def test_stop_latency_returns_none_when_never_stops():
    samples = make_samples([
        (0.00, 0.30),
        (0.10, 0.30),
        (0.20, 0.30),
    ])

    latency = compute_stop_latency(
        samples, t_step_off=0.10, threshold=0.05,
    )

    assert latency is None


def test_stop_latency_skips_samples_before_step_off():
    samples = make_samples([
        (0.00, 0.00),   # already at rest before step
        (0.05, 0.00),
        (0.10, 0.30),   # commanded ON here; not our window
        (0.20, 0.30),   # step_off issued at 0.20
        (0.30, 0.30),
        (0.40, 0.02),
    ])

    latency = compute_stop_latency(
        samples, t_step_off=0.20, threshold=0.05,
    )

    assert latency == pytest.approx(0.20, abs=1e-9)


# ---------- compute_steady_state ----------

def test_steady_state_returns_mean_std_count_in_window():
    samples = make_samples([
        (0.0, 0.10),
        (0.5, 0.20),
        (1.0, 0.30),
        (1.5, 0.32),
        (2.0, 0.34),
        (2.5, 0.36),
        (3.0, 0.05),   # outside the [1.5, 2.5] window
    ])

    ss = compute_steady_state(samples, t_from=1.5, t_to=2.5)

    assert ss.count == 3
    assert ss.mean == pytest.approx((0.32 + 0.34 + 0.36) / 3.0, abs=1e-9)
    expected_std = math.sqrt(
        sum((x - ss.mean) ** 2 for x in [0.32, 0.34, 0.36]) / 3.0
    )
    assert ss.std == pytest.approx(expected_std, abs=1e-9)


def test_steady_state_returns_zero_count_when_window_empty():
    samples = make_samples([
        (0.0, 0.10),
        (0.5, 0.20),
    ])

    ss = compute_steady_state(samples, t_from=1.0, t_to=2.0)

    assert ss.count == 0
    assert ss.mean is None
    assert ss.std is None


# ---------- summarize_segment integration ----------

def test_summarize_segment_packs_latency_and_steady_state():
    # Sample timestamps deliberately stay clear of the ss-window boundary so
    # the test is not sensitive to IEEE-754 rounding of t_step_off - ss_window.
    samples = make_samples([
        (0.00, 0.00),
        (0.05, 0.00),
        (0.20, 0.10),   # crosses threshold 0.05 after step_on=0.10
        (0.40, 0.35),
        (0.80, 0.35),   # steady-state window is [1.00, 1.50]
        (1.10, 0.35),   # in window
        (1.30, 0.35),   # in window
        (1.45, 0.35),   # in window
        (1.55, 0.30),   # step_off issued at 1.50; still rotating just after
        (1.80, 0.02),   # below threshold after 0.30s
    ])

    summary = summarize_segment(
        samples=samples,
        target_w=0.35,
        publish_hz=50.0,
        t_step_on=0.10,
        t_step_off=1.50,
        threshold=0.05,
        ss_window=0.50,
    )

    assert summary.target_w == pytest.approx(0.35)
    assert summary.publish_hz == pytest.approx(50.0)
    assert summary.start_latency == pytest.approx(0.10, abs=1e-9)
    assert summary.stop_latency == pytest.approx(0.30, abs=1e-9)
    assert summary.ss_count == 3
    assert summary.ss_mean == pytest.approx(0.35, abs=1e-9)
    assert summary.ss_error == pytest.approx(0.0, abs=1e-9)


# ---------- response_classification ----------

def _summary(
    target_w=0.35,
    publish_hz=50.0,
    start_latency=0.10,
    stop_latency=0.30,
    ss_mean=0.35,
    ss_std=0.0,
    ss_count=5,
    name="seg",
):
    return SegmentSummary(
        name=name,
        target_w=target_w,
        publish_hz=publish_hz,
        start_latency=start_latency,
        stop_latency=stop_latency,
        ss_mean=ss_mean,
        ss_std=ss_std,
        ss_error=None if ss_mean is None else ss_mean - target_w,
        ss_count=ss_count,
        n_samples=20,
        threshold=0.05,
    )


def test_response_classification_uses_start_latency_when_present():
    s = _summary(start_latency=0.10, ss_mean=0.0)

    assert response_classification(s, min_ss_fraction=0.3) is True


def test_response_classification_uses_ss_fraction_when_no_start_latency():
    # No start_latency reported but ss_mean is 50% of target (well above 30%).
    s = _summary(start_latency=None, ss_mean=0.18, target_w=0.35)

    assert response_classification(s, min_ss_fraction=0.3) is True


def test_response_classification_rejects_segment_with_no_motion():
    s = _summary(start_latency=None, ss_mean=0.0, target_w=0.50)

    assert response_classification(s, min_ss_fraction=0.3) is False


def test_response_classification_rejects_wrong_sign():
    # Commanded negative but ss_mean came back positive — count as non-response.
    s = _summary(start_latency=0.10, ss_mean=+0.33, target_w=-0.50)

    assert response_classification(s, min_ss_fraction=0.3) is False


# ---------- aggregate_repetitions ----------

def test_aggregate_repetitions_counts_runs_and_response_rate():
    segs = [
        _summary(target_w=0.6, start_latency=0.15, ss_mean=0.52),
        _summary(target_w=0.6, start_latency=None, ss_mean=0.0),
        _summary(target_w=0.6, start_latency=0.20, ss_mean=0.55),
    ]

    stats = aggregate_repetitions(segs)

    assert stats.target_w == pytest.approx(0.6)
    assert stats.publish_hz == pytest.approx(50.0)
    assert stats.n == 3
    assert stats.n_responded == 2
    assert stats.response_rate == pytest.approx(2.0 / 3.0)


def test_aggregate_repetitions_averages_only_responded_for_latency():
    segs = [
        _summary(target_w=0.6, start_latency=0.10, stop_latency=1.0, ss_mean=0.50),
        _summary(target_w=0.6, start_latency=None, stop_latency=None, ss_mean=0.0),
        _summary(target_w=0.6, start_latency=0.30, stop_latency=1.4, ss_mean=0.54),
    ]

    stats = aggregate_repetitions(segs)

    assert stats.start_lat_avg == pytest.approx(0.20, abs=1e-9)
    assert stats.stop_lat_avg == pytest.approx(1.20, abs=1e-9)


def test_aggregate_repetitions_ss_mean_includes_all_runs():
    # ss_mean across-runs counts ALL runs (zero counts as "no response, 0 rad/s").
    segs = [
        _summary(target_w=0.6, ss_mean=0.50),
        _summary(target_w=0.6, ss_mean=0.0),
        _summary(target_w=0.6, ss_mean=0.55),
    ]

    stats = aggregate_repetitions(segs)

    assert stats.ss_mean_avg == pytest.approx((0.50 + 0.0 + 0.55) / 3.0, abs=1e-9)


def test_aggregate_repetitions_rejects_mixed_targets():
    segs = [
        _summary(target_w=0.6),
        _summary(target_w=0.5),
    ]

    with pytest.raises(ValueError):
        aggregate_repetitions(segs)


def test_aggregate_repetitions_rejects_empty_input():
    with pytest.raises(ValueError):
        aggregate_repetitions([])


def test_aggregate_repetitions_handles_all_nonresponse():
    segs = [
        _summary(target_w=0.6, start_latency=None, stop_latency=None, ss_mean=0.0),
        _summary(target_w=0.6, start_latency=None, stop_latency=None, ss_mean=0.0),
    ]

    stats = aggregate_repetitions(segs)

    assert stats.n == 2
    assert stats.n_responded == 0
    assert stats.response_rate == 0.0
    assert stats.start_lat_avg is None
    assert stats.stop_lat_avg is None
    assert stats.ss_mean_avg == pytest.approx(0.0, abs=1e-9)


# ---------- axis="linear" support ----------

def test_compute_start_latency_supports_linear_axis():
    samples = make_linear_samples([
        (0.00, 0.00),
        (0.10, 0.02),
        (0.15, 0.06),  # |lin_x| crosses threshold 0.05
        (0.20, 0.20),
    ])

    latency = compute_start_latency(
        samples, t_step_on=0.10, threshold=0.05, axis="linear",
    )

    assert latency == pytest.approx(0.05, abs=1e-9)


def test_compute_start_latency_linear_does_not_read_angular_field():
    # angular field is loud but we should ignore it under axis="linear".
    samples = [
        Sample(t_mono=0.0, odom_lin_x=0.0, odom_ang_z=1.0),
        Sample(t_mono=0.1, odom_lin_x=0.0, odom_ang_z=1.0),
        Sample(t_mono=0.2, odom_lin_x=0.0, odom_ang_z=1.0),
    ]

    latency = compute_start_latency(
        samples, t_step_on=0.0, threshold=0.05, axis="linear",
    )

    assert latency is None


def test_compute_stop_latency_supports_linear_axis():
    samples = make_linear_samples([
        (0.00, 0.20),
        (0.10, 0.20),   # step_off at 0.10
        (0.30, 0.10),
        (0.40, 0.04),   # first below threshold 0.05
    ])

    latency = compute_stop_latency(
        samples, t_step_off=0.10, threshold=0.05, axis="linear",
    )

    assert latency == pytest.approx(0.30, abs=1e-9)


def test_compute_steady_state_supports_linear_axis():
    samples = make_linear_samples([
        (0.0, 0.10),
        (1.5, 0.20),
        (2.0, 0.18),
        (2.5, 0.22),
    ])

    ss = compute_steady_state(samples, t_from=1.5, t_to=2.5, axis="linear")

    assert ss.count == 3
    assert ss.mean == pytest.approx((0.20 + 0.18 + 0.22) / 3.0, abs=1e-9)


def test_summarize_segment_axis_linear_uses_linear_x():
    samples = [
        Sample(t_mono=0.0, odom_lin_x=0.0, odom_ang_z=0.0),
        Sample(t_mono=0.1, odom_lin_x=0.0, odom_ang_z=0.0),
        Sample(t_mono=0.2, odom_lin_x=0.10, odom_ang_z=0.0),  # crosses 0.05
        Sample(t_mono=1.5, odom_lin_x=0.20, odom_ang_z=0.0),
        Sample(t_mono=2.0, odom_lin_x=0.20, odom_ang_z=0.0),
        Sample(t_mono=2.5, odom_lin_x=0.20, odom_ang_z=0.0),  # step_off at 2.5
        Sample(t_mono=3.0, odom_lin_x=0.02, odom_ang_z=0.0),  # below threshold
    ]

    summary = summarize_segment(
        samples=samples,
        target_w=0.20,
        publish_hz=50.0,
        t_step_on=0.1,
        t_step_off=2.5,
        threshold=0.05,
        ss_window=1.0,
        axis="linear",
    )

    assert summary.start_latency == pytest.approx(0.10, abs=1e-9)
    assert summary.stop_latency == pytest.approx(0.50, abs=1e-9)
    assert summary.ss_mean == pytest.approx(0.20, abs=1e-9)
    assert summary.ss_error == pytest.approx(0.0, abs=1e-9)
