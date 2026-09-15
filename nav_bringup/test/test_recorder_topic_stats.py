"""Tests for topic rate and TF lag metrics."""

import pytest

from nav_bringup.recorder import topic_stats
from nav_bringup.recorder.topic_stats import TfSample
from nav_bringup.recorder.topic_stats import TopicSample
from nav_bringup.recorder.topic_stats import TopicStats


def test_rate_counter_counts_per_period():
    """Rates are messages per elapsed period; counts restart after each sample."""
    ticks = iter([10.0, 15.0, 25.0])
    counter = topic_stats.RateCounter(
        ['/cmd_vel', '/odom', '/cmd_vel'], clock=lambda: next(ticks))
    for _ in range(10):
        counter.record('/cmd_vel')
    counter.record('/odom')
    counter.record('/unknown')

    first = counter.sample(1000.0, {'/cmd_vel': 1, '/odom': 0})
    counter.record('/odom')
    second = counter.sample(1010.0)

    assert counter.topics == ['/cmd_vel', '/odom']
    assert first == [
        TopicSample(1000.0, '/cmd_vel', 10, 2.0, 1),
        TopicSample(1000.0, '/odom', 1, 0.2, 0),
    ]
    assert second == [
        TopicSample(1010.0, '/cmd_vel', 0, 0.0, None),
        TopicSample(1010.0, '/odom', 1, 0.1, None),
    ]


def test_transform_lag_ms():
    """Lag is the age of the stamp; future stamps give negative lag."""
    assert topic_stats.transform_lag_ms(100.25, 100.0) == 250.0
    assert topic_stats.transform_lag_ms(100.0, 100.5) == -500.0


def test_percentile():
    """Percentiles interpolate linearly between ranks."""
    assert topic_stats.percentile([4.0, 1.0, 3.0, 2.0], 0.5) == 2.5
    assert topic_stats.percentile([4.0, 1.0, 3.0, 2.0], 0.95) == pytest.approx(3.85)
    assert topic_stats.percentile([7.0], 0.95) == 7.0
    with pytest.raises(ValueError):
        topic_stats.percentile([], 0.5)
    with pytest.raises(ValueError):
        topic_stats.percentile([1.0], 1.5)


def test_csv_round_trip_and_summary(tmp_path):
    """Written rows are summarized; malformed rows are counted and skipped."""
    path = tmp_path / 'topics.csv'
    writer = topic_stats.MetricsCsvWriter(path)

    assert writer.write([TopicSample(1.0, '/cmd_vel', 100, 20.0, 1),
                         TfSample(1.0, 'map->robot', 12.5)]) == 2
    writer.write([TopicSample(6.0, '/cmd_vel', 0, 0.0, 2), TfSample(6.0, 'map->robot', None)])
    writer.write([TopicSample(11.0, '/cmd_vel', 50, 10.0, None),
                  TfSample(11.0, 'map->robot', 30.0),
                  TfSample(11.0, 'map->robot', 20.0)])
    with path.open('a', encoding='utf-8') as handle:
        handle.write('garbage,row\n')
        handle.write('12.000,topic,/cmd_vel,x,1.0,,\n')

    lines = path.read_text(encoding='utf-8').splitlines()
    summary = topic_stats.summarize_topics(path)

    assert lines[:3] == [
        'stamp,kind,name,messages,rate_hz,publishers,lag_ms',
        '1.000,topic,/cmd_vel,100,20.000,1,',
        '1.000,tf,map->robot,,,,12.5',
    ]
    assert summary.topics == (TopicStats('/cmd_vel', 3, 10.0, 0.0, 1, 2),)
    transform = summary.transforms[0]
    assert (transform.name, transform.samples, transform.missing_samples) == (
        'map->robot', 4, 1)
    assert (transform.lag_p50_ms, transform.lag_max_ms) == (20.0, 30.0)
    assert transform.lag_p95_ms == pytest.approx(29.0)
    assert summary.skipped_rows == 2


def test_summary_of_missing_or_unmeasured_files(tmp_path):
    """A missing file is empty; a transform that never resolved has no percentiles."""
    assert topic_stats.summarize_topics(tmp_path / 'missing.csv') == topic_stats.TopicsSummary()

    path = tmp_path / 'topics.csv'
    topic_stats.MetricsCsvWriter(path).write([TfSample(1.0, 'map->robot', None)])
    transform = topic_stats.summarize_topics(path).transforms[0]

    assert (transform.samples, transform.missing_samples) == (1, 1)
    assert (transform.lag_p50_ms, transform.lag_p95_ms, transform.lag_max_ms) == (
        None, None, None)
