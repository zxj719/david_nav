"""Record shared navigation events without ROS initialization or graph traffic."""

import json
import threading
from types import SimpleNamespace as NS

import pytest

from nav_bringup.recorder.events import EventLog, read_events


def _message(**values):
    defaults = dict(event_id='producer:1', stamp=1789544395.1234567,
                    source='tasks', event_type='task_succeeded', severity='info',
                    data_json='{"task_id":"t","generation":2,"ownership_released":true}')
    defaults.update(values)
    return NS(**defaults)


def _recorder(tmp_path, **kwargs):
    from nav_bringup.recorder.nav_events import NavEventRecorder

    warnings = []
    log = EventLog(tmp_path / 'events.jsonl',
                   clock=lambda: pytest.fail('receipt is not event time'))
    return NavEventRecorder(log, warnings.append, **kwargs), log, warnings


def test_record_preserves_source_time_identity_and_measurement_precision(tmp_path):
    """ROS payloads become normal five-field JSONL without using receipt time."""
    recorder, log, warnings = _recorder(tmp_path)
    message = _message(source='dock', event_type='dock_measured', data_json=json.dumps({
        'exy': 0.0123456789123, 'eyaw': -0.00123456789123, 'detail': '测量',
        'task_id': 't', 'sub_id': 3, 'generation': 7}))
    assert recorder.receive(message)
    assert not recorder.receive(message)
    lines = log.path.read_text().splitlines()
    assert len(lines) == 1 and not warnings
    row = json.loads(lines[0])
    assert set(row) == {'stamp', 'source', 'type', 'severity', 'data'}
    assert row['stamp'] == message.stamp
    assert row['data']['exy'] == 0.0123456789123
    assert row['data']['__nav_event'] == {'event_id': 'producer:1'}
    assert row['source'] == 'dock' and row['type'] == 'dock_measured'
    assert read_events(log.path)[1] == 0


@pytest.mark.parametrize('values', [
    {'event_id': ''}, {'event_id': ' '}, {'event_id': 'x' * 257},
    {'source': None}, {'source': ' '}, {'event_type': ''}, {'severity': 'fatal'},
    {'stamp': float('nan')}, {'stamp': float('inf')}, {'stamp': True}, {'stamp': '1'},
    {'data_json': '[]'}, {'data_json': 'null'}, {'data_json': '{bad'},
    {'data_json': '{"x":NaN}'}, {'data_json': '{"x":1e999}'},
    {'data_json': '{"x":1,"x":2}'}, {'data_json': '{"__nav_event":{}}'},
    {'data_json': '{"x":"' + 'x' * 65536 + '"}'},
    {'data_json': '{"x":' + '[' * 33 + '1' + ']' * 33 + '}'},
    {'data_json': '{"x":"\\ud800"}'},
])
def test_invalid_envelope_is_rejected_without_partial_record(tmp_path, values):
    """Malformed remote input cannot crash the monitor or write ambiguous JSON."""
    recorder, log, warnings = _recorder(tmp_path)
    assert not recorder.receive(_message(**values))
    assert not log.path.exists()
    assert len(warnings) == 1 and 'navigation event' in warnings[0]
    assert 'producer:1' not in warnings[0]


def test_write_failure_is_retryable_and_does_not_consume_identity(tmp_path, monkeypatch):
    """Dedupe admission follows the write, not merely successful decoding."""
    recorder, log, warnings = _recorder(tmp_path)
    record = log.record

    def fail(*args, **kwargs):
        raise OSError('private file path must not appear in a warning')

    monkeypatch.setattr(log, 'record', fail)
    assert not recorder.receive(_message())
    monkeypatch.setattr(log, 'record', record)
    assert recorder.receive(_message())
    assert len(log.path.read_text().splitlines()) == 1
    assert len(warnings) == 1 and 'private' not in warnings[0]


def test_bounded_deduplication_and_close_fence(tmp_path):
    """Memory is bounded; closing stops late callbacks without deleting recorded history."""
    recorder, log, _ = _recorder(tmp_path, capacity=2)
    for identity in ('p:1', 'p:2', 'p:1', 'p:3'):
        recorder.receive(_message(event_id=identity))
    assert len(log.path.read_text().splitlines()) == 3
    assert recorder.receive(_message(event_id='p:2'))  # The least recent ID was evicted.
    recorder.close()
    recorder.close()
    assert not recorder.receive(_message(event_id='p:4'))
    assert len(log.path.read_text().splitlines()) == 4


def test_concurrent_duplicate_callbacks_record_once(tmp_path):
    """A future multithreaded monitor cannot race identity admission and append."""
    recorder, log, warnings = _recorder(tmp_path)
    results = []
    threads = [threading.Thread(target=lambda: results.append(recorder.receive(_message())))
               for _ in range(12)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=2)
        assert not thread.is_alive()
    assert results.count(True) == 1
    assert len(log.path.read_text().splitlines()) == 1 and not warnings


def test_bad_diagnostic_sink_does_not_stop_later_valid_input(tmp_path):
    """Neither remote payload nor diagnostic delivery can become motion authority."""
    from nav_bringup.recorder.nav_events import NavEventRecorder

    def fail(message):
        raise OSError('sink unavailable')

    log = EventLog(tmp_path / 'events.jsonl')
    recorder = NavEventRecorder(log, fail)
    assert not recorder.receive(_message(data_json='bad'))
    assert recorder.receive(_message())


@pytest.mark.parametrize('capacity', [0, -1, True, 1.5])
def test_invalid_cache_capacity_is_rejected_before_io(tmp_path, capacity):
    """The explicit bound must be a positive integer, not silently unlimited."""
    with pytest.raises(ValueError):
        _recorder(tmp_path, capacity=capacity)
    assert not (tmp_path / 'events.jsonl').exists()
