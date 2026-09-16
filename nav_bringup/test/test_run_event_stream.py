"""Navrun outward events use only owned files and fake publication callbacks."""

import json
from types import SimpleNamespace as NS

import pytest

from nav_bringup.recorder.events import EventLog
from nav_bringup.recorder.nav_events import NavEventRecorder


def _stream(tmp_path, publish=None):
    from nav_bringup.recorder.run_event_stream import RunEventStream

    sent, warnings = [], []
    stream = RunEventStream(tmp_path / 'events.jsonl', publish or sent.append, warnings.append)
    return stream, sent, warnings


def test_initial_history_and_new_lines_keep_identity_time_and_safe_fields(tmp_path):
    """A monitor may start after the parent; its bounded cursor does not replace timestamps."""
    log = EventLog(tmp_path / 'events.jsonl', clock=lambda: 12.3456789)
    log.append('navrun', 'run_started', data={'mode': 'precise', 'command': ['secret-argument']})
    stream, sent, warnings = _stream(tmp_path)
    assert stream.pump() == 1
    assert sent[0]['source'] == 'navrun' and sent[0]['event_type'] == 'run_started'
    assert sent[0]['stamp'] == 12.345679  # Preserve the actual already-recorded JSONL timestamp.
    data = json.loads(sent[0]['data_json'])
    assert data['mode'] == 'precise' and data['fields_omitted'] == 1
    assert 'secret-argument' not in json.dumps(sent)
    assert data['record_offset'] == 0
    log.append('navrun', 'process_exited', data={'return_code': 3, 'error': 'private-error'})
    assert stream.pump() == 1 and stream.pump() == 0
    assert len(sent) == 2 and sent[0]['event_id'] != sent[1]['event_id']
    assert json.loads(sent[1]['data_json'])['return_code'] == 3
    assert not warnings and stream.offset == log.path.stat().st_size


def test_restart_resumes_cursor_and_identity(tmp_path):
    """A successful checkpoint survives a new monitor instance on the same run."""
    log = EventLog(tmp_path / 'events.jsonl', clock=lambda: 1)
    log.append('navrun', 'run_started')
    first, sent, _ = _stream(tmp_path)
    assert first.pump() == 1
    prefix = first.identity_prefix
    first.close()
    second, later, _ = _stream(tmp_path)
    assert second.identity_prefix == prefix and second.pump() == 0
    log.append('navrun', 'startup_ready')
    assert second.pump() == 1
    assert later[0]['event_id'].startswith(prefix)
    assert later[0]['event_id'] != sent[0]['event_id']


def test_publish_failure_retries_same_record_without_consuming_cursor(tmp_path):
    """Failure is bounded by a pump call, not an unbounded busy retry loop."""
    log = EventLog(tmp_path / 'events.jsonl', clock=lambda: 1)
    log.append('navrun', 'run_started')
    attempts = []

    def publish(message):
        attempts.append(message)
        if len(attempts) == 1:
            raise OSError('secret transport detail')

    stream, _, warnings = _stream(tmp_path, publish)
    assert stream.pump() == 0 and stream.offset == 0
    assert stream.pump() == 1
    assert attempts[0] == attempts[1] and len(warnings) == 1
    assert 'secret' not in warnings[0]


def test_foreign_records_are_not_forwarded_and_self_echo_is_not_recorded(tmp_path):
    """A two-way topic cannot recursively amplify local or remote run events."""
    log = EventLog(tmp_path / 'events.jsonl', clock=lambda: 1)
    stream, sent, _ = _stream(tmp_path)
    recorder = NavEventRecorder(log, lambda value: pytest.fail(value),
                                ignored_prefixes=(stream.identity_prefix,))
    log.append('navrun', 'run_started')
    assert stream.pump() == 1
    assert not recorder.receive(NS(**sent[0]))
    assert recorder.receive(NS(event_id='other:1', source='navrun', event_type='run_started',
                               severity='info', stamp=2.0, data_json='{}'))
    assert recorder.receive(NS(event_id='task:1', source='tasks', event_type='task_failed',
                               severity='error', stamp=2.0, data_json='{}'))
    assert stream.pump() == 0 and len(sent) == 1
    assert len(log.path.read_text().splitlines()) == 3


def test_partial_line_is_not_consumed_and_pump_has_a_record_budget(tmp_path):
    """A live append boundary is not corrupt JSON; a backlog cannot monopolize the node."""
    path = tmp_path / 'events.jsonl'
    line = '{"stamp":1,"source":"navrun","type":"started","severity":"info","data":{}}'
    path.write_text(line[:20])
    stream, sent, _ = _stream(tmp_path)
    assert stream.pump() == 0 and stream.offset == 0
    path.write_text((line + '\n') * 5)
    assert stream.pump(max_records=2) == 2
    assert stream.pump(max_records=2) == 2
    assert stream.pump(max_records=2) == 1
    assert len({message['event_id'] for message in sent}) == 5


@pytest.mark.parametrize('change', ['truncate', 'replace'])
def test_mutated_event_file_fails_closed_without_reusing_identity(tmp_path, change):
    """A run timeline is append-only; changed file identity cannot silently reset the cursor."""
    log = EventLog(tmp_path / 'events.jsonl', clock=lambda: 1)
    log.append('navrun', 'started')
    stream, sent, warnings = _stream(tmp_path)
    assert stream.pump() == 1
    if change == 'truncate':
        log.path.write_text('')
    else:
        log.path.rename(tmp_path / 'original.jsonl')
        log.append('navrun', 'started')
    assert stream.pump() == 0 and warnings
    assert len(sent) == 1


def test_bad_record_is_skipped_without_hiding_a_later_valid_event(tmp_path):
    """No raw line is leaked in warnings, and valid later local events remain available."""
    path = tmp_path / 'events.jsonl'
    path.write_text('private invalid input\n')
    EventLog(path, clock=lambda: 1).append('navrun', 'startup_ready')
    stream, sent, warnings = _stream(tmp_path)
    assert stream.pump() == 1 and len(sent) == 1
    assert len(warnings) == 1 and 'private' not in warnings[0]


def test_close_suppresses_future_reads_and_publication(tmp_path):
    """Closing the stream cannot destroy its caller's node or event file."""
    stream, sent, _ = _stream(tmp_path)
    stream.close()
    stream.close()
    EventLog(tmp_path / 'events.jsonl').append('navrun', 'started')
    assert stream.pump() == 0 and not sent


def test_publication_does_not_hold_the_parent_append_lock(tmp_path):
    """DDS backpressure must not retain the advisory lock needed by the parent writer."""
    import fcntl

    path = tmp_path / 'events.jsonl'
    EventLog(path).append('navrun', 'started')

    def publish(message):
        with path.open('a') as handle:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)

    stream, _, warnings = _stream(tmp_path, publish)
    assert stream.pump() == 1 and not warnings


@pytest.mark.parametrize('error', [FileNotFoundError, BlockingIOError])
def test_publication_io_errors_are_not_confused_with_an_uncreated_or_busy_file(tmp_path, error):
    """Only source-file open/lock conditions are silent; actual sink failures are diagnosed."""
    EventLog(tmp_path / 'events.jsonl').append('navrun', 'started')

    def publish(message):
        raise error('private detail')

    stream, _, warnings = _stream(tmp_path, publish)
    assert stream.pump() == 0 and stream.offset == 0 and len(warnings) == 1


def test_failed_checkpoint_is_retried_even_without_more_records(tmp_path, monkeypatch):
    """An idle pump still repairs a failed cursor write without repeating publication."""
    import nav_bringup.recorder.run_event_stream as module

    log = EventLog(tmp_path / 'events.jsonl')
    log.append('navrun', 'started')
    stream, sent, warnings = _stream(tmp_path)
    assert stream.pump() == 1
    log.append('navrun', 'startup_ready')
    replace = module.os.replace
    monkeypatch.setattr(module.os, 'replace', lambda *args: (_ for _ in ()).throw(OSError()))
    assert stream.pump() == 1
    monkeypatch.setattr(module.os, 'replace', replace)
    assert stream.pump() == 0 and len(sent) == 2 and warnings
    stream.close()
    reopened, again, _ = _stream(tmp_path)
    assert reopened.pump() == 0 and not again


def test_oversized_line_is_discarded_in_bounded_chunks_across_restart(tmp_path):
    """No arbitrarily large line is buffered; restart remembers a partial discard."""
    path = tmp_path / 'events.jsonl'
    path.write_bytes(b'x' * (2 * 1048576) + b'\n')
    EventLog(path).append('navrun', 'startup_ready')
    stream, sent, warnings = _stream(tmp_path)
    assert stream.pump() == 0
    assert 0 < stream.offset <= 1048576 and len(warnings) == 1
    stream.close()
    reopened, sent, warnings = _stream(tmp_path)
    assert reopened.pump() == 0
    assert reopened.pump() == 1
    assert sent[0]['event_type'] == 'startup_ready' and not warnings


def test_only_one_publisher_owns_cursor_and_failed_open_releases_its_lease(tmp_path):
    """A second observer cannot race checkpoints; corrupt state is never overwritten."""
    stream, _, _ = _stream(tmp_path)
    with pytest.raises(BlockingIOError):
        _stream(tmp_path)
    stream.close()
    state = tmp_path / 'nav_event_cursor.json'
    state.write_text('corrupt private state')
    with pytest.raises(ValueError):
        _stream(tmp_path)
    assert state.read_text() == 'corrupt private state'
    # A second failure must be the same invalid state, not a leaked file lease.
    with pytest.raises(ValueError):
        _stream(tmp_path)


@pytest.mark.parametrize('line', [
    '[]', '{"source":"navrun","data":null}',
    '{"source":"navrun","data":{},"stamp":NaN,"type":"started","severity":"info"}',
    '{"source":"navrun","data":{},"stamp":true,"type":"started","severity":"info"}',
    '{"source":"navrun","data":{},"stamp":1,"type":"started","severity":"fatal"}',
    '{"source":"navrun","data":{"x":1,"x":2},"stamp":1,"type":"x","severity":"info"}',
])
def test_invalid_complete_records_do_not_hide_following_valid_event(tmp_path, line):
    """Bad local lines remain on disk for audit, but are not put on the public topic."""
    path = tmp_path / 'events.jsonl'
    path.write_text(line + '\n')
    EventLog(path).append('navrun', 'startup_ready')
    stream, sent, warnings = _stream(tmp_path)
    assert stream.pump() == 1 and len(warnings) == 1
    assert sent[0]['event_type'] == 'startup_ready'
    assert path.read_text().startswith(line)


@pytest.mark.parametrize('bound', [True, 0, -1, 1.5])
def test_batch_bound_is_explicit_and_positive(tmp_path, bound):
    """A malformed caller bound cannot accidentally request an unlimited scan."""
    stream, sent, _ = _stream(tmp_path)
    with pytest.raises(ValueError):
        stream.pump(max_records=bound)
    assert not sent
