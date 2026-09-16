"""Tests for the structured event log."""

import json
import os
from pathlib import Path
import subprocess
import sys
import threading

import pytest

from nav_bringup.recorder import events


def test_append_and_read_round_trip(tmp_path):
    """Recorded events are read back unchanged and in order."""
    log = events.EventLog(tmp_path / 'events.jsonl', clock=iter([1.5, 2.5]).__next__)

    first = log.append('navrun', 'run_started', data={'mode': 'tour'})
    second = log.append('bt', 'escape_triggered', 'warn', {'node': 'EscapeObstacle'})
    read, skipped = events.read_events(log.path)

    assert read == [first, second]
    assert skipped == 0


def test_json_line_format_keeps_non_ascii(tmp_path):
    """Each line is a JSON object with stable keys and readable Chinese text."""
    log = events.EventLog(tmp_path / 'events.jsonl', clock=lambda: 10.0)

    log.append('executor', 'task_failed', 'error', {'reason': '目标点不存在'})
    line = log.path.read_text(encoding='utf-8')

    assert line == (
        '{"stamp": 10.0, "source": "executor", "type": "task_failed", '
        '"severity": "error", "data": {"reason": "目标点不存在"}}\n')


def test_invalid_severity_is_rejected_without_writing(tmp_path):
    """Severity is validated before anything is written."""
    log = events.EventLog(tmp_path / 'events.jsonl')

    with pytest.raises(ValueError):
        log.append('navrun', 'x', severity='fatal')
    assert not log.path.exists()


def test_read_events_skips_and_counts_malformed_lines(tmp_path):
    """Corrupt lines do not hide the valid ones."""
    path = tmp_path / 'events.jsonl'
    valid = events.Event(3.0, 'navrun', 'run_stopped').to_json()
    path.write_text(
        '\n'.join(['not json', '[1, 2]', json.dumps({'stamp': 1}), valid, '']),
        encoding='utf-8')

    read, skipped = events.read_events(path)

    assert [event.event_type for event in read] == ['run_stopped']
    assert skipped == 3


def test_read_events_missing_file(tmp_path):
    """A run without events has an empty timeline."""
    assert events.read_events(tmp_path / 'events.jsonl') == ([], 0)


def test_concurrent_appends_produce_whole_lines(tmp_path):
    """Appends from several threads never interleave within a line."""
    log = events.EventLog(tmp_path / 'events.jsonl')

    def worker(index):
        for count in range(50):
            log.append('worker', 'tick', data={'worker': index, 'count': count, 'pad': 'x' * 200})

    threads = [threading.Thread(target=worker, args=(index,)) for index in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    read, skipped = events.read_events(log.path)

    assert len(read) == 200
    assert skipped == 0


def test_independent_processes_append_complete_large_lines(tmp_path):
    """Monitor and parent writers share an advisory append lock, not a Python-only lock."""
    path = tmp_path / 'events.jsonl'
    code = (
        'import sys\n'
        'from nav_bringup.recorder.events import EventLog\n'
        'log = EventLog(sys.argv[1])\n'
        'for index in range(25):\n'
        '    log.append(sys.argv[2], "tick", data={"index":index,"pad":"x"*20000})\n'
    )
    env = dict(os.environ)
    env['PYTHONPATH'] = str(Path(__file__).resolve().parents[1])
    results = []

    def worker(name):
        results.append(subprocess.run(
            [sys.executable, '-c', code, str(path), name], env=env,
            capture_output=True, timeout=10, check=False))

    threads = [threading.Thread(target=worker, args=(f'writer{i}',)) for i in range(3)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=15)
        assert not thread.is_alive()
    assert len(results) == 3 and all(result.returncode == 0 for result in results)
    recorded, skipped = events.read_events(path)
    assert len(recorded) == 75 and skipped == 0
    assert len({(row.source, row.data['index']) for row in recorded}) == 75
    assert all(len(row.data['pad']) == 20000 for row in recorded)
