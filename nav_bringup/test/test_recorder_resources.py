"""Tests for resource sampling and aggregation."""

import csv
import threading
from types import SimpleNamespace

from nav_bringup.recorder import resources

MB = 1024 * 1024


class _NoSuchProcess(Exception):
    pass


class _AccessDenied(Exception):
    pass


class _ZombieProcess(Exception):
    pass


class _FakeProcess:
    """Process double backed by a mutable table keyed by pid."""

    def __init__(self, table, pid):
        if pid not in table:
            raise _NoSuchProcess(pid)
        self.pid = pid
        self._table = table

    def _entry(self):
        entry = self._table.get(self.pid)
        if entry is None or entry.get('gone'):
            raise _NoSuchProcess(self.pid)
        return entry

    def children(self, recursive=False):
        found = []
        pending = list(self._entry().get('children', []))
        while pending:
            pid = pending.pop(0)
            found.append(_FakeProcess(self._table, pid))
            if recursive:
                pending.extend(self._table[pid].get('children', []))
        return found

    def name(self):
        return self._entry()['name']

    def cpu_percent(self, interval=None):
        return self._entry()['cpu']

    def memory_info(self):
        return SimpleNamespace(rss=self._entry()['rss'])

    def num_threads(self):
        return self._entry()['threads']


def _fake_psutil(table, system_cpu):
    return SimpleNamespace(
        Process=lambda pid: _FakeProcess(table, pid),
        NoSuchProcess=_NoSuchProcess,
        AccessDenied=_AccessDenied,
        ZombieProcess=_ZombieProcess,
        cpu_percent=lambda interval=None: system_cpu[0],
        virtual_memory=lambda: SimpleNamespace(used=2048 * MB),
    )


def _table():
    return {
        100: {'name': 'ros2', 'cpu': 1.0, 'rss': 50 * MB, 'threads': 4, 'children': [101]},
        101: {'name': 'controller_server', 'cpu': 30.0, 'rss': 80 * MB, 'threads': 12,
              'children': [102]},
        102: {'name': 'helper', 'cpu': 2.0, 'rss': 10 * MB, 'threads': 1},
        200: {'name': 'unrelated', 'cpu': 99.0, 'rss': 1 * MB, 'threads': 1},
    }


def test_sample_covers_process_tree_and_system_row(tmp_path):
    """The system row comes first, followed by every process of the tree."""
    table = _table()
    sampler = resources.ResourceSampler(
        lambda: [100], tmp_path / 'resources.csv', clock=lambda: 5.0,
        psutil_module=_fake_psutil(table, [12.5]))

    samples = sampler.sample()

    assert [sample.name for sample in samples] == [
        resources.SYSTEM_ROW_NAME, 'ros2', 'controller_server', 'helper']
    assert samples[0].cpu_percent == 12.5
    assert samples[0].rss_mb == 2048.0
    assert samples[2].rss_mb == 80.0
    with (tmp_path / 'resources.csv').open(encoding='utf-8', newline='') as handle:
        rows = list(csv.reader(handle))
    assert rows[0] == list(resources.CSV_COLUMNS)
    assert rows[2] == ['5.000', '100', 'ros2', '1.0', '50.0', '4']
    assert len(rows) == 5


def test_header_is_written_once_and_vanished_processes_are_skipped(tmp_path):
    """Later samples append rows only and drop processes that exited."""
    table = _table()
    sampler = resources.ResourceSampler(
        lambda: [100], tmp_path / 'resources.csv', clock=lambda: 5.0,
        psutil_module=_fake_psutil(table, [10.0]))

    sampler.sample()
    table[102]['gone'] = True
    second = sampler.sample()

    assert [sample.name for sample in second] == [
        resources.SYSTEM_ROW_NAME, 'ros2', 'controller_server']
    with (tmp_path / 'resources.csv').open(encoding='utf-8', newline='') as handle:
        rows = list(csv.reader(handle))
    assert sum(1 for row in rows if row[0] == 'stamp') == 1
    assert len(rows) == 1 + 4 + 3


def test_missing_root_process_yields_system_row_only(tmp_path):
    """A root that already exited does not break sampling."""
    sampler = resources.ResourceSampler(
        lambda: [999], tmp_path / 'resources.csv',
        psutil_module=_fake_psutil(_table(), [3.0]))

    assert [sample.pid for sample in sampler.sample()] == [0]


def test_run_returns_immediately_when_stopped(tmp_path):
    """A stop event that is already set prevents any sampling."""
    sampler = resources.ResourceSampler(
        lambda: [100], tmp_path / 'resources.csv',
        psutil_module=_fake_psutil(_table(), [3.0]))
    stop = threading.Event()
    stop.set()

    sampler.run(stop, interval_sec=0.01)

    assert not (tmp_path / 'resources.csv').exists()


def test_summarize_resources_aggregates_by_name(tmp_path):
    """Averages and peaks are computed per process name and for the system."""
    table = _table()
    system_cpu = [20.0]
    sampler = resources.ResourceSampler(
        lambda: [100], tmp_path / 'resources.csv', clock=lambda: 1.0,
        psutil_module=_fake_psutil(table, system_cpu))
    sampler.sample()
    table[101]['cpu'] = 50.0
    table[101]['rss'] = 90 * MB
    system_cpu[0] = 40.0
    sampler.sample()

    summary = resources.summarize_resources(sampler.csv_path)

    assert summary.system == resources.ProcessStats(
        resources.SYSTEM_ROW_NAME, 2, 30.0, 40.0, 2048.0)
    assert [stats.name for stats in summary.processes] == ['controller_server', 'helper', 'ros2']
    assert summary.processes[0] == resources.ProcessStats(
        'controller_server', 2, 40.0, 50.0, 90.0)


def test_summarize_missing_file_is_empty(tmp_path):
    """Runs without samples have an empty resource summary."""
    assert resources.summarize_resources(tmp_path / 'missing.csv') == resources.ResourceSummary()
