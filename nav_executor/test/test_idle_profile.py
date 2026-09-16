"""Passive CPU evidence contracts using virtual time and synthetic proc data."""

import csv
import json
import os

import pytest

from nav_executor.app import idle_profile as profile


class _World:
    def __init__(self, rate=3, delay=0):
        self.now, self.rate, self.delay = 0.0, rate, delay
        self.start, self.failure = 100, None

    def read(self):
        if self.failure and self.now >= 2:
            raise self.failure
        return profile._Counters(self.start, int(self.rate * self.now), 4096, 2, 'S')

    def sleep(self, seconds):
        self.now += seconds + self.delay

    def collect(self, **kwargs):
        rows = []
        result = profile._collect(self.read, 60, 1, 100, rows.append,
                                  clock=lambda: self.now, sleep=self.sleep, **kwargs)
        return result, rows


@pytest.mark.parametrize('rate,threshold', [(2, False), (3, True), (120, True)])
def test_complete_window_uses_one_core_cpu_and_excludes_priming_zero(rate, threshold):
    """Sixty real intervals are measured, with no child CPU or first-zero bias."""
    result, rows = _World(rate).collect(idle_confirmed=True)
    assert result['complete'] and result['r6_eligible']
    assert result['intervals'] == 60 and len(rows) == 61
    assert rows[0]['cpu_percent'] is None
    assert result['cpu_mean_percent'] == rate
    assert result['r6_at_or_above_three_percent'] is threshold


def test_unconfirmed_idle_and_delayed_cadence_do_not_make_a_policy_conclusion():
    """Diagnostic data is retained without pretending the R6 prerequisites are satisfied."""
    for world, confirmed in [(_World(), False), (_World(delay=2), True)]:
        result, _ = world.collect(idle_confirmed=confirmed)
        assert result['complete'] and not result['r6_eligible']
        assert result['r6_at_or_above_three_percent'] is None


def test_sparse_long_interval_is_not_the_required_one_second_r6_capture():
    """Two endpoint samples may give an average, but not the specified 1 Hz evidence."""
    world = _World()
    result = profile._collect(world.read, 60, 60, 100, lambda row: None,
                              clock=lambda: world.now, sleep=world.sleep, idle_confirmed=True)
    assert result['complete'] and result['cpu_mean_percent'] == 3
    assert not result['r6_eligible']
    assert result['r6_at_or_above_three_percent'] is None


@pytest.mark.parametrize('error', [
    FileNotFoundError('gone'), PermissionError('denied'),
    ValueError('bad stat'), KeyboardInterrupt(),
])
def test_interrupted_or_unreadable_window_is_incomplete(error):
    """Process loss, permission failure and interruption cannot become zero CPU evidence."""
    world = _World()
    world.failure = error
    result, rows = world.collect(idle_confirmed=True)
    assert not result['complete'] and not result['r6_eligible']
    assert result['reason'] and len(rows) == 2
    assert result['cpu_mean_percent'] == 3


@pytest.mark.parametrize('fault', ['identity', 'cpu', 'clock'])
def test_pid_reuse_counter_reset_and_clock_regression_are_not_aggregated(fault):
    """Do not combine two process lifetimes or invalid counter intervals."""
    world, rows = _World(), []

    def emit(row):
        rows.append(row)
        if len(rows) == 2:
            if fault == 'identity':
                world.start += 1
            elif fault == 'cpu':
                world.rate = 0
            else:
                world.now = -2

    result = profile._collect(world.read, 60, 1, 100, emit,
                              clock=lambda: world.now, sleep=world.sleep, idle_confirmed=True)
    assert not result['complete'] and result['reason']
    assert len(rows) == 2 and result['r6_at_or_above_three_percent'] is None


def test_stat_parser_handles_parentheses_without_recording_process_command(tmp_path):
    """Read fields after the final comm parenthesis; do not expose argv or process names."""
    fields = ['S'] + ['0'] * 21
    fields[11], fields[12], fields[17], fields[19], fields[21] = '17', '9', '4', '123', '2'
    directory = tmp_path / '42'
    directory.mkdir()
    path = directory / 'stat'
    path.write_text('42 (private ) process (name)) ' + ' '.join(fields))
    result = profile._read_counters(42, tmp_path)
    assert (result.cpu_ticks, result.start_ticks, result.threads) == (26, 123, 4)
    assert result.rss_bytes == 2 * os.sysconf('SC_PAGE_SIZE')
    assert 'private' not in repr(result)
    fields[0] = 'Z'
    path.write_text('42 (dead) ' + ' '.join(fields))
    with pytest.raises(ValueError, match='exited'):
        profile._read_counters(42, tmp_path)


@pytest.mark.parametrize('text', ['42 bad', '43 (wrong) S', '42 (short) S'])
def test_bad_stat_is_rejected(tmp_path, text):
    """Missing fields and mismatched process IDs fail before producing a sample."""
    (tmp_path / '42').mkdir()
    (tmp_path / '42/stat').write_text(text)
    with pytest.raises(ValueError):
        profile._read_counters(42, tmp_path)


def test_cli_records_only_current_test_process_and_refuses_overwrite(tmp_path):
    """A tiny self-profile tests real proc reads without ROS, children or target signals."""
    output = tmp_path / 'evidence'
    args = ['--pid', str(os.getpid()), '--output', str(output),
            '--seconds', '0.02', '--interval', '0.01']
    assert profile.main(args) == 0
    before = (output / 'summary.json').read_bytes()
    result = json.loads(before)
    assert result['complete'] and not result['r6_eligible']
    assert result['r6_at_or_above_three_percent'] is None
    metadata = json.loads((output / 'metadata.json').read_text())
    assert metadata['pid'] == os.getpid() and 'cmdline' not in metadata
    with (output / 'samples.csv').open() as handle:
        rows = list(csv.DictReader(handle))
    assert rows[0]['cpu_percent'] == '' and len(rows) > 1
    assert profile.main(args) == 2
    assert (output / 'summary.json').read_bytes() == before


@pytest.mark.parametrize('extra', [
    ['--seconds', 'nan'], ['--interval', '0'], ['--pid', '0'],
    ['--seconds', '0.1', '--interval', '1'],
])
def test_bad_cli_arguments_create_no_output(tmp_path, extra):
    """Invalid ranges cannot silently schedule an unbounded or empty measurement."""
    output = tmp_path / 'invalid'
    with pytest.raises(SystemExit) as result:
        profile.main(['--pid', '42', '--output', str(output), *extra])
    assert result.value.code == 2 and not output.exists()


@pytest.mark.parametrize('error,code', [
    (FileNotFoundError('gone'), 1), (KeyboardInterrupt(), 130),
])
def test_cli_initial_failure_is_incomplete(tmp_path, monkeypatch, error, code):
    """No initial sample is different from a measured idle zero, including Ctrl+C."""
    def fail(pid):
        raise error

    monkeypatch.setattr(profile, '_read_counters', fail)
    output = tmp_path / 'failed'
    assert profile.main(['--pid', '42', '--output', str(output), '--confirm-idle']) == code
    result = json.loads((output / 'summary.json').read_text())
    assert not result['complete'] and not result['r6_eligible']
    assert result['cpu_mean_percent'] is None and result['rows'] == 0


def test_irregular_intervals_use_time_weighted_cpu_not_mean_of_percentages():
    """A long scheduling delay must not give a short high-CPU interval equal weight."""
    stamps, ticks = iter([0, 1, 60]), iter([0, 50, 100])
    rows = []
    result = profile._collect(
        lambda: profile._Counters(100, next(ticks), 4096, 1, 'S'),
        60, 1, 100, rows.append, clock=lambda: next(stamps), sleep=lambda seconds: None,
        idle_confirmed=True)
    assert result['complete'] and result['cpu_mean_percent'] == pytest.approx(100 / 60)
    assert rows[1]['cpu_percent'] == 50
    assert result['cpu_mean_percent'] != pytest.approx(
        sum(row['cpu_percent'] for row in rows[1:]) / 2)
    assert not result['r6_eligible']
