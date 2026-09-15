"""Exercise command status and cleanup with an injected observer, never real ROS."""

from dataclasses import replace
import signal
from types import SimpleNamespace

import pytest

from nav_bringup.navcfg import cli, live_cli, live_ros
from nav_bringup.navcfg.live_check import Finding, LiveSnapshot, LiveSpec
from nav_bringup.navcfg.interface_check import InterfaceResult


@pytest.fixture
def observer(monkeypatch):
    """Replace every live interaction and clock before dispatching the real CLI."""
    state = SimpleNamespace(now=0.0, initialized=False, closed=False, localized=True,
                            snapshot=LiveSnapshot(True, 0.1, {}, {}, {}),
                            handlers={}, queried=False)

    class Probe:
        def __init__(self, spec):
            state.initialized = True

        def __enter__(self):
            return self

        def __exit__(self, *_):
            state.closed = True

        def localized(self):
            return state.localized

        def spin(self, duration):
            state.now += duration

        def snapshot(self, *_):
            state.queried = True
            return state.snapshot

    def register(signum, callback):
        previous = state.handlers.get(signum, 'original')
        state.handlers[signum] = callback
        return previous

    monkeypatch.setattr(live_cli, 'load_live_spec',
                        lambda _: (LiveSpec('world', 'wheel', 'axle', (), {}), ()))
    monkeypatch.setattr(live_cli.time, 'monotonic', lambda: state.now)
    monkeypatch.setattr(live_cli.signal, 'signal', register)
    monkeypatch.setattr(live_ros, 'RosProbe', Probe)
    monkeypatch.setattr(live_cli, 'check_interface', lambda *_: InterfaceResult(0, (), {}))
    monkeypatch.setattr(live_ros.rclpy, 'init', lambda **_: pytest.fail('ROS init forbidden'))
    return state


def _run(*args):
    return cli.main(['check', '--run-dir', '/not-read-by-fake', *args])


def test_live_requires_explicit_opt_in(observer):
    """Merely selecting check must not initialize ROS."""
    assert _run() == 2
    assert not observer.initialized


@pytest.mark.parametrize('option,value', [
    ('--sample-sec', '0'), ('--parameter-timeout', '-1'), ('--wait-timeout', 'nan')])
def test_invalid_bounds_fail_before_ros(observer, option, value):
    """No malformed timing input can turn a bounded diagnostic into an accidental wait."""
    assert _run('--live', option, value) == 2
    assert not observer.initialized


@pytest.mark.parametrize('strict,expected', [(False, 0), (True, 1)])
def test_strict_errors_and_normal_diagnostic_exit(observer, capsys, strict, expected):
    """Errors are always printed; strictness only determines the supervisor exit code."""
    observer.snapshot = replace(observer.snapshot, tf_age_sec=None)
    assert _run('--live', *(['--strict'] if strict else [])) == expected
    assert 'navigation_tf_missing' in capsys.readouterr().out
    assert observer.closed
    assert all(handler == 'original' for handler in observer.handlers.values())


def test_warn_does_not_fail_strict_and_can_write_owned_event_log(observer, tmp_path):
    """TF age warnings retain warn-only policy and an explicit diagnostic event trail."""
    observer.snapshot = replace(observer.snapshot, tf_age_sec=1.0)
    events = tmp_path / 'events.jsonl'
    assert _run('--live', '--strict', '--events', str(events)) == 0
    assert 'localization_ready' in events.read_text()
    assert 'navigation_tf_stale' in events.read_text()


def test_bounded_unlocalized_check_returns_pending(observer, capsys):
    """Missing manual initial pose is neither a successful check nor a startup failure."""
    observer.localized = False
    assert _run('--live', '--strict', '--wait-timeout', '0.5') == 3
    assert '等待在 RViz 手动给初始位姿' in capsys.readouterr().out
    assert observer.closed


def test_lost_localization_during_collection_is_pending(observer):
    """Initialization loss cannot produce a zero-error success result."""
    observer.snapshot = replace(observer.snapshot, localized=False)
    assert _run('--live') == 3


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM])
def test_signal_during_wait_cleans_up_without_followup_queries(observer, monkeypatch, signum):
    """Only the check process is interrupted; it never sends a stop to localization."""
    def interrupted(*_):
        observer.handlers[signum](signum, None)
        return False

    monkeypatch.setattr(live_cli, 'wait_localization', interrupted)
    assert _run('--live') == 128 + signum
    assert observer.closed


def test_initial_warnings_remain_visible(observer, monkeypatch, capsys):
    """Dormant rendered parameter selectors do not disappear from runtime reports."""
    monkeypatch.setattr(live_cli, 'load_live_spec', lambda _: (
        LiveSpec('world', 'wheel', 'axle', (), {}),
        (Finding('warn', 'inactive_parameter_selector', 'unused'),)))
    assert _run('--live', '--strict') == 0
    assert 'inactive_parameter_selector' in capsys.readouterr().out


def test_prelaunch_wait_does_not_query_parameters_or_contract_data(observer, monkeypatch):
    """Only map-to-odom is required before navigation launch can begin."""
    monkeypatch.setattr(live_cli, 'check_interface', lambda *_: pytest.fail('interface probe'))
    assert _run('--live', '--wait-localization-only') == 0
    assert observer.now == 0.0 and observer.closed
    assert not observer.queried


@pytest.mark.parametrize('strict,expected', [(False, 0), (True, 1)])
def test_interface_errors_participate_in_full_check(
        observer, monkeypatch, capsys, strict, expected):
    """Independent module findings use the same strictness policy as parameter checks."""
    monkeypatch.setattr(live_cli, 'check_interface', lambda *_: InterfaceResult(1, (
        Finding('error', 'localization_odom_missing', 'no samples'),), {}))
    assert _run('--live', *(['--strict'] if strict else [])) == expected
    assert 'localization_odom_missing' in capsys.readouterr().out


def test_pending_interface_is_not_success(observer, monkeypatch, capsys):
    """Localization loss during the separate observation remains pending."""
    monkeypatch.setattr(live_cli, 'check_interface', lambda *_: InterfaceResult(3, (), None))
    assert _run('--live') == 3
    assert 'localization_waiting' in capsys.readouterr().out


def test_missing_interface_dependency_is_an_operational_failure(observer, monkeypatch):
    """Absent required observation cannot be silently skipped in non-strict mode."""
    def missing(*_):
        raise LookupError('interface observer not installed')

    monkeypatch.setattr(live_cli, 'check_interface', missing)
    assert _run('--live') == 2
