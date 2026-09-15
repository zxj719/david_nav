"""Pure CLI/configuration tests with an injected observer and deterministic clock."""

from dataclasses import replace
import json
import signal
from types import SimpleNamespace

import pytest
import yaml

from nav_perception_iface import contract_cli, contract_probe
from nav_perception_iface.configuration import load_contract_profile
from test_contracts import _profile, _ready


def test_contract_profile_overlay_preserves_sources_and_replaces_sequences(tmp_path):
    """Independent subprocess inputs retain the established recursive overlay semantics."""
    robot, mode = tmp_path / 'robot.yaml', tmp_path / 'mode.yaml'
    robot.write_text(yaml.safe_dump(dict(_profile(), schema_version=1, example=[1, 2])))
    mode.write_text(yaml.safe_dump({'schema_version': 1, 'mode': 'tour', 'profile': {
        'localization': {'odom_topic': '/other_odom'}, 'example': [3]}}))
    before = robot.read_bytes(), mode.read_bytes()
    profile = load_contract_profile(robot, mode)
    assert profile['localization']['odom_topic'] == '/other_odom'
    assert profile['chassis']['odom_topic'] == '/wheel_odom'
    assert profile['example'] == [3]
    assert (robot.read_bytes(), mode.read_bytes()) == before


@pytest.mark.parametrize('document', [[], {'schema_version': True}, {'schema_version': 2}])
def test_invalid_contract_documents_fail_without_ros(tmp_path, document):
    """Malformed source data cannot be interpreted as a default robot profile."""
    path = tmp_path / 'robot.yaml'
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError):
        load_contract_profile(path)


@pytest.fixture
def observation(monkeypatch):
    """Install fake observation, signal ownership and time without initializing ROS."""
    state = SimpleNamespace(now=0.0, ready=True, snapshot=_ready(), sampled=False,
                            closed=False, initialized=False, handlers={}, interrupt=None,
                            close_error=False)

    class Probe:
        def __init__(self, profile, **kwargs):
            pass

        def __enter__(self):
            state.initialized = True
            return self

        def __exit__(self, *_):
            state.closed = True
            if state.close_error:
                raise RuntimeError('context cleanup failed')

        def localized(self):
            return state.ready

        def spin(self, duration):
            state.now += duration
            if state.interrupt:
                state.handlers[state.interrupt](state.interrupt, None)

        def sample(self, duration, stopped):
            state.sampled = True
            return state.snapshot

    def register(signum, handler):
        old = state.handlers.get(signum, 'original')
        state.handlers[signum] = handler
        return old

    monkeypatch.setattr(contract_cli, 'load_contract_profile', lambda *_: _profile())
    monkeypatch.setattr(contract_cli.time, 'monotonic', lambda: state.now)
    monkeypatch.setattr(contract_cli.signal, 'signal', register)
    monkeypatch.setattr(contract_probe, 'ContractProbe', Probe)
    monkeypatch.setattr(contract_probe.rclpy, 'init', lambda **_: pytest.fail('ROS init'))
    return state


def test_ready_cli_reports_actual_evidence_as_versioned_json(observation, capsys):
    """One bounded check emits machine-readable evidence and restores signal handlers."""
    assert contract_cli.main(['--profile', '/not-read']) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['schema_version'] == 1 and report['status'] == 'ready'
    assert '/loc_odom' in report['evidence']['samples']
    assert report['findings'] == []
    assert observation.sampled and observation.closed
    assert set(observation.handlers.values()) == {'original'}


def test_silent_odom_publisher_is_not_success(observation, capsys):
    """A discoverable endpoint without data must fail the contract after localization."""
    observation.snapshot = replace(_ready(), samples=frozenset({'/wheel_odom'}))
    assert contract_cli.main(['--profile', '/not-read']) == 1
    assert 'localization_odom_missing' in capsys.readouterr().out


def test_cleanup_failure_cannot_emit_a_success_report_first(observation, capsys):
    """The single result is published only after all observer resources are released."""
    observation.close_error = True
    assert contract_cli.main(['--profile', '/not-read']) == 2
    report = json.loads(capsys.readouterr().out)
    assert report['status'] == 'error'


def test_waiting_cli_does_not_sample_odom(observation, capsys):
    """Manual localization remains pending rather than failing on nonexistent odometry."""
    observation.ready = False
    assert contract_cli.main(['--profile', '/not-read', '--wait-timeout', '0.2']) == 3
    output = capsys.readouterr()
    assert '等待在 RViz' in output.err
    assert json.loads(output.out)['status'] == 'waiting'
    assert not observation.sampled and observation.closed


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM, signal.SIGHUP])
def test_waiting_interruption_closes_only_observer(observation, signum):
    """Every supported signal preserves its exit status without starting data sampling."""
    observation.ready = False
    observation.interrupt = signum
    assert contract_cli.main(['--profile', '/not-read']) == 128 + signum
    assert observation.closed and not observation.sampled
    assert set(observation.handlers.values()) == {'original'}


@pytest.mark.parametrize('option,value', [
    ('--sample-sec', '0'), ('--sample-sec', 'nan'), ('--wait-timeout', '-1')])
def test_bad_windows_fail_before_observer(observation, option, value):
    """Invalid bounds never initialize a context or wait indefinitely."""
    assert contract_cli.main(['--profile', '/not-read', option, value]) == 2
    assert not observation.initialized
