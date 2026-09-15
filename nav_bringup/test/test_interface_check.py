"""Validate the subprocess contract without importing perception runtime or starting ROS."""

import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from nav_bringup.navcfg.interface_check import parse_report
from nav_bringup.navcfg import interface_check


def _report(status='ready', findings=()):
    return {'schema_version': 1, 'status': status, 'findings': list(findings),
            'evidence': {'transforms': [], 'publishers': {}, 'subscribers': {},
                         'samples': ['/odom'], 'types': {}, 'cloud_frame': None}}


def test_valid_report_retains_findings_and_window_evidence():
    """A typed error remains an error even though localization is ready."""
    report = _report(findings=[{'severity': 'error', 'code': 'chassis_cmd_missing',
                               'message': 'no compatible subscriber'}])
    result = parse_report(json.dumps(report), 1)
    assert result.status == 1
    assert result.findings[0].code == 'chassis_cmd_missing'
    assert result.evidence['samples'] == ['/odom']


@pytest.mark.parametrize('change,exit_code', [
    ({'schema_version': True}, 0), ({'schema_version': 2}, 0),
    ({'status': 'unknown'}, 0), ({'findings': []}, 1), ({'evidence': None}, 0),
    ({'findings': [{'severity': 'error', 'code': 'bad', 'message': 'bad'}]}, 0),
])
def test_inconsistent_report_cannot_be_success(change, exit_code):
    """Version, evidence and exit status must agree before accepting a subprocess result."""
    with pytest.raises(ValueError):
        parse_report(json.dumps(dict(_report(), **change)), exit_code)


@pytest.mark.parametrize('text', ['nothing', '{broken', '{}\n{}'])
def test_missing_or_multiple_results_are_rejected(text):
    """A crashed or malformed observer cannot be reported as a successful check."""
    with pytest.raises(ValueError):
        parse_report(text, 0)


@pytest.fixture
def helper(monkeypatch):
    """Model a same-group leaf child and temporary output, without launching anything."""
    state = SimpleNamespace(now=0.0, running=False, stubborn=False, calls=[], code=0,
                            output=json.dumps(_report()).encode(), path=None, command=None)

    class Child:
        def __init__(self, command, **kwargs):
            assert kwargs['start_new_session'] is False
            assert kwargs['close_fds'] is True
            state.command = command
            state.path = Path(kwargs['stdout'].name)
            kwargs['stdout'].write(state.output)
            self.returncode = None if state.running else state.code

        def poll(self):
            return self.returncode

        def terminate(self):
            state.calls.append('terminate')
            if not state.stubborn:
                self.returncode = 143

        def kill(self):
            state.calls.append('kill')
            self.returncode = -9

        def wait(self, timeout=None):
            if self.returncode is None:
                raise subprocess.TimeoutExpired('fixture', timeout)
            return self.returncode

    def sleep(duration):
        state.now += duration

    monkeypatch.setattr(interface_check, 'resolve_executable', lambda *_: '/fixture/probe')
    monkeypatch.setattr(interface_check.subprocess, 'Popen', Child)
    monkeypatch.setattr(interface_check.time, 'monotonic', lambda: state.now)
    monkeypatch.setattr(interface_check.time, 'sleep', sleep)
    return state


def test_child_uses_frozen_inputs_and_does_not_escape_owner_group(helper, tmp_path):
    """No private ROS environment, package runtime import or arbitrary profile is substituted."""
    result = interface_check.check_interface(tmp_path, 0.1, lambda: False)
    assert result.status == 0
    assert helper.command == [
        '/fixture/probe', '--profile', str(tmp_path / 'params/inputs/robot.yaml'),
        '--mode', str(tmp_path / 'params/inputs/mode.yaml'),
        '--sample-sec', '0.1', '--wait-timeout', '5']
    assert not helper.path.exists()
    assert helper.calls == []


def test_interrupted_child_is_closed_before_temporary_output_is_removed(helper, tmp_path):
    """Cancellation returns no successful result and terminates only the owned child."""
    helper.running = True
    assert interface_check.check_interface(tmp_path, 0.1, lambda: helper.now > 0.1) is None
    assert helper.calls == ['terminate'] and not helper.path.exists()


def test_stuck_observer_has_deadline_and_kill_escalation(helper, tmp_path):
    """A broken independent observer cannot leave an unbounded silent startup wait."""
    helper.running = helper.stubborn = True
    with pytest.raises(RuntimeError, match='budget'):
        interface_check.check_interface(tmp_path, 0.1, lambda: False)
    assert helper.calls == ['terminate', 'kill']
    assert not helper.path.exists()


def test_excessive_output_is_rejected_even_after_success_exit(helper, tmp_path):
    """A zero child exit cannot bypass bounded report memory."""
    helper.output = b'x' * (1024 * 1024 + 1)
    with pytest.raises(RuntimeError, match='budget'):
        interface_check.check_interface(tmp_path, 0.1, lambda: False)
    assert not helper.path.exists()
