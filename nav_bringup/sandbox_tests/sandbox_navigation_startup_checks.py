"""Actual Nav2 startup and complete effective dumps without goals, hardware or motion."""

import json
import os
from pathlib import Path
import signal
import sys
import time

import pytest
import yaml

from conftest import require_sandbox
from nav_bringup.navcfg.live_inputs import load_live_spec
from nav_bringup.navrun.locking import RunLock
from nav_bringup.navrun.owned_process import OwnedProcess
from nav_bringup.recorder.events import read_events
from nav_bringup.recorder.manifest import read_manifest
from nav_bringup.recorder.param_dump import node_file_name


def _flatten(values, parts=()):
    result = {}
    for key, value in values.items():
        path = (*parts, key)
        if isinstance(value, dict):
            result.update(_flatten(value, path))
        else:
            result['.'.join(path)] = value
    return result


@pytest.mark.parametrize('mode', ['tour', 'precise'])
def test_recorded_navigation_wait_launch_check_dump_and_shutdown(tmp_path, mode):
    """The real stack consumes frozen example inputs and exits without touching providers."""
    require_sandbox()
    root = tmp_path / 'private_run'
    log = tmp_path / 'wrapper.log'
    provider_log = tmp_path / 'provider.log'
    owner = provider = None
    try:
        owner = OwnedProcess([
            sys.executable, str(Path(__file__).with_name('recorded_navigation_fixture.py')),
            '--root', str(root), '--mode', mode], os.environ, log)
        deadline = time.monotonic() + 25
        run = None
        while owner.poll() is None and time.monotonic() < deadline:
            link = root / 'runs/latest'
            if link.exists():
                run = link.resolve()
                events = read_events(run / 'events.jsonl')[0]
                if any(event.event_type == 'localization_waiting' for event in events):
                    break
            time.sleep(0.05)
        assert run is not None and owner.poll() is None, log.read_text()
        events = read_events(run / 'events.jsonl')[0]
        assert any(event.event_type == 'localization_waiting' for event in events)
        assert all(event.data['name'] == 'localization' for event in events
                   if event.event_type == 'process_started')
        provider = OwnedProcess([
            sys.executable, str(Path(__file__).with_name('navigation_provider_fixture.py')),
            '--profile', str(run / 'params/inputs/robot.yaml')], os.environ, provider_log)
        deadline = time.monotonic() + 210
        completed = False
        while owner.poll() is None and time.monotonic() < deadline:
            assert provider.poll() is None, provider_log.read_text()
            events = read_events(run / 'events.jsonl')[0]
            if any(event.event_type == 'parameter_dump_finished' for event in events):
                completed = True
                break
            time.sleep(0.1)
        assert completed, log.read_text()[-16000:]
        assert any(event.event_type == 'startup_ready' for event in events)
        assert any(event.event_type == 'interface_contracts_ready' for event in events)
        assert any((run / 'logs/ros').iterdir()), 'ROS logs must enter the owned run record'
        assert str(run / 'logs/ros') in (run / 'logs/launch.log').read_text()
        report = yaml.safe_load((run / 'params/effective/dump_report.yaml').read_text())
        assert not report['failed'] and not report['list_failed'], report
        assert not report['duplicates'], report
        spec, _ = load_live_spec(run)
        assert set(spec.parameters).issubset(report['dumped']), report
        missing, mismatched = [], []
        for node, expected in spec.parameters.items():
            document = yaml.safe_load(
                (run / 'params/effective' / node_file_name(node)).read_text())
            assert set(document) == {node}, node
            # Dumps can contain legacy leading-dot names. They are evidence,
            # not source selectors to validate with the stricter input parser.
            values = _flatten(document[node]['ros__parameters'])
            assert values, node
            missing += [f'{node}/{key}' for key in expected if key not in values]
            mismatched += [f'{node}/{key}' for key in expected if key in values and
                           json.dumps(values[key]) != json.dumps(expected[key])]
        # Preserve the actual findings for review; do not manufacture mock parameters.
        findings = [event.data for event in events if event.source == 'navcfg'
                    and event.severity == 'error']
        print('actual live findings:', json.dumps(findings, ensure_ascii=False))
        errors = [event for event in events if event.source == 'navcfg'
                  and event.severity == 'error']
        assert all(event.event_type in ('parameter_missing', 'parameter_mismatch')
                   for event in errors), findings
        assert sorted(missing) == sorted(event.data['message'].split(': ', 1)[0]
                                         for event in errors
                                         if event.event_type == 'parameter_missing')
        assert sorted(mismatched) == sorted(event.data['message'].split(': ', 1)[0]
                                            for event in errors
                                            if event.event_type == 'parameter_mismatch')
        assert not mismatched, 'declared rendered values must match the actual Nav2 dump'
        print('parameter comparison:', json.dumps({
            'mode': mode, 'nodes': len(spec.parameters),
            'declared_matches': sum(map(len, spec.parameters.values())) - len(missing),
            'undeclared_reported': len(missing)}))
        os.kill(owner.pid, signal.SIGINT)
        deadline = time.monotonic() + 60
        while owner.poll() is None and time.monotonic() < deadline:
            time.sleep(0.1)
        assert owner.poll() == 130, log.read_text()[-10000:]
        assert provider.poll() is None, 'navigation must not stop external localization'
        assert read_manifest(run / 'manifest.yaml')['exit_code'] == 130
        events = read_events(run / 'events.jsonl')[0]
        assert sum(event.event_type == 'stop_zero_sent' for event in events) == 2
        assert all(not Path('/proc', str(event.data['pid'])).exists()
                   for event in events if event.event_type == 'process_started')
        with RunLock(root / 'lease'):
            pass
    finally:
        if owner is not None:
            owner.close(90)
        if provider is not None:
            provider.close()
    reports = [json.loads(line) for line in provider_log.read_text().splitlines()
               if line.startswith('{')]
    assert len(reports) == 1 and reports[0]['commands'] >= 5
    assert reports[0]['nonzero'] == 0


def test_real_nav2_strict_check_reports_legacy_parameters_and_stops(tmp_path):
    """Existing inert Nav2 settings remain errors, not startup-success exemptions."""
    require_sandbox()
    root, log = tmp_path / 'private_run', tmp_path / 'wrapper.log'
    owner = provider = None
    try:
        owner = OwnedProcess([
            sys.executable, str(Path(__file__).with_name('recorded_navigation_fixture.py')),
            '--root', str(root), '--strict'], os.environ, log)
        deadline = time.monotonic() + 25
        run = None
        while owner.poll() is None and time.monotonic() < deadline:
            if (root / 'runs/latest').exists():
                run = (root / 'runs/latest').resolve()
                if any(event.event_type == 'localization_waiting' for event in
                       read_events(run / 'events.jsonl')[0]):
                    break
            time.sleep(0.05)
        assert run is not None and owner.poll() is None, log.read_text()
        provider = OwnedProcess([
            sys.executable, str(Path(__file__).with_name('navigation_provider_fixture.py')),
            '--profile', str(run / 'params/inputs/robot.yaml')],
            os.environ, tmp_path / 'provider.log')
        deadline = time.monotonic() + 60
        while owner.poll() is None and time.monotonic() < deadline:
            assert provider.poll() is None, (tmp_path / 'provider.log').read_text()
            time.sleep(0.1)
        assert owner.poll() == 1, log.read_text()[-16000:]
        events = read_events(run / 'events.jsonl')[0]
        errors = [event for event in events if event.source == 'navcfg'
                  and event.severity == 'error']
        assert errors and all(event.event_type == 'parameter_missing' for event in errors)
        assert not any(event.event_type == 'startup_ready' for event in events)
        assert not any(event.data.get('name') == 'parameter_dump' for event in events)
        assert sum(event.event_type == 'stop_zero_sent' for event in events) == 2
        assert provider.poll() is None
        assert read_manifest(run / 'manifest.yaml')['exit_code'] == 1
        assert all(not Path('/proc', str(event.data['pid'])).exists()
                   for event in events if event.event_type == 'process_started')
        with RunLock(root / 'lease'):
            pass
    finally:
        if owner is not None:
            owner.close(90)
        if provider is not None:
            provider.close()
    reports = [json.loads(line) for line in (tmp_path / 'provider.log').read_text().splitlines()
               if line.startswith('{')]
    assert len(reports) == 1 and reports[0]['nonzero'] == 0
