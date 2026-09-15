"""Host wiring uses fake processes/platform commands and a private DDS directory."""

from dataclasses import replace
import os
import signal

import pytest
import yaml

from nav_bringup.navrun.dds_hygiene import MappingView
from nav_bringup.navrun.platform import PlatformError, PlatformStep
from nav_bringup.navrun.preflight import plan_preflight, PreflightError, ProcessInfo
from nav_bringup.navrun.recorded_startup import StartupInterrupted
from nav_bringup.navrun.runtime_commands import effective_profile
from nav_bringup.navrun.runtime_host import bind_host_steps
from nav_bringup.recorder.events import EventLog, read_events

from test_runtime_commands import _plan


SELF = ProcessInfo(100, 1, 1.0, ('navrun',), 1000)


def _steps(tmp_path, **kwargs):
    directory = tmp_path / 'private_dds'
    directory.mkdir(exist_ok=True)
    callbacks = dict(platform_runner=lambda *args: (0, ''), snapshot=lambda: (SELF,),
                     send=lambda *args: pytest.fail('unexpected signal'),
                     observe=lambda: MappingView(frozenset(), True))
    callbacks.update(kwargs)
    return {step.name: step.execute for step in bind_host_steps(directory, **callbacks)}


def test_binding_itself_does_not_invoke_any_adapter(tmp_path):
    """Constructing the host sequence has no read, subprocess or removal effects."""
    def forbidden(*args):
        pytest.fail('construction must not call adapters')

    result = bind_host_steps(tmp_path / 'not-created', forbidden, forbidden, forbidden, forbidden)
    assert [step.name for step in result] == ['platform', 'preflight', 'dds']
    assert not (tmp_path / 'not-created').exists()


def test_platform_partial_success_is_recorded_before_failure(tmp_path):
    """The second failing command cannot erase evidence that the first was applied."""
    plan = replace(_plan(tmp_path), platform=(
        PlatformStep('maxn', ('/fake/maxn',), 30),
        PlatformStep('clocks', ('/fake/clocks',), 30)))
    calls = []

    def runner(argv, timeout):
        calls.append(argv)
        return (0, 'applied') if len(calls) == 1 else (126, 'denied')

    events = EventLog(tmp_path / 'events.jsonl')
    with pytest.raises(PlatformError) as failed:
        _steps(tmp_path, platform_runner=runner)['platform'](plan, events, lambda: None)
    assert [result.status for result in failed.value.results] == ['succeeded', 'failed']
    assert [event.data['status'] for event in read_events(events.path)[0]] == [
        'succeeded', 'failed']


def test_fake_residual_is_cleaned_using_only_original_identity(tmp_path):
    """Preflight binding delegates exact identity, then rechecks the resulting graph."""
    plan = _plan(tmp_path)
    profile = effective_profile(plan)
    profile['preflight'] = {'kill_patterns': ['fixture_residual']}
    target = ProcessInfo(101, 1, 2.0, ('fixture_residual',), 1000)
    current = [SELF, target]
    generation = plan_preflight(profile, current, SELF.pid)
    artifacts = dict(plan.bundle.artifacts)
    artifacts['params/inputs/robot.yaml'] = yaml.safe_dump(profile).encode()
    plan = replace(plan, preflight=generation,
                   bundle=replace(plan.bundle, artifacts=tuple(artifacts.items())))
    sent = []

    def send(expected, signum):
        sent.append((expected, signum))
        current.remove(expected)
        return True

    steps = _steps(tmp_path, snapshot=lambda: tuple(current), send=send)
    steps['preflight'](plan, EventLog(tmp_path / 'events.jsonl'), lambda: None)
    assert sent == [(target, signal.SIGTERM)]


def test_new_driver_cannot_silently_change_the_prepared_chassis_choice(tmp_path):
    """A driver appearing after planning causes rejection without any new signal."""
    plan = _plan(tmp_path)
    driver = ProcessInfo(101, 1, 2.0, ('fake_chassis',), 1000)
    steps = _steps(tmp_path, snapshot=lambda: (SELF, driver))
    with pytest.raises(PreflightError):
        steps['preflight'](plan, EventLog(tmp_path / 'events.jsonl'), lambda: None)


@pytest.mark.parametrize('complete', [True, False])
def test_dds_uses_private_directory_and_retains_incomplete_evidence(tmp_path, complete):
    """Only a private fixture with complete mapping evidence can be removed."""
    steps = _steps(tmp_path, observe=lambda: MappingView(frozenset(), complete))
    path = tmp_path / 'private_dds/fastrtps_1234567890abcdef'
    path.write_bytes(b'private fixture')
    assert path.stat().st_uid == os.getuid()
    events = EventLog(tmp_path / 'events.jsonl')
    steps['dds'](_plan(tmp_path), events, lambda: None)
    assert path.exists() is not complete
    assert read_events(events.path)[0][-1].severity == ('info' if complete else 'warn')


@pytest.mark.parametrize('name', ['platform', 'preflight', 'dds'])
def test_interruption_is_not_rewritten_as_an_io_or_cleanup_failure(tmp_path, name):
    """Explicit supervisor interruption crosses backend exception handlers intact."""
    events = EventLog(tmp_path / 'events.jsonl')
    with pytest.raises(StartupInterrupted) as stopped:
        _steps(tmp_path)[name](_plan(tmp_path), events, lambda: signal.SIGTERM)
    assert stopped.value.signum == signal.SIGTERM
