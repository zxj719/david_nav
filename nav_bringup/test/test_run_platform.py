"""Platform settings use fake runners only: no sudo or Jetson command is executed."""

import subprocess

import pytest

from nav_bringup.navrun import platform


def _profile(**values):
    return {'platform': dict(jetson=True, maxn=True, jetson_clocks=True, **values)}


def test_command_plan_is_noninteractive_and_preserves_existing_power_mode() -> None:
    """The old MAXN mode zero and clock ordering survive without a password pipe."""
    steps = platform.plan_platform(_profile())
    assert [step.argv for step in steps] == [
        ('/usr/bin/sudo', '-n', '--', '/usr/sbin/nvpmodel', '-m', '0'),
        ('/usr/bin/sudo', '-n', '--', '/usr/bin/jetson_clocks')]
    assert all(step.timeout_sec > 0 for step in steps)


@pytest.mark.parametrize('profile', [{}, {'platform': {}}, {
    'platform': {'jetson': False, 'maxn': False, 'jetson_clocks': False}}])
def test_non_jetson_and_default_profiles_have_no_platform_steps(profile) -> None:
    """Another robot does not run any Jetson-specific command by default."""
    assert platform.plan_platform(profile) == ()


def test_board_specific_power_mode_is_configuration_only() -> None:
    """Different Jetson boards can select their actual MAXN id without editing code."""
    assert platform.plan_platform(_profile(maxn_mode_id=7))[0].argv[-1] == '7'


@pytest.mark.parametrize('values', [
    {'jetson': 'yes'}, {'maxn': 1}, {'jetson_clocks': None},
    {'jetson': False, 'maxn': True}, {'maxn_mode_id': True}, {'maxn_mode_id': -1},
    {'maxn_mode_id': '0; anything'}, {'command_timeout_sec': 0},
    {'command_timeout_sec': float('nan')}, {'command_timeout_sec': float('inf')},
])
def test_invalid_platform_settings_fail_before_runner_is_called(values) -> None:
    """Type errors, bad timeout and inconsistent platform switches are not coerced."""
    profile = _profile()
    profile['platform'].update(values)
    with pytest.raises(ValueError):
        platform.configure_platform(profile, runner=lambda *_: pytest.fail('runner called'))


def test_dry_run_returns_plan_without_invoking_any_command() -> None:
    """Dry-run remains free of host mutations even for David's enabled platform flags."""
    result = platform.configure_platform(
        _profile(), dry_run=True, runner=lambda *_: pytest.fail('runner called'))
    assert [item.status for item in result] == ['planned', 'planned']


def test_success_runs_power_before_clocks_and_records_results() -> None:
    """The supervisor can record each successful step with its exact invocation."""
    calls = []

    def run(argv, timeout):
        calls.append((argv, timeout))
        return 0, 'configured'

    result = platform.configure_platform(_profile(command_timeout_sec=8.0), runner=run)
    assert [item.name for item in result] == ['maxn', 'jetson_clocks']
    assert [item.status for item in result] == ['succeeded', 'succeeded']
    assert all(timeout == 8.0 for _, timeout in calls)


def test_first_failure_stops_later_steps_and_exposes_partial_results() -> None:
    """Permission failure is reported, not followed by more platform modifications."""
    calls = []

    def run(argv, timeout):
        calls.append(argv)
        return 1, 'sudo: a password is required'

    with pytest.raises(platform.PlatformError) as caught:
        platform.configure_platform(_profile(), runner=run)
    assert len(calls) == 1
    assert caught.value.results[-1].return_code == 1
    assert caught.value.results[-1].status == 'failed'
    assert 'sudoers' in str(caught.value)


def test_clock_failure_retains_successful_power_step() -> None:
    """Failure reporting does not claim that an already applied power setting rolled back."""
    outcomes = iter([(0, 'power configured'), (124, 'clock deadline')])
    with pytest.raises(platform.PlatformError) as caught:
        platform.configure_platform(_profile(), runner=lambda *_: next(outcomes))
    assert [item.status for item in caught.value.results] == ['succeeded', 'failed']
    assert caught.value.results[-1].return_code == 124


def test_subprocess_adapter_uses_no_shell_and_no_stdin(monkeypatch) -> None:
    """Even the production command adapter is tested through a fake subprocess call."""
    def fake(argv, **kwargs):
        assert argv == ['safe', 'argument with spaces']
        assert kwargs['stdin'] == subprocess.DEVNULL
        assert kwargs.get('shell', False) is False
        assert kwargs['timeout'] == 2.0
        return subprocess.CompletedProcess(argv, 0, stdout='out', stderr='err')

    monkeypatch.setattr(subprocess, 'run', fake)
    assert platform.run_platform_command(('safe', 'argument with spaces'), 2.0) == (0, 'outerr')


@pytest.mark.parametrize('error, code', [(FileNotFoundError(), 127), (
    subprocess.TimeoutExpired(['fake'], 1), 124), (PermissionError(), 126)])
def test_subprocess_errors_are_bounded_results(monkeypatch, error, code: int) -> None:
    """Missing binaries, denied execution and timeouts produce explicit failure codes."""
    def fake(*_args, **_kwargs):
        raise error

    monkeypatch.setattr(subprocess, 'run', fake)
    result, _ = platform.run_platform_command(('fake',), 1.0)
    assert result == code
