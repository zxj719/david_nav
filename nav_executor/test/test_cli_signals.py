"""CLI dispatch and signal ownership using injected runtimes and callbacks."""

import asyncio
from pathlib import Path
import signal

import pytest

from nav_executor.app import cli
from nav_executor.app.signals import run_with_signals

PROFILE = Path(__file__).resolve().parents[2] / 'david_config' / 'robot' / 'david.yaml'


def test_help_does_not_parse_or_load_configuration(monkeypatch, capsys):
    """The frozen help is available even with an invalid config path and no ROS runtime."""
    monkeypatch.setattr(cli, 'parse_arguments', lambda args: pytest.fail('parsed help'))
    assert cli.main(['--config', '/missing', '--help']) == 0
    assert capsys.readouterr().out == cli.build_help_text() + '\n'


def test_cli_passes_explicit_profile_and_ros_arguments_to_outer_runtime(monkeypatch):
    """The actual main function resolves config then returns the runtime exit status."""
    observed = []

    async def fake(resolved):
        observed.append(resolved)
        return 143

    monkeypatch.setattr('nav_executor.app.signals.run_with_signals', fake)
    assert cli.main(['--robot-profile', str(PROFILE), '--server-uri', 'ws://configured',
                     '--ros-args', '-r', '__node:=example']) == 143
    assert observed[0].config.server_uri == 'ws://configured'
    assert observed[0].ros_args == ('--ros-args', '-r', '__node:=example')


def test_bad_config_returns_failure_without_running_ros(monkeypatch, capsys):
    """Known CLI errors are readable and cannot reach ROS construction."""
    monkeypatch.setattr('nav_executor.app.signals.run_with_signals',
                        lambda *args: pytest.fail('runtime called'))
    assert cli.main(['--robot-profile', str(PROFILE), '--nav-retry-count', 'bad']) == 1
    assert 'executor failed: ConfigError:' in capsys.readouterr().out


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM, signal.SIGHUP])
def test_signals_request_stop_without_canceling_runtime(monkeypatch, signum):
    """Repeated notifications keep the first exit reason and wait for orderly cleanup."""
    async def scenario():
        handlers, removed, restored, messages = {}, [], [], []
        loop = asyncio.get_running_loop()
        monkeypatch.setattr(loop, 'add_signal_handler',
                            lambda sig, callback, *args: handlers.update({sig: (callback, args)}))
        monkeypatch.setattr(loop, 'remove_signal_handler', removed.append)
        monkeypatch.setattr(signal, 'getsignal', lambda sig: f'old-{sig}')
        monkeypatch.setattr(signal, 'signal', lambda sig, prior: restored.append((sig, prior)))

        async def runner(resolved, stop, emit):
            callback, args = handlers[signum]
            callback(*args)
            callback(*args)
            await stop.wait()
            await asyncio.sleep(0)
            assert removed == []  # Handlers remain active throughout cleanup.

        result = await run_with_signals(None, runner=runner, emit=messages.append)
        assert result == 128 + signum
        assert len(messages) == 1
        assert removed == [signal.SIGINT, signal.SIGTERM, signal.SIGHUP]
        assert restored == [(sig, f'old-{sig}') for sig in removed]

    asyncio.run(scenario())


def test_signal_handlers_restore_after_runtime_error(monkeypatch):
    """A fully cleaned failed run must not leave its process handlers installed."""
    async def scenario():
        loop, removed = asyncio.get_running_loop(), []
        monkeypatch.setattr(loop, 'add_signal_handler', lambda *args: None)
        monkeypatch.setattr(loop, 'remove_signal_handler', removed.append)
        monkeypatch.setattr(signal, 'getsignal', lambda sig: signal.SIG_DFL)
        monkeypatch.setattr(signal, 'signal', lambda *args: None)

        async def broken(*args, **kwargs):
            raise OSError('failed after cleanup')

        with pytest.raises(OSError):
            await run_with_signals(None, runner=broken)
        assert removed == [signal.SIGINT, signal.SIGTERM, signal.SIGHUP]

    asyncio.run(scenario())
