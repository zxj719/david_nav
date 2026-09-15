"""Readiness timeout and argv forwarding tests; no ROS context or exec occurs."""

import pytest

from nav_bringup.launching import ready_exec


def test_timeout_never_starts_executor(monkeypatch) -> None:
    """A false readiness result keeps the executor process absent."""
    monkeypatch.setattr(ready_exec, 'wait_ready', lambda *_: False)
    monkeypatch.setattr(ready_exec.os, 'execv', lambda *_: pytest.fail('must not exec'))
    assert ready_exec.main(['--', '/fake/worker']) == 1


def test_ready_forwards_exact_argv_without_shell(monkeypatch) -> None:
    """Whitespace and shell metacharacters remain ordinary arguments."""
    calls = []
    monkeypatch.setattr(ready_exec, 'wait_ready', lambda *_: True)
    monkeypatch.setattr(ready_exec.os, 'execv', lambda *args: calls.append(args))
    arguments = ['/fake/worker', '--pose-file', '/tmp/a b.json', '$(not executed)']
    assert ready_exec.main(['--', *arguments]) == 0
    assert calls == [(arguments[0], arguments)]


@pytest.mark.parametrize('arguments', [
    ['--timeout', '0', '--', '/fake/worker'], ['--timeout', 'nan', '--', '/fake/worker'],
    ['--timeout', '-1', '--', '/fake/worker'], ['--', 'relative'], [],
])
def test_invalid_gate_configuration_never_initializes_ros(monkeypatch, arguments) -> None:
    """Invalid inputs fail before the read-only ROS probe is created."""
    monkeypatch.setattr(ready_exec, 'wait_ready', lambda *_: pytest.fail('must not probe'))
    with pytest.raises(SystemExit):
        ready_exec.main(arguments)
