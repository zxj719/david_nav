"""Guard and shutdown sequencing with fake ROS, locks, terminals and publishers."""

from contextlib import nullcontext
import signal
from types import SimpleNamespace

import pytest

from chassis_teleop import node as runtime


@pytest.fixture
def runtime_hooks(monkeypatch):
    """Record every startup/cleanup action without acquiring locks or initializing ROS."""
    history = []
    alive = [False]
    node = SimpleNamespace(
        config=SimpleNamespace(tty_device='/unused/tty', period_sec=0.05),
        guard_topics=('/drive', '/nav', '/safe'), discovery_sec=0.1,
        get_publishers_info_by_topic=lambda _: [],
        armed=False)

    def arm():
        history.append('arm')
        node.armed = True

    def stop_burst():
        if node.armed:
            history.append('zero_burst')

    node.arm = arm
    node.publish_stop_burst = stop_burst
    node.destroy_node = lambda: history.append('destroy')
    node.handle_key = lambda key: 'quit' if key == 'q' else 'ignored'
    node.publish_current_command = lambda: history.append('command')

    class Lease:
        def __enter__(self):
            history.append('lock')

        def __exit__(self, *_):
            history.append('unlock')

    class Terminal:
        def __enter__(self):
            history.append('tty')
            return SimpleNamespace(read_key=lambda _: 'q')

        def __exit__(self, *_):
            history.append('restore_tty')

    def initialize(**kwargs):
        history.append('init')
        alive[0] = True

    def shutdown():
        history.append('shutdown')
        alive[0] = False

    now = [0.0]

    def spin(*_, **kwargs):
        now[0] += kwargs['timeout_sec']

    monkeypatch.setattr(runtime, 'RunLock', Lease)
    monkeypatch.setattr(runtime, 'KeyboardNode', lambda: node)
    monkeypatch.setattr(runtime, 'TerminalKeyboard', lambda _: Terminal())
    monkeypatch.setattr(runtime.rclpy, 'init', initialize)
    monkeypatch.setattr(runtime.rclpy, 'ok', lambda: alive[0])
    monkeypatch.setattr(runtime.rclpy, 'shutdown', shutdown)
    monkeypatch.setattr(runtime.rclpy, 'spin_once', spin)
    monkeypatch.setattr(runtime.signal, 'signal', lambda *_: None)
    monkeypatch.setattr(runtime.time, 'monotonic', lambda: now[0])
    return SimpleNamespace(history=history, node=node)


def test_guarded_start_and_quit_order(runtime_hooks):
    """Quit never publishes a stale command; stop precedes TTY/context/lease release."""
    assert runtime.main([]) == 0
    assert runtime_hooks.history == [
        'lock', 'init', 'tty', 'arm', 'zero_burst', 'restore_tty', 'destroy', 'shutdown', 'unlock']


def test_existing_publisher_rejects_without_stop_or_command(runtime_hooks, capsys):
    """A rejected keyboard must not send zero to someone else's control graph."""
    runtime_hooks.node.get_publishers_info_by_topic = lambda _: [object()]
    assert runtime.main([]) == 2
    assert 'publisher' in capsys.readouterr().err
    assert runtime_hooks.history == [
        'lock', 'init', 'tty', 'restore_tty', 'destroy', 'shutdown', 'unlock']


def test_missing_tty_never_arms_output(runtime_hooks, monkeypatch):
    """No controlling terminal is a startup error with context and lock cleanup."""
    def unavailable(_):
        raise runtime.TerminalUnavailable('TTY absent')

    monkeypatch.setattr(runtime, 'TerminalKeyboard', unavailable)
    assert runtime.main([]) == 2
    assert runtime_hooks.history == ['lock', 'init', 'destroy', 'shutdown', 'unlock']


def test_lock_busy_precedes_ros_and_terminal(runtime_hooks, monkeypatch):
    """Contention cannot create a ROS node, open a terminal or touch another run."""
    def busy():
        raise runtime.LockBusy('another navigation run')

    monkeypatch.setattr(runtime, 'RunLock', busy)
    assert runtime.main([]) == 2
    assert runtime_hooks.history == []


def test_terminal_failure_after_arming_stops_before_cleanup(runtime_hooks, monkeypatch):
    """A read-side disconnection retains the stop burst on the already-owned output."""
    def read(_):
        raise runtime.TerminalUnavailable('disconnected')

    monkeypatch.setattr(runtime, 'TerminalKeyboard',
                        lambda _: nullcontext(SimpleNamespace(read_key=read)))
    assert runtime.main([]) == 2
    assert runtime_hooks.history == [
        'lock', 'init', 'arm', 'zero_burst', 'destroy', 'shutdown', 'unlock']


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM])
def test_signal_during_discovery_never_arms(runtime_hooks, monkeypatch, signum):
    """An interrupted discovery window restores resources without a stop publication."""
    handlers = {}

    def register(number, handler):
        previous = handlers.get(number, 'original')
        handlers[number] = handler
        return previous

    def query(_):
        handlers[signum](signum, None)
        return []

    monkeypatch.setattr(runtime.signal, 'signal', register)
    runtime_hooks.node.get_publishers_info_by_topic = query
    assert runtime.main([]) == 0
    assert runtime_hooks.history == [
        'lock', 'init', 'tty', 'restore_tty', 'destroy', 'shutdown', 'unlock']
    assert all(handler == 'original' for handler in handlers.values())


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM])
def test_signal_during_key_read_prevents_stale_command(runtime_hooks, monkeypatch, signum):
    """An interrupt racing with read completion prevents publishing that key's motion."""
    handlers = {}

    def register(number, handler):
        previous = handlers.get(number, 'original')
        handlers[number] = handler
        return previous

    def read(_):
        handlers[signum](signum, None)
        return 'w'

    monkeypatch.setattr(runtime.signal, 'signal', register)
    monkeypatch.setattr(runtime, 'TerminalKeyboard',
                        lambda _: nullcontext(SimpleNamespace(read_key=read)))
    assert runtime.main([]) == 0
    assert runtime_hooks.history == [
        'lock', 'init', 'arm', 'zero_burst', 'destroy', 'shutdown', 'unlock']
    assert all(handler == 'original' for handler in handlers.values())
