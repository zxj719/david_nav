"""Guard and shutdown sequencing with fake ROS, locks, terminals and publishers."""

from contextlib import nullcontext
import os
import pty
import signal
import socket
from types import SimpleNamespace

import pytest

from chassis_teleop import node as runtime
from nav_bringup.navrun.locking import LockBusy, RunLock


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


def test_delegated_keyboard_waits_for_owner_before_arming(runtime_hooks):
    """Publisher exclusion and ready/permit precede even the first zero command."""
    import threading

    history = runtime_hooks.history
    channel = SimpleNamespace(ready=lambda: history.append('ready'),
                              permitted=lambda: history.append('permit') or True,
                              check_owner=lambda: history.append('owner'))
    keyboard = SimpleNamespace(read_key=lambda _: 'q')
    runtime.rclpy.init()
    runtime.run_keyboard(runtime_hooks.node, keyboard, threading.Event(), channel)
    assert history == ['init', 'ready', 'permit', 'arm', 'owner', 'owner']


def test_delegated_owner_loss_prevents_processing_received_motion(runtime_hooks):
    """A disconnect racing with a motion key cannot publish that motion."""
    import threading

    checks = []

    def owner():
        checks.append(True)
        if len(checks) > 1:
            raise runtime.KeyboardOwnerLost('owner gone')

    channel = SimpleNamespace(ready=lambda: None, permitted=lambda: True, check_owner=owner)
    runtime.rclpy.init()
    with pytest.raises(runtime.KeyboardOwnerLost):
        runtime.run_keyboard(runtime_hooks.node, SimpleNamespace(read_key=lambda _: 'w'),
                             threading.Event(), channel)
    assert runtime_hooks.history == ['init', 'arm']


def test_delegated_missing_permit_has_bounded_unarmed_wait(runtime_hooks):
    """A stalled owner never silently activates keyboard output."""
    import threading

    channel = SimpleNamespace(ready=lambda: None, permitted=lambda: False)
    runtime.rclpy.init()
    with pytest.raises(runtime.KeyboardOwnerLost, match='activation'):
        runtime.run_keyboard(runtime_hooks.node, SimpleNamespace(read_key=lambda _: 'w'),
                             threading.Event(), channel)
    assert runtime_hooks.history == ['init']


@pytest.mark.parametrize('valid', [False, True])
def test_managed_entry_requires_real_flock_before_ros(tmp_path, monkeypatch, valid):
    """A complete capability tuple cannot replace the kernel ownership check."""
    path = tmp_path / 'lease'
    parent, child = socket.socketpair()
    master, slave = pty.openpty()
    entered = []
    descriptors = []
    monkeypatch.setattr(runtime, 'DEFAULT_LOCK_PATH', path)
    monkeypatch.setattr(runtime, '_run', lambda *args: entered.append(args) or 0)
    try:
        with RunLock(path) as lease:
            with path.open('rb') as wrong:
                source = lease.fileno() if valid else wrong.fileno()
                descriptors = [os.dup(fd) for fd in (source, child.fileno(), slave)]
                args = [item for name, fd in zip(('lock', 'control', 'tty'), descriptors)
                        for item in (f'--{name}-fd', str(fd))]
                assert runtime.managed_main(args) == (0 if valid else 2)
                assert bool(entered) == valid
                for fd in descriptors:
                    with pytest.raises(OSError):
                        os.fstat(fd)
                with pytest.raises(LockBusy):
                    RunLock(path).acquire()
    finally:
        parent.close()
        child.close()
        os.close(master)
        os.close(slave)


def test_owner_loss_after_activation_stops_and_restores(runtime_hooks, monkeypatch):
    """A disconnected supervisor cannot skip the armed keyboard's stop burst."""
    def lost():
        raise runtime.KeyboardOwnerLost('disconnected')

    channel = SimpleNamespace(ready=lambda: None, permitted=lambda: True, check_owner=lost)
    with pytest.raises(runtime.KeyboardOwnerLost):
        runtime._run([], channel)
    assert runtime_hooks.history == [
        'init', 'tty', 'arm', 'zero_burst', 'restore_tty', 'destroy', 'shutdown']
