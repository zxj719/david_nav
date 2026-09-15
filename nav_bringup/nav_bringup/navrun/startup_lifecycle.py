"""Foreground lifecycle for an already prepared, locked navigation run.

This module executes supplied commands; it does not discover or alter platform,
DDS, localization, aliases, or configuration. The outer entry point must retain
its lease through recording finalization. Tests supply ordinary private Python
children, never ROS velocity publishers or hardware commands.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
import signal
import time
from typing import Callable, Mapping, Optional, Tuple

from nav_bringup.navrun.locking import RunLock
from nav_bringup.navrun.keyboard_session import KeyboardSession
from nav_bringup.navrun.owned_process import OwnedProcess
from nav_bringup.recorder.events import EventLog


@dataclass(frozen=True)
class StartupGate:
    """A command that must finish successfully before recording starts.

    Attributes:
        name: Unique event/log label, not a filesystem path.
        argv: Exact executable argument vector.
        timeout_sec: Optional deadline. Manual localization intentionally has none.
    """

    name: str
    argv: Tuple[str, ...]
    timeout_sec: Optional[float] = None


@dataclass(frozen=True)
class StartupCommands:
    """Resolved commands selected by the outer startup entry point.

    Attributes:
        launch: Foreground launch command, expected to remain alive.
        gates: Ordered readiness and localization/check commands.
        record: Optional persistent rosbag command; empty explicitly disables it.
        monitor: Optional persistent diagnostic command; its failure only warns.
        zero: Required bounded stop command. It must publish only zero velocity.
        parameter_dump: Optional one-shot diagnostic, owned separately from the
            monitor so its children cannot escape the supervisor's cleanup group.
        keyboard: Optional directly supervised keyboard, never a launch child.
        tty_device: Profile-selected terminal, captured by the outer recorded owner.
    """

    launch: Tuple[str, ...]
    gates: Tuple[StartupGate, ...]
    record: Tuple[str, ...]
    monitor: Tuple[str, ...]
    zero: Tuple[str, ...]
    parameter_dump: Tuple[str, ...] = ()
    keyboard: Tuple[str, ...] = ()
    tty_device: str = ''


@dataclass(frozen=True)
class LifecycleResult:
    """Outcome for the outer recorder to persist before releasing ownership.

    Attributes:
        exit_code: Shell-compatible status, never success for an unexpected exit.
        exit_reason: Stage or signal identifying why supervision stopped.
        cleanup_errors: Descriptions of failed stop attempts requiring inspection.
        remaining: Still-owned groups when cleanup could not finish. The caller
            must retain its lease and these handles, and retry close(), rather
            than publishing a completed run or relinquishing navigation ownership.
    """

    exit_code: int
    exit_reason: str
    cleanup_errors: Tuple[str, ...] = ()
    remaining: Tuple[OwnedProcess, ...] = ()


class _Finished(Exception):
    def __init__(self, code: int, reason: str) -> None:
        self.result = LifecycleResult(code, reason)


class _Supervisor:
    def __init__(self, commands, run_dir, events, environ, stopped, stop_timeout, keyboard):
        self.commands = commands
        self.run_dir = run_dir
        self.events = events
        self.environ = dict(environ)
        self.environ.update(ROS_LOG_DIR=str(run_dir / 'logs/ros'),
                            NAVRUN_RUN_DIR=str(run_dir), NAVRUN_RUN_ID=run_dir.name,
                            PYTHONUNBUFFERED='1')
        self.stopped = stopped
        self.stop_timeout = stop_timeout
        self.children = {}
        self.cleanup_errors = []
        self.launched = False
        self.record_broken = False
        self.monitor_failed = False
        self.dump_finished = False
        self.keyboard = keyboard
        self.keyboard_permitted = False

    def emit(self, event, severity='info', **data):
        # A broken event file must not prevent stop attempts on other children.
        try:
            self.events.append('navrun', event, severity, data)
        except OSError as error:
            if not self.record_broken:
                self.cleanup_errors.append(f'event log: {error}')
            self.record_broken = True

    def start(self, name, argv):
        child = OwnedProcess(argv, self.environ, self.run_dir / 'logs' / (name + '.log'))
        self.children[name] = child
        self.emit('process_started', name=name, pid=child.pid)
        return child

    def check(self):
        signum = self.stopped()
        if signum is not None:
            reason = 'ctrl_c' if signum == signal.SIGINT else 'terminated'
            self.emit('signal_received', 'warn', signal=int(signum))
            raise _Finished(128 + int(signum), reason)
        if self.record_broken:
            raise _Finished(2, 'record_failed')
        keyboard = self.children.get('keyboard')
        if keyboard is not None and keyboard.poll() is not None:
            code = keyboard.poll()
            if code == 0 and self.keyboard_permitted:
                raise _Finished(0, 'keyboard_quit')
            raise _Finished(code if code > 0 else 128 - code if code < 0 else 1,
                            'keyboard_exited')
        for name in ('launch', 'record'):
            child = self.children.get(name)
            code = child.poll() if child is not None else None
            if code is not None:
                self.emit('process_exited', 'error', name=name, exit_code=code)
                raise _Finished(code if code > 0 else 128 - code if code < 0 else 1,
                                name + '_exited')
        monitor = self.children.get('monitor')
        if monitor is not None and not self.monitor_failed and monitor.poll() is not None:
            self.monitor_failed = True
            self.emit('monitor_failed', 'warn', exit_code=monitor.poll())
            self.close('monitor')
        dump = self.children.get('parameter_dump')
        if dump is not None and not self.dump_finished and dump.poll() is not None:
            self.dump_finished = True
            code = dump.poll()
            self.emit('parameter_dump_finished', 'info' if code == 0 else 'warn', exit_code=code)
            self.close('parameter_dump')

    def gate(self, gate):
        self.check()
        child = self.start(gate.name, gate.argv)
        deadline = (time.monotonic() + gate.timeout_sec
                    if gate.timeout_sec is not None else None)
        self.emit('startup_gate_started', name=gate.name)
        while True:
            self.check()
            code = child.poll()
            if code is not None:
                self.close(gate.name)
                if gate.name in self.children:
                    raise _Finished(2, gate.name + '_cleanup_failed')
                self.emit('startup_gate_finished', 'info' if code == 0 else 'error',
                          name=gate.name, exit_code=code)
                if code != 0:
                    raise _Finished(code if code > 0 else 128 - code, gate.name + '_failed')
                return
            if deadline is not None and time.monotonic() >= deadline:
                raise _Finished(1, gate.name + '_timeout')
            time.sleep(0.05)

    def execute(self):
        self.check()
        if self.keyboard is not None:
            argv, descriptors = self.keyboard.command(self.commands.keyboard)
            child = OwnedProcess(argv, self.environ, self.run_dir / 'logs/keyboard.log',
                                 pass_fds=descriptors)
            self.children['keyboard'] = child
            self.keyboard.child_started()
            self.emit('process_started', name='keyboard', pid=child.pid)
            deadline = time.monotonic() + 30.0
            while True:
                self.check()
                if self.keyboard.ready():
                    break
                if time.monotonic() >= deadline:
                    raise _Finished(1, 'keyboard_ready_timeout')
                time.sleep(0.05)
            self.emit('keyboard_guard_ready')
        self.check()
        self.start('launch', self.commands.launch)
        self.launched = True
        if self.keyboard is not None:
            self.check()
            self.keyboard.permit()
            self.keyboard_permitted = True
            self.emit('keyboard_permitted')
        for gate in self.commands.gates:
            self.gate(gate)
        self.check()
        if self.commands.parameter_dump:
            try:
                self.start('parameter_dump', self.commands.parameter_dump)
            except OSError as error:
                self.emit('parameter_dump_failed', 'warn', error=str(error))
        self.check()
        if self.commands.monitor:
            try:
                self.start('monitor', self.commands.monitor)
            except OSError as error:
                self.emit('monitor_failed', 'warn', error=str(error))
        self.check()
        if self.commands.record:
            self.start('record', self.commands.record)
        else:
            self.emit('recording_disabled')
        self.emit('startup_ready')
        while True:
            self.check()
            time.sleep(0.05)

    def close(self, name):
        child = self.children.get(name)
        if child is None:
            return
        try:
            code = child.close(self.stop_timeout)
            del self.children[name]
            self.emit('process_stopped', name=name, exit_code=code)
        except Exception as error:
            self.cleanup_errors.append(f'{name}: {error}')
            self.emit('cleanup_failed', 'error', name=name, error=str(error))

    def zero(self, name):
        try:
            child = self.start(name, self.commands.zero)
            deadline = time.monotonic() + 5.0
            while child.poll() is None and time.monotonic() < deadline:
                time.sleep(0.05)
            code = child.poll()
            if code != 0:
                raise RuntimeError(f'zero command failed or timed out (status={code})')
            self.emit('stop_zero_sent', phase=name)
        except Exception as error:
            self.cleanup_errors.append(f'{name}: {error}')
            self.emit('stop_zero_failed', 'error', phase=name, error=str(error))
        finally:
            self.close(name)

    def cleanup(self):
        if self.keyboard is not None:
            try:
                self.keyboard.revoke()
            except Exception as error:
                self.cleanup_errors.append(f'keyboard revoke: {error}')
                self.emit('keyboard_revoke_failed', 'error', error=str(error))
        if self.launched:
            # Attempt stopping before waiting on any diagnostic helper to exit.
            self.zero('zero_before')
        self.close('keyboard')
        for gate in self.commands.gates:
            self.close(gate.name)
        if self.launched:
            self.close('launch')
            # An external driver survives launch. Try again after navigation
            # publishers have stopped, so a final nonzero cannot overtake zero.
            self.zero('zero_after')
        self.close('record')
        self.close('parameter_dump')
        self.close('monitor')


def supervise(
    commands: StartupCommands, run_dir: Path, events: EventLog,
    environ: Mapping[str, str], lease: RunLock,
    stopped: Callable[[], Optional[int]], stop_timeout_sec: float = 10.0,
    keyboard: Optional[KeyboardSession] = None,
) -> LifecycleResult:
    """Run prepared commands and attempt every owned cleanup before returning.

    The caller creates/finalizes the manifest and retains the lease across this
    call and that finalization. All helpers use private groups and run-local
    output/ROS logs. No signal handler or caller environment is modified here.
    A signal request is polled while gates wait, including indefinite manual
    localization. Monitor failures warn; gate, launch and recording failures
    fail the run. Cleanup errors override the exit code and remain explicit.

    Args:
        commands: Already resolved commands, all validated before launching.
        run_dir: Existing caller-owned directory with a logs subdirectory.
        events: Append-only writer for this run.
        environ: Inherited environment, copied rather than mutated.
        lease: Acquired single-instance lock, retained by the caller.
        stopped: Return the first requested signal number, or None to continue.
        stop_timeout_sec: Finite nonnegative per-group flushing grace period.
        keyboard: Optional parent-owned terminal/socket capabilities, retained by
            the outer recorder until every returned child handle has drained.

    Returns:
        Status including cleanup failures; no unexpected launcher exit is success.

    Raises:
        ValueError: Invalid commands, labels, paths or time limits.
        RuntimeError: The caller has not acquired the lease.
        OSError: The run-local ROS log directory cannot be created.
    """
    lease.fileno()
    validate_commands(commands, run_dir, stop_timeout_sec)
    if bool(commands.keyboard) != (keyboard is not None):
        raise ValueError('keyboard command and parent-owned capabilities must agree')
    (run_dir / 'logs/ros').mkdir(exist_ok=True)
    owner = _Supervisor(commands, run_dir, events, environ, stopped, stop_timeout_sec, keyboard)
    result = LifecycleResult(2, 'startup_failed')
    try:
        owner.execute()
    except _Finished as finished:
        result = finished.result
    except KeyboardInterrupt:
        result = LifecycleResult(130, 'ctrl_c')
    except Exception as error:
        owner.emit('startup_failed', 'error', error=str(error))
    finally:
        owner.cleanup()
    if owner.cleanup_errors:
        return LifecycleResult(2, 'cleanup_failed', tuple(owner.cleanup_errors),
                               tuple(owner.children.values()))
    return result


def validate_commands(
    commands: StartupCommands, run_dir: Path, stop_timeout: float = 10.0,
) -> None:
    """Validate a complete command plan before any host or process mutation.

    Args:
        commands: Resolved vectors and startup gates.
        run_dir: Existing absolute run directory with logs.
        stop_timeout: Finite nonnegative cleanup grace period.

    Raises:
        ValueError: A path, label, command or time limit is invalid.
    """
    if not run_dir.is_absolute() or not (run_dir / 'logs').is_dir():
        raise ValueError('expected an absolute existing run directory with logs')
    _timeout(stop_timeout)
    reserved = {'launch', 'record', 'monitor', 'parameter_dump', 'keyboard',
                'zero_before', 'zero_after', 'ros'}
    argv_list = [commands.launch, commands.zero]
    for gate in commands.gates:
        if not re.fullmatch('[a-z][a-z0-9_]*', gate.name) or gate.name in reserved:
            raise ValueError('gate names must be unique safe nonreserved labels')
        reserved.add(gate.name)
        if gate.timeout_sec is not None:
            _timeout(gate.timeout_sec)
        argv_list.append(gate.argv)
    argv_list.extend(argv for argv in (
        commands.record, commands.monitor, commands.parameter_dump, commands.keyboard) if argv)
    if bool(commands.keyboard) != bool(commands.tty_device):
        raise ValueError('keyboard and terminal must be selected together')
    if commands.tty_device and (
            not isinstance(commands.tty_device, str) or '\0' in commands.tty_device
            or not Path(commands.tty_device).is_absolute()):
        raise ValueError('keyboard terminal must be an absolute path without NUL')
    if any(not argv or any(not isinstance(arg, str) or '\0' in arg for arg in argv)
           for argv in argv_list):
        raise ValueError('commands must be nonempty argument vectors without NUL')


def _timeout(value):
    if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
        raise ValueError('time limits must be nonnegative and finite')
