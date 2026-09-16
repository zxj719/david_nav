"""Bind prepared startup plans to installed programs without launching anything."""

from __future__ import annotations

from pathlib import Path
import shutil
import sys
from typing import Any, Callable, Dict

import yaml

from nav_bringup.launching.executables import resolve_executable
from nav_bringup.launching.graph import NAVIGATION_MODES
from nav_bringup.navcfg import render
from nav_bringup.navrun import event_tail
from nav_bringup.navrun.startup_lifecycle import StartupCommands, StartupGate
from nav_bringup.navrun.teleop_options import apply_options, TeleopOptions
from nav_bringup.navrun.up_planning import UpPlan


def effective_profile(plan: UpPlan) -> Dict[str, Any]:
    """Read the frozen profile bytes rather than mutable source files.

    Args:
        plan: Prepared and validated startup plan.

    Returns:
        Robot settings with the selected mode overlay applied.

    Raises:
        ValueError: The prepared mode no longer matches its declared identity.
    """
    artifacts = dict(plan.bundle.artifacts)
    robot = yaml.safe_load(artifacts['params/inputs/robot.yaml'])
    mode = render.parse_mode(yaml.safe_load(artifacts['params/inputs/mode.yaml']),
                             Path(plan.bundle.options.run_dir) / 'params/inputs/mode.yaml')
    if mode.name != plan.bundle.options.mode:
        raise ValueError('prepared mode does not match startup identity')
    overrides = yaml.safe_load(artifacts.get('params/inputs/teleop_options.yaml', b'{}'))
    if not isinstance(overrides, dict) or set(overrides) - {
            'layout', 'linear_speed', 'angular_speed'}:
        raise ValueError('invalid frozen teleop overrides')
    return apply_options(render.deep_merge(robot, mode.profile), mode.name,
                         TeleopOptions(**overrides))


def bind_commands(
    plan: UpPlan, strict: bool = False,
    resolve: Callable[[str, str], str] = resolve_executable,
    which: Callable[[str], str | None] = shutil.which, python: str = sys.executable,
) -> StartupCommands:
    """Build exact installed launch/check/record/monitor/zero argument vectors.

    Parameter dumping is a separately owned nonfatal one-shot, never a detached
    child of monitor. No daemon, subprocess, ROS context or configuration write
    is started by this function. The caller validates all vectors before host work.

    Args:
        plan: Immutable configuration and explicit recording selection.
        strict: Request failure when the live configuration checker finds errors.
        resolve: Read-only package executable resolver, injectable for tests.
        which: Read-only PATH resolver for ros2.
        python: Absolute interpreter used by the zero-only module and readiness tail.

    Returns:
        Bound commands using the exact same run directory as the prepared bundle.

    Raises:
        ValueError: Invalid selectors, unresolved paths or missing teleop ownership.
        LookupError: A required installed executable is unavailable.
    """
    options = plan.bundle.options
    if plan.errors:
        raise ValueError('cannot bind a rejected startup plan')
    if type(strict) is not bool or not Path(python).is_absolute():
        raise ValueError('strict must be boolean and python must be an absolute path')
    ros2 = which('ros2')
    navrun = resolve('nav_bringup', 'navrun')
    if not ros2 or not all(Path(value).is_absolute() for value in (ros2, navrun)):
        raise LookupError('ros2 and navrun must resolve to absolute executable paths')
    profile = effective_profile(plan)
    run_dir = options.run_dir
    launch = (ros2, 'launch', 'nav_bringup', 'nav.launch.py', 'run_dir:=' + run_dir,
              'robot:=' + plan.bundle.robot, 'site:=' + plan.bundle.site, 'mode:=' + options.mode)
    executor_launch = launch + ('phase:=executor',) if options.start_executor else ()
    launch += ('phase:=core',)
    gates = []
    prelaunch = []
    navigation = options.mode in NAVIGATION_MODES
    if navigation:
        ready = resolve('nav_bringup', 'nav_ready_exec')
        navcfg = resolve('nav_bringup', 'navcfg')
        if not all(Path(value).is_absolute() for value in (ready, navcfg)):
            raise LookupError('readiness and check executables must resolve to absolute paths')
        gates.append(StartupGate('ready', (
            ready, '--timeout', f'{options.ready_timeout_sec:g}', '--action', options.ready_action,
            '--', python, '-c', 'raise SystemExit(0)'), options.ready_timeout_sec + 5.0))
        check = (navcfg, 'check', '--live', '--run-dir', run_dir,
                 '--events', str(Path(run_dir) / 'events.jsonl'))
        prelaunch.append(StartupGate('localization', check + ('--wait-localization-only',)))
        gates.append(StartupGate('live_check', check + (('--strict',) if strict else ())))
    monitor = (navrun, 'monitor', '--run-dir', run_dir,
               '--base-frame', render.profile_value(profile, 'frames.nav_base'),
               '--map-frame', render.profile_value(profile, 'frames.map'))
    for topic in profile.get('record', {}).get('monitor_topics', []):
        monitor += ('--topic', topic)
    if not navigation:
        monitor += ('--no-bt',)
    record = (ros2, *plan.recording.argv[1:]) if plan.recording.enabled else ()
    zero = (python, '-m', 'nav_bringup.navrun.zero_command',
            '--topic', '/' + options.chassis_command_topic.lstrip('/'))
    keyboard = ()
    terminal = ''
    if options.mode == 'teleop':
        if not options.supervisor_keyboard:
            raise ValueError('teleop requires a supervisor-owned keyboard graph')
        executable = resolve('chassis_teleop', 'chassis_teleop_managed')
        if not Path(executable).is_absolute():
            raise LookupError('managed keyboard must resolve to an absolute path')
        keyboard = (executable, '--ros-args', '--params-file',
                    str(Path(run_dir) / 'params/rendered/teleop.yaml'))
        terminal = render.profile_value(profile, 'teleop.tty_device')
    return StartupCommands(launch, tuple(gates), record, monitor, zero,
                           (navrun, 'dump-params', '--run-dir', run_dir), keyboard, terminal,
                           tuple(prelaunch), executor_launch, event_tail.command(python))
