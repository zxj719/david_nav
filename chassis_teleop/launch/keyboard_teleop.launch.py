"""Delegate keyboard bringup to recorded navrun ownership, never parallel nodes."""

import signal
from typing import List

from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.events import Shutdown
from launch.events import matches_action
from launch.events.process import ProcessExited, SignalProcess
from launch.substitutions import LaunchConfiguration

from nav_bringup.launching.executables import resolve_executable
from nav_bringup.navrun.teleop_options import TeleopOptions


def _exited(event: ProcessExited, context: LaunchContext) -> List[Action]:
    if event.returncode != 0:
        raise RuntimeError(f'keyboard navrun failed with exit {event.returncode}; '
                           'inspect its run record')
    return [EmitEvent(event=Shutdown(reason='keyboard navrun finished'))]


def _setup(context: LaunchContext) -> List[Action]:
    def value(name):
        return LaunchConfiguration(name).perform(context)

    options = TeleopOptions(
        layout=value('layout'),
        linear_speed=float(value('linear_speed')) if value('linear_speed') else None,
        angular_speed=float(value('angular_speed')) if value('angular_speed') else None)
    options.values('teleop')
    command = [resolve_executable('nav_bringup', 'navrun'), 'up', '--mode', 'teleop',
               '--robot', value('robot'), '--chassis', value('start_driver'),
               '--record', value('record'), '--layout', value('layout')]
    for name in ('linear_speed', 'angular_speed', 'repo', 'runs_root', 'source_root'):
        if value(name):
            command.extend(['--' + name.replace('_', '-'), value(name)])
    if value('dry_run') == 'true':
        command.append('--dry-run')
    # Navrun owns independent child groups and needs time for two zero bursts,
    # launch/keyboard teardown and recording finalization before escalation.
    owner = ExecuteProcess(cmd=command, name='keyboard_navrun', output='screen',
                           shell=False, emulate_tty=False, on_exit=_exited,
                           sigterm_timeout='60', sigkill_timeout='60')
    # Interactive Humble otherwise assumes SIGINT already reached all children.
    # Explicitly notify only our action, including process-targeted interruption.
    stop = RegisterEventHandler(OnShutdown(on_shutdown=[EmitEvent(event=SignalProcess(
        signal_number=signal.SIGINT, process_matcher=matches_action(owner)))]))
    return [stop, owner]


def generate_launch_description() -> LaunchDescription:
    """Declare the convenience entry without opening terminals or starting ROS.

    Returns:
        A single recorded navrun process, with failures propagated to launch.
        The inherited controlling terminal is opened by navrun, not launch stdin.
    """
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='david'),
        DeclareLaunchArgument('start_driver', default_value='auto', choices=['auto', 'on', 'off']),
        DeclareLaunchArgument('layout', default_value='wasd', choices=['wasd', 'uiojkl']),
        DeclareLaunchArgument('linear_speed', default_value='',
                              description='m/s; empty uses profile, cannot exceed profile limits'),
        DeclareLaunchArgument('angular_speed', default_value='',
                              description='rad/s; empty uses profile; deadzones still apply'),
        DeclareLaunchArgument('record', default_value='light', choices=['light', 'full', 'off']),
        DeclareLaunchArgument('repo', default_value='', description='Recorded code repository'),
        DeclareLaunchArgument('runs_root', default_value='', description='Run record root'),
        DeclareLaunchArgument('source_root', default_value='', description='Source assets'),
        DeclareLaunchArgument('dry_run', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=_setup),
    ])
