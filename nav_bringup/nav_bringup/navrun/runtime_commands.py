"""把已准备好的启动计划绑定到已安装的程序上，过程中不启动任何东西.

绑定只解析路径并拼参数向量：拼得出来不代表跑得起来。
"""

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
    """读已冻结的档案字节，而不是可变的源文件.

    这样启动期间有人改源文件也不会影响本次运行的取值。

    Args:
        plan: 已准备并校验过的启动计划。

    Returns:
        叠加选定模式差异之后的机器人设置。

    Raises:
        ValueError: 已准备的模式与其声明身份不再一致。
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
    """拼出已安装的启动/检查/录制/监控/置零命令的精确参数向量.

    参数转储是一次性的、单独持有的非致命操作，绝不是 monitor 的脱管子进程。
    本函数不启动守护进程、不起子进程、不建 ROS 上下文、不写配置。
    调用方在动主机之前会先校验全部向量。

    Args:
        plan: 不可变配置与显式的录制选择。
        strict: 在线配置检查发现错误时是否要求失败。
        resolve: 只读的包可执行文件解析器，测试可注入。
        which: ros2 的只读 PATH 解析器。
        python: 置零模块与就绪等待所用的绝对路径解释器。

    Returns:
        与已准备运行包使用同一个运行目录的绑定命令。

    Raises:
        ValueError: 选择子非法、路径无法解析，或遥控归属缺失。
        LookupError: 缺少某个必需的已安装可执行文件。
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
