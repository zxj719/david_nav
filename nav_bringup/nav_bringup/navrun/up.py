"""统一启动 CLI：一个纯预览路径，一个在锁作用域内带记录的运行路径.

两条路径共用同一份配置与启动图，因此预览看到的就是将要启动的东西；
但预览成功只说明配置与入口点齐备，不构成对硬件或部署的任何验收。
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import sys
from typing import Any, Dict, Tuple

from nav_bringup.launching.executables import resolve_executable
from nav_bringup.navcfg import schema
from nav_bringup.navrun import control_client
from nav_bringup.navrun.preflight import PreflightError, ProcessInfo
from nav_bringup.navrun.locking import DEFAULT_LOCK_PATH
from nav_bringup.navrun.recorded_startup import run_recorded_startup, StartupOptions
from nav_bringup.navrun.runtime_commands import bind_commands
from nav_bringup.navrun.runtime_host import bind_host_steps
from nav_bringup.navrun.teleop_options import TeleopOptions
from nav_bringup.navrun.up_planning import discover_inputs, plan_up, report_plan, UpRequest
from nav_bringup.recorder.run_dir import default_runs_root


def _processes() -> Tuple[ProcessInfo, ...]:
    from nav_bringup.navrun.process_backend import snapshot_processes

    return snapshot_processes()


def preview(args: argparse.Namespace) -> Dict[str, Any]:
    """读取资源文件与进程列表，返回启动预览.

    Args:
        args: 已解析的 up 参数；source_root 可选地覆盖已安装资源。

    Returns:
        JSON 兼容的计划。用于试算的虚拟运行目录从不会被真的创建，
        过程中也不启动 git、ROS CLI、平台命令或任何其它子进程。

    Raises:
        ValueError: 配置或选择子非法。
        LookupError: 缺少必需的已安装资源。
        OSError: 某个必需输入读不到。
        PreflightError: 进程可见性不足，无法完成底盘自动探测。
    """
    root = (args.runs_root or default_runs_root()).expanduser().absolute()
    plan = _plan(args, root / 'dry_run_preview')
    report = report_plan(plan)
    if not plan.errors:
        try:
            report['runtime_commands'] = asdict(bind_commands(
                plan, strict=args.strict, resolve=resolve_executable))
        except (ValueError, LookupError, OSError) as error:
            report['errors'].append(str(error))
    return report


def _plan(args, run_dir):
    """解析资源与 schema，并基于只读进程快照生成启动计划.

    Args:
        args: CLI 的模式、机器人/场地、底盘、记录及可选源码根配置。
        run_dir: 计划所属运行目录路径。

    Returns:
        plan_up 生成的未执行计划；不在本函数创建 ROS 或发送信号。

    Raises:
        ValueError: 没有 schema 或请求/配置验证失败。
        LookupError: 资源包或所需可执行文件未找到。
        OSError: 输入资源或进程快照无法读取。
    """
    request = UpRequest(args.mode, args.robot, args.site, run_dir,
                        args.chassis, args.record, args.no_executor,
                        TeleopOptions(args.layout, args.linear_speed, args.angular_speed),
                        getattr(args, 'mode_file', None), getattr(args, 'composition', False),
                        getattr(args, 'oa', False))

    def share(package: str) -> Path:
        if args.source_root is not None:
            path = args.source_root.expanduser().absolute() / package
            if not (path / 'package.xml').is_file():
                raise LookupError(f'package resources not found: {path}')
            return path
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(package))

    inputs = discover_inputs(request, share)
    paths = (schema.find_source_schemas(args.source_root.expanduser())
             if args.source_root is not None
             else schema.find_installed_schemas(schema.ament_prefixes(os.environ)))
    if not paths:
        raise ValueError('no package schemas found')
    return plan_up(request, inputs, schema.load_catalog(paths), _processes(), os.getpid(),
                   resolve_executable)


def take_over(lock_path: Path, budget_sec: float) -> Dict[str, Any]:
    """启动前请当前持有者协作退出；只等待，不强杀、不删锁、不抢占.

    没有持有者时立即返回。持有者让出之前本函数不会去碰任何资源，因此失败时现场保持原样。

    Args:
        lock_path: 导航租约路径。
        budget_sec: 有限且为正的总预算（秒），含交换请求与等待清理。

    Returns:
        终态响应；``state`` 为 inactive 表示本来就没人持有。

    Raises:
        ValueError: 预算非法。
        RuntimeError: 持有者不肯让出——没有协作端点、响应不合法，或预算内没清理完。
    """
    if not math.isfinite(budget_sec) or budget_sec <= 0:
        raise ValueError('--takeover-timeout must be finite and positive')
    try:
        result = control_client.request_takeover(
            lock_path, min(10.0, budget_sec), budget_sec)
    except (OSError, ValueError, TimeoutError) as error:
        raise RuntimeError(
            'takeover refused: the current owner is not a cooperative navrun '
            '(%s); stop it yourself' % type(error).__name__) from error
    if result['state'] == 'inactive':
        return result
    if not result.get('cleanup_complete'):
        raise RuntimeError(
            'takeover incomplete: the owner did not finish cleanup '
            'within %g s (state=%s)' % (budget_sec, result.get('state')))
    return result


def start(args: argparse.Namespace) -> int:
    """执行已准备好的带记录启动，不修改现网的任何快捷方式.

    给了 ``--takeover`` 时，先请当前持有者协作退出并确认它清理完成，再照常启动；
    持有者不肯让出就直接失败，不会强杀、不会删锁，也不会两套栈同时在跑。

    Args:
        args: 已解析的模式、档案、记录、仓库与严格检查选择子。

    Returns:
        清理与收尾完成之后记录下来的退出状态，语义与 shell 一致；
        它描述这次启动怎么结束，不描述导航任务成没成功。

    Raises:
        ValueError: 从安装副本启动时必须显式给出仓库，否则无法追溯。
        OSError: 缺少必需文件、可执行文件，或记录操作不可用。
        RuntimeError: 无法建立归属、主机准备或记录。
    """
    if getattr(args, 'takeover', False):
        released = take_over(DEFAULT_LOCK_PATH,
                             float(getattr(args, 'takeover_timeout', 60.0)))
        if released['state'] != 'inactive':
            print('navrun up: took over from run %s (pid %s)'
                  % (released.get('run_id'), released.get('pid')), file=sys.stderr)
    repo = args.repo or args.source_root
    if repo is None:
        candidate = Path(__file__).resolve().parents[3]
        if not (candidate / '.git').exists():
            raise ValueError('cannot locate source repository; provide --repo explicitly')
        repo = candidate
    options = StartupOptions(
        args.mode, args.robot, 'none' if args.mode in ('chassis', 'teleop') else args.site,
        (args.runs_root or default_runs_root()).expanduser().absolute(),
        repo.expanduser().absolute(), DEFAULT_LOCK_PATH,
        command=getattr(args, 'invocation', ('navrun', 'up', '--mode', args.mode)))
    result = run_recorded_startup(
        options, lambda path: _plan(args, path),
        lambda plan: bind_commands(plan, strict=args.strict, resolve=resolve_executable),
        bind_host_steps(Path('/dev/shm')))
    print(f'navrun: run record {result.run.path} '
          f'({result.exit_reason}, exit {result.exit_code})', file=sys.stderr)
    return result.exit_code


def run(args: argparse.Namespace) -> int:
    """按参数分发：要么只读预览，要么显式执行带记录的启动.

    Args:
        args: 已解析的启动参数。

    Returns:
        预览有效返回 0，配置无效返回 2，实际启动则返回其运行期退出码。
        预览有效从不等于硬件或部署通过验收。
    """
    try:
        if not args.dry_run:
            return start(args)
        report = preview(args)
    except (ValueError, LookupError, OSError, PreflightError, RuntimeError) as error:
        print(f'navrun up: {error}', file=sys.stderr)
        return 2
    print(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2))
    return 2 if report['errors'] else 0
