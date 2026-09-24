"""``navrun`` 命令行入口.

各子命令的职责分工写在各自模块里；本模块只负责参数解析与分发，
不直接操作机器人，也不代替子命令做任何校验。

Example:
    navrun up --mode tour
    navrun doctor
    navrun monitor --run-dir ~/nav_runs/latest --base-frame <frames.nav_base> --topic /cmd_vel
    navrun dump-params --run-dir ~/nav_runs/latest
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
from typing import Optional, Sequence

from nav_bringup.doctor import checks
from nav_bringup.doctor import live
from nav_bringup.recorder import events
from nav_bringup.recorder import monitor
from nav_bringup.recorder import param_dump
from nav_bringup.recorder import run_dir
from nav_bringup.recorder import topic_stats


def build_parser() -> argparse.ArgumentParser:
    """创建 ``navrun`` 的参数解析器.

    只注册参数，不执行任何子命令，也不读取配置或触碰文件系统。

    Returns:
        已注册全部子命令的解析器。
    """
    parser = argparse.ArgumentParser(
        prog='navrun', description='Navigation run supervisor and recorder.')
    subparsers = parser.add_subparsers(dest='subcommand', required=True)

    selftest_parser = subparsers.add_parser(
        'selftest', help='private Nav2 experiment; requires optional nav_sim package')
    selftest_parser.add_argument('--mode', required=True, choices=('tour', 'precise'))
    selftest_parser.add_argument('--robot', default='david')
    selftest_parser.add_argument('--source-root', type=Path)
    selftest_parser.add_argument('--output-dir', type=Path,
                                 help='new private directory; default: a temporary run')
    selftest_parser.add_argument('--timeout', type=float, default=180.0)
    authority = selftest_parser.add_mutually_exclusive_group()
    authority.add_argument('--prepare-only', action='store_true',
                           help='write private inputs without ROS; not an executed selftest')
    authority.add_argument('--startup-only', action='store_true',
                           help='start/observe/stop private Nav2 without goals; not closed-loop')
    authority.add_argument('--allow-goals', action='store_true',
                           help='explicitly permit two simulated navigation tasks in domain 99')

    from nav_bringup.navrun.locking import DEFAULT_LOCK_PATH

    gc_parser = subparsers.add_parser('gc', help='prune completed run artifacts; keep summaries')
    gc_parser.add_argument('--runs-root', type=Path)
    gc_parser.add_argument('--lock-path', type=Path, default=DEFAULT_LOCK_PATH)
    gc_parser.add_argument('--dry-run', action='store_true', help='read-only preview; no locks')
    gc_parser.add_argument('--max-runs', type=int, default=30)
    gc_parser.add_argument('--max-total-bytes', type=int, default=20 * 1024 ** 3)
    gc_parser.add_argument('--bag-max-age-days', type=float, default=7.0)

    for command in ('status', 'down'):
        control_parser = subparsers.add_parser(command, help='contact the current managed owner')
        control_parser.add_argument('--lock-path', type=Path, default=DEFAULT_LOCK_PATH)
        control_parser.add_argument('--timeout', type=float, default=2.0,
                                    help='positive cooperative response deadline in seconds')
        control_parser.add_argument('--wait-timeout', type=float, default=30.0,
                                    help='down cleanup wait bound; expiry never forces a kill')

    check_parser = subparsers.add_parser('check', help='validate a prepared run; ROS is opt-in')
    selection = check_parser.add_mutually_exclusive_group()
    selection.add_argument('--run-dir', type=Path, help='explicit prepared run directory')
    selection.add_argument('--runs-root', type=Path, help='select latest under this runs root')
    check_parser.add_argument('--source-root', type=Path, help='source schema discovery root')
    check_parser.add_argument('--schema', action='append', type=Path,
                              help='explicit schema file; repeat to override discovery')
    check_parser.add_argument('--live', action='store_true', help='explicitly observe ROS')
    check_parser.add_argument('--strict', action='store_true', help='fail on live check errors')
    check_parser.add_argument('--wait-timeout', type=float, default=0.0,
                              help='localization wait bound; zero waits indefinitely')
    check_parser.add_argument('--sample-sec', type=float, default=1.0)
    check_parser.add_argument('--parameter-timeout', type=float, default=5.0)

    for command in ('ls', 'show'):
        record_parser = subparsers.add_parser(command, help='inspect saved runs as JSON')
        record_parser.add_argument('--runs-root', type=Path,
                                   help='default: $NAV_RUNS_ROOT or ~/nav_runs; read-only')
        if command == 'show':
            record_parser.add_argument('selector', help='exact run ID or latest')
            for option in ('params', 'diff', 'events'):
                record_parser.add_argument('--' + option, action='store_true',
                                           help='include saved ' + option + ' evidence')

    up_parser = subparsers.add_parser('up', help='start and record a navigation mode')
    up_parser.add_argument('--mode', required=True, choices=(
        'tour', 'precise', 'chassis', 'teleop'))
    up_parser.add_argument('--robot', default='david')
    up_parser.add_argument('--mode-file', type=Path,
                           help='explicit complete mode overlay; identity must match --mode')
    up_parser.add_argument('--site', default='daolan')
    up_parser.add_argument('--record', choices=('light', 'full', 'off'), default='light')
    up_parser.add_argument('--chassis', choices=('auto', 'on', 'off'), default='auto')
    up_parser.add_argument('--layout', choices=('wasd', 'uiojkl'), help='teleop-only layout')
    up_parser.add_argument('--linear-speed', type=float, help='teleop-only gear in m/s')
    up_parser.add_argument('--angular-speed', type=float, help='teleop-only gear in rad/s')
    up_parser.add_argument('--no-executor', action='store_true')
    up_parser.add_argument(
        '--OA', '--oa', dest='oa', action='store_true',
        help='add this robot direct PointCloud2 source to the local costmap; '
             'needs the robot overlay modes/overlays/oa.yaml')
    up_parser.add_argument('--composition', action='store_true',
                           help='opt in to the isolated Nav2 component container')
    up_parser.add_argument(
        '--takeover', action='store_true',
        help='ask the current owner to stop cooperatively, then start; never forces')
    up_parser.add_argument(
        '--takeover-timeout', type=float, default=60.0,
        help='seconds allowed for the current owner to finish its cleanup')
    up_parser.add_argument('--dry-run', action='store_true')
    up_parser.add_argument('--strict', action='store_true', help='fail on live check errors')
    up_parser.add_argument('--repo', type=Path, help='repository whose code state is recorded')
    up_parser.add_argument('--runs-root', type=Path, help='run root; not created by --dry-run')
    up_parser.add_argument('--source-root', type=Path, help='use source-tree resource files')

    doctor_parser = subparsers.add_parser(
        'doctor', help='read-only checks for known navigation failure modes')
    doctor_parser.add_argument(
        '--json', action='store_true', help='print results as JSON instead of text')
    doctor_parser.add_argument(
        '--live', action='store_true',
        help='also check the running ROS graph (short ros2 queries and a TF probe node)')
    doctor_parser.add_argument(
        '--cmd-topic', action='append', default=[], metavar='TOPIC',
        help='velocity command topic that must have one publisher at most; repeatable '
        '(robot profile chassis.cmd_topic)')
    doctor_parser.add_argument(
        '--watch-topic', action='append', default=[], metavar='TOPIC',
        help='topic whose publishers must all run on one host; repeatable')
    doctor_parser.add_argument(
        '--map-frame', default='map', help='global frame of the localization transform')
    doctor_parser.add_argument(
        '--odom-frame', default='odom', help='odometry frame of the localization transform')
    doctor_parser.add_argument(
        '--tf-wait', type=float, default=3.0,
        help='seconds to wait for the map to odom transform')

    monitor_parser = subparsers.add_parser(
        'monitor', help='record behavior tree triggers, topic rates and TF lag of a run')
    monitor_parser.add_argument(
        '--run-dir', type=Path, default=None,
        help='run directory to record into (default: $NAVRUN_RUN_DIR; unset prints only)')
    monitor_parser.add_argument(
        '--base-frame', required=True,
        help='robot frame for pose and TF lag (robot profile frames.nav_base)')
    monitor_parser.add_argument(
        '--map-frame', default='map', help='global frame for pose and TF lag')
    monitor_parser.add_argument(
        '--bt-topic', default=monitor.DEFAULT_BT_TOPIC, help='behavior tree log topic')
    monitor_parser.add_argument(
        '--no-bt', action='store_true', help='do not report behavior tree triggers')
    monitor_parser.add_argument(
        '--topic', action='append', default=[], metavar='TOPIC',
        help='topic whose rate is recorded; repeatable (robot profile record.monitor_topics)')
    monitor_parser.add_argument(
        '--period', type=float, default=topic_stats.DEFAULT_PERIOD_SEC,
        help='seconds between topics.csv samples')
    monitor_parser.add_argument(
        '--min-interval', type=float, default=0.5,
        help='minimum seconds between reports of the same trigger')
    monitor_parser.add_argument(
        '--ready-action', default=None, metavar='ACTION',
        help='action whose server marks the stack as ready, e.g. /navigate_to_pose')
    monitor_parser.add_argument(
        '--dump-params-on-ready', action='store_true',
        help='dump effective parameters into the run directory once the ready action is up')

    dump_defaults = param_dump.DumpSettings()
    dump_parser = subparsers.add_parser(
        'dump-params', help='dump the effective parameters of every running node')
    dump_parser.add_argument(
        '--run-dir', type=Path, default=None,
        help='run directory; dumps go to its params/effective (default: $NAVRUN_RUN_DIR)')
    dump_parser.add_argument(
        '--output-dir', type=Path, default=None,
        help='write the dumps here instead of into a run directory')
    dump_parser.add_argument(
        '--spin-time', type=float, default=dump_defaults.spin_time_sec,
        help='discovery seconds for each ros2 call')
    dump_parser.add_argument(
        '--timeout', type=float, default=dump_defaults.timeout_sec,
        help='seconds before one ros2 call is abandoned')
    dump_parser.add_argument(
        '--attempts', type=int, default=dump_defaults.attempts, help='dump attempts per node')
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """运行 ``navrun`` 命令行.

    Args:
        argv: 不含程序名的参数；默认取 ``sys.argv[1:]``。

    Returns:
        进程退出状态，由被分发到的子命令决定；本函数自身不改写它。
    """
    parser = build_parser()
    args = parser.parse_args(sys.argv[1:] if argv is None else list(argv))
    args.invocation = ('navrun', *(sys.argv[1:] if argv is None else list(argv)))
    if args.subcommand == 'selftest':
        try:
            from nav_sim.selftest import run
        except ImportError as error:
            print(f'selftest requires the optional nav_sim package and dependencies: {error}',
                  file=sys.stderr)
            return 2
        return run(args)
    if args.subcommand == 'gc':
        from nav_bringup.navrun import gc

        return gc.run(args)
    if args.subcommand in ('status', 'down'):
        from nav_bringup.navrun import control_client

        return control_client.run(args)
    if args.subcommand == 'check':
        from nav_bringup.navrun import check

        return check.run(args)
    if args.subcommand in ('ls', 'show'):
        from nav_bringup.navrun import records

        return records.run(args)
    if args.subcommand == 'up':
        from nav_bringup.navrun import up

        return up.run(args)
    if args.subcommand == 'doctor':
        return _doctor(args)
    if args.subcommand == 'monitor':
        return _monitor(parser, args)
    if args.subcommand == 'dump-params':
        return _dump_params(parser, args)
    parser.error(f'unsupported command: {args.subcommand}')


def _run_directory(explicit: Optional[Path]) -> Optional[run_dir.RunDirectory]:
    """从显式路径或环境变量选择已有运行目录描述，不创建目录.

    显式路径优先；仅展开用户主目录，不解析绝对路径、符号链接或读取清单。

    Args:
        explicit: 可选显式运行目录；None 时读取 NAVRUN_RUN_DIR。

    Returns:
        根与运行 ID 组成的 RunDirectory；两种来源均无值时返回 None。
    """
    directory = explicit
    if directory is None and os.environ.get('NAVRUN_RUN_DIR'):
        directory = Path(os.environ['NAVRUN_RUN_DIR'])
    if directory is None:
        return None
    directory = directory.expanduser()
    return run_dir.RunDirectory(directory.parent, directory.name)


def _doctor(args: argparse.Namespace) -> int:
    """执行只读 doctor 检查，按显式 live 选项追加在线探针并打印报告.

    Args:
        args: 包含输出格式、话题/帧选择和秒制等待预算的 CLI 选项。

    Returns:
        最坏状态为 FAIL 时返回一，否则返回零；WARN 不当作命令失败。
        本函数不修复检查项，也不重启 ROS daemon 或定位栈。
    """
    results = checks.run_checks(checks.default_context(os.environ))
    if args.live:
        results.extend(live.run_live_checks(live.LiveContext(
            runner=live.default_runner(),
            command_topics=tuple(args.cmd_topic),
            watched_topics=tuple(args.watch_topic),
            map_frame=args.map_frame,
            odom_frame=args.odom_frame,
            transform_probe=live.ros_transform_probe(args.tf_wait),
        )))
    if args.json:
        print(json.dumps([result.to_dict() for result in results], ensure_ascii=False, indent=2))
    else:
        print(checks.format_report(results), end='')
    return 1 if checks.worst_status(results) is checks.Status.FAIL else 0


def _monitor(parser: argparse.ArgumentParser, args: argparse.Namespace) -> int:
    """把 CLI 选项转换为观察器配置并进入该运行记录的监测循环.

    Args:
        parser: 用于报告参数错误的解析器。
        args: 包含运行目录、帧、话题及秒制周期的选项。

    Returns:
        monitor.spin 的退出码。

    Raises:
        SystemExit: 监测配置无效，由 parser.error 报告。
        Exception: 运行目录解析或观察循环失败。
    """
    try:
        options = monitor.MonitorOptions(
            base_frame=args.base_frame,
            map_frame=args.map_frame,
            bt_topic=None if args.no_bt else args.bt_topic,
            topics=tuple(args.topic),
            period_sec=args.period,
            min_trigger_interval_sec=args.min_interval,
            ready_action=args.ready_action,
            dump_params_on_ready=args.dump_params_on_ready,
        )
    except ValueError as error:
        parser.error(str(error))
    return monitor.spin(_run_directory(args.run_dir), options)


def _dump_params(parser: argparse.ArgumentParser, args: argparse.Namespace) -> int:
    """采集真实 ROS 参数并输出完整性结果，不以部分 dump 宣称全部成功.

    Args:
        parser: CLI 错误报告器。
        args: 输出/运行目录及秒制发现、超时、重试次数选项。

    Returns:
        参数报告完整时为零，否则为一；属于运行记录时另追加采集事件。

    Raises:
        SystemExit: 重试次数或输出目录选择无效。
        OSError: 参数输出或事件记录写入失败。
    """
    if args.attempts < 1:
        parser.error('--attempts must be at least 1')
    run = None if args.output_dir is not None else _run_directory(args.run_dir)
    if args.output_dir is not None:
        output_dir = args.output_dir.expanduser()
    elif run is not None:
        output_dir = run.params_effective
    else:
        parser.error('dump-params needs --output-dir, --run-dir or $NAVRUN_RUN_DIR')
    settings = param_dump.DumpSettings(
        spin_time_sec=args.spin_time, timeout_sec=args.timeout, attempts=args.attempts)
    report = param_dump.dump_graph(output_dir, settings=settings)
    if run is not None:
        events.EventLog(run.events).append(
            'navrun', 'params_dumped', 'info' if report.ok else 'warn',
            report.to_dict())
    problems = ', node list failed' if report.list_failed else ''
    print(
        f'navrun: dumped {len(report.dumped)} of {len(report.nodes)} nodes into {output_dir} '
        f'(skipped {len(report.skipped)}, failed {len(report.failed)}{problems})',
        file=sys.stderr,
    )
    return 0 if report.ok else 1


if __name__ == '__main__':
    sys.exit(main())
