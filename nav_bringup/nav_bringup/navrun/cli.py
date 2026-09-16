"""Command line entry point ``navrun``.

Example:
    navrun wrap --mode tour --alias 4 -- ~/ros2_ws/src/david_nav/start_navigation_headless.sh
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
from typing import Dict, Optional, Sequence

from nav_bringup.doctor import checks
from nav_bringup.doctor import live
from nav_bringup.navrun import event_tail, wrap
from nav_bringup.recorder import events
from nav_bringup.recorder import linked_process
from nav_bringup.recorder import monitor
from nav_bringup.recorder import param_dump
from nav_bringup.recorder import run_dir
from nav_bringup.recorder import topic_stats

DEFAULT_REPO_DIR = '~/ros2_ws/src'


def build_parser() -> argparse.ArgumentParser:
    """Create the ``navrun`` argument parser.

    Returns:
        The parser with every subcommand registered.
    """
    parser = argparse.ArgumentParser(
        prog='navrun', description='Navigation run supervisor and recorder.')
    subparsers = parser.add_subparsers(dest='subcommand', required=True)

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
        'tour', 'precise', 'bench_slamgt', 'chassis', 'teleop'))
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
    up_parser.add_argument('--composition', action='store_true',
                           help='opt in to the isolated Nav2 component container')
    up_parser.add_argument('--dry-run', action='store_true')
    up_parser.add_argument('--strict', action='store_true', help='fail on live check errors')
    up_parser.add_argument('--repo', type=Path, help='repository whose code state is recorded')
    up_parser.add_argument('--runs-root', type=Path, help='run root; not created by --dry-run')
    up_parser.add_argument('--source-root', type=Path, help='use source-tree resource files')

    wrap_parser = subparsers.add_parser(
        'wrap', help='record a run around an existing launch command')
    wrap_parser.add_argument('--mode', required=True, help='run mode, e.g. tour or precise')
    wrap_parser.add_argument(
        '--alias', default=None, help='shell alias that started the run (default: $NAVRUN_ALIAS)')
    wrap_parser.add_argument('--robot', default=None, help='robot profile name')
    wrap_parser.add_argument('--site', default=None, help='site profile name')
    wrap_parser.add_argument(
        '--runs-root', type=Path, default=None, help='default: $NAV_RUNS_ROOT or ~/nav_runs')
    wrap_parser.add_argument(
        '--repo', type=Path, default=None, help='git repository to record (default: '
        '$NAVRUN_REPO or ~/ros2_ws/src)')
    wrap_parser.add_argument(
        '--file', action='append', default=[], metavar='NAME=PATH',
        help='input file to fingerprint in the manifest; repeatable')
    wrap_parser.add_argument(
        '--sample-interval', type=float, default=1.0, help='seconds between resource samples')
    wrap_parser.add_argument(
        '--no-gc', action='store_true', help='do not prune old runs before starting')
    wrap_parser.add_argument(
        '--monitor-base-frame', default=None, metavar='FRAME',
        help='run navrun monitor next to the command with this robot frame '
        '(robot profile frames.nav_base)')
    wrap_parser.add_argument(
        '--monitor-topic', action='append', default=[], metavar='TOPIC',
        help='topic whose rate the monitor records; repeatable')
    wrap_parser.add_argument(
        '--monitor-period', type=float, default=topic_stats.DEFAULT_PERIOD_SEC,
        help='seconds between topics.csv samples')
    wrap_parser.add_argument(
        '--ready-action', default=wrap.DEFAULT_READY_ACTION, metavar='ACTION',
        help='action whose server marks the stack as ready (default: %(default)s)')
    wrap_parser.add_argument(
        '--no-dump-params', action='store_true',
        help='do not dump effective parameters when the ready action is up')
    wrap_parser.add_argument(
        '--loc-process', default=None, metavar='EXECUTABLE',
        help='executable of the separately started localization to link to the run '
        '(robot profile localization.process_name)')
    wrap_parser.add_argument(
        '--loc-config-flag', action='append', default=None, metavar='FLAG',
        help='name of a localization flag whose value is a config file to hash, e.g. '
        'config for --config; repeatable (default: config)')
    wrap_parser.add_argument(
        '--loc-log', action='append', default=[], metavar='GLOB',
        help='localization log files to excerpt into logs/loc; {pid} is replaced with the '
        'process id; repeatable')
    wrap_parser.add_argument(
        'command', nargs=argparse.REMAINDER, help='command to run, placed after --')

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


def parse_file_arguments(values: Sequence[str]) -> Dict[str, Path]:
    """Parse repeated ``NAME=PATH`` arguments.

    Args:
        values: Raw argument values.

    Returns:
        Mapping of each name to its expanded path.

    Raises:
        ValueError: If a value has no ``=`` or an empty name or path.
    """
    files: Dict[str, Path] = {}
    for value in values:
        name, separator, path = value.partition('=')
        if not separator or not name or not path:
            raise ValueError(f'--file expects NAME=PATH, got {value!r}')
        files[name] = Path(path).expanduser()
    return files


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run the ``navrun`` command line.

    Args:
        argv: Arguments without the program name; defaults to ``sys.argv[1:]``.

    Returns:
        The process exit status.
    """
    parser = build_parser()
    args = parser.parse_args(sys.argv[1:] if argv is None else list(argv))
    args.invocation = ('navrun', *(sys.argv[1:] if argv is None else list(argv)))
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
    return _wrap(parser, args)


def _run_directory(explicit: Optional[Path]) -> Optional[run_dir.RunDirectory]:
    directory = explicit
    if directory is None and os.environ.get('NAVRUN_RUN_DIR'):
        directory = Path(os.environ['NAVRUN_RUN_DIR'])
    if directory is None:
        return None
    directory = directory.expanduser()
    return run_dir.RunDirectory(directory.parent, directory.name)


def _as_flag(name: str) -> str:
    return name if name.startswith('-') else f'--{name}'


def _doctor(args: argparse.Namespace) -> int:
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
            wrap.EVENT_SOURCE, 'params_dumped', 'info' if report.ok else 'warn',
            report.to_dict())
    problems = ', node list failed' if report.list_failed else ''
    print(
        f'navrun: dumped {len(report.dumped)} of {len(report.nodes)} nodes into {output_dir} '
        f'(skipped {len(report.skipped)}, failed {len(report.failed)}{problems})',
        file=sys.stderr,
    )
    return 0 if report.ok else 1


def _wrap(parser: argparse.ArgumentParser, args: argparse.Namespace) -> int:
    command = list(args.command)
    if command and command[0] == '--':
        command = command[1:]
    if not command:
        parser.error('wrap needs a command after --')
    try:
        files = parse_file_arguments(args.file)
    except ValueError as error:
        parser.error(str(error))
    repo_dir = args.repo or Path(os.environ.get('NAVRUN_REPO', DEFAULT_REPO_DIR))
    monitor_command = None
    if args.monitor_base_frame:
        monitor_command = wrap.monitor_command(
            base_frame=args.monitor_base_frame,
            topics=args.monitor_topic,
            period_sec=args.monitor_period,
            ready_action=args.ready_action,
            dump_params=not args.no_dump_params,
        )
    linked = ()
    if args.loc_process:
        flags = tuple(_as_flag(flag) for flag in (args.loc_config_flag or ['config']))
        try:
            linked = (linked_process.LinkSpec(
                name='loc', executable=args.loc_process, config_flags=flags,
                log_globs=tuple(args.loc_log)),)
        except ValueError as error:
            parser.error(str(error))
    options = wrap.WrapOptions(
        command=command,
        mode=args.mode,
        runs_root=(args.runs_root or run_dir.default_runs_root()).expanduser(),
        repo_dir=repo_dir.expanduser(),
        alias=args.alias,
        robot=args.robot,
        site=args.site,
        files=files,
        sample_interval_sec=args.sample_interval,
        gc=not args.no_gc,
        monitor_command=monitor_command,
        event_flush_command=event_tail.command() if monitor_command else None,
        linked=linked,
    )
    try:
        result = wrap.run_wrapped(options)
    except ValueError as error:
        parser.error(str(error))
    print(
        f'navrun: run record {result.run.path} '
        f'({result.exit_reason}, exit {result.exit_code})',
        file=sys.stderr,
    )
    return result.exit_code


if __name__ == '__main__':
    sys.exit(main())
