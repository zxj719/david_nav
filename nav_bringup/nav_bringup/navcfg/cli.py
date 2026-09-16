"""Command line of navcfg, the robot configuration tool (refactor plan §12).

``navcfg render`` writes the Nav2 parameter file of a robot profile and mode.
``navcfg validate`` applies schema and cross-file safety rules before launch.
``navcfg doc`` generates the checked-in parameter reference from package schemas.
"""

import argparse
import os
from pathlib import Path
import sys
from typing import Dict, Optional, Sequence

from nav_bringup.navcfg import behavior_trees
from nav_bringup.navcfg import documentation
from nav_bringup.navcfg import render
from nav_bringup.navcfg import schema
from nav_bringup.navcfg import validation

PACKAGE_NAME = 'nav_bringup'
BASE_FILE = 'nav2_base.yaml'
BINDINGS_FILE = 'nav2_bindings.yaml'


def _installed_config(file_name: str) -> Path:
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory(PACKAGE_NAME)) / 'config' / file_name


def build_parser() -> argparse.ArgumentParser:
    """Build the parser of the navcfg command line.

    Returns:
        The argument parser.
    """
    parser = argparse.ArgumentParser(prog='navcfg', description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest='command', required=True)
    render_parser = commands.add_parser(
        'render', help='write the Nav2 parameter file of a robot profile and mode')
    render_parser.add_argument(
        '--profile', required=True, type=Path, help='robot profile, e.g. robot/david.yaml')
    render_parser.add_argument(
        '--mode', type=Path, help='mode overlay, e.g. modes/precise.yaml (default: none)')
    render_parser.add_argument(
        '--base', type=Path,
        help=f'base parameter tree (default: share/{PACKAGE_NAME}/config/{BASE_FILE})')
    render_parser.add_argument(
        '--bindings', type=Path,
        help=f'binding table (default: share/{PACKAGE_NAME}/config/{BINDINGS_FILE})')
    render_parser.add_argument(
        '--output', required=True, type=Path, help='parameter file to write')
    render_parser.add_argument(
        '--bt-output-dir', type=Path,
        help='also render BT XML and bind Nav2 paths to this directory')
    render_parser.add_argument(
        '--bt-template-dir', type=Path, help='override BT template directory')
    render_parser.add_argument('--bt-bindings', type=Path, help='override BT binding table')
    validate_parser = commands.add_parser(
        'validate', help='check a robot profile, mode, site, Nav2 tree, BTs and URDF')
    identity = validate_parser.add_mutually_exclusive_group(required=True)
    identity.add_argument('--profile', type=Path, help='explicit robot profile file')
    identity.add_argument('--robot', help='robot name selecting its name_config package')
    validate_parser.add_argument('--mode', required=True, type=Path,
                                 help='overlay file with --profile; mode name with --robot')
    validate_parser.add_argument('--site', type=Path,
                                 help='site file with --profile; optional site name with --robot')
    validate_parser.add_argument('--urdf', type=Path, help='robot URDF with --profile')
    validate_parser.add_argument(
        '--bt-dir', action='append', type=Path,
        help='directory containing rendered BT file names; repeat to add search directories')
    validate_parser.add_argument('--base', type=Path, help='base parameter tree')
    validate_parser.add_argument('--bindings', type=Path, help='binding table')
    validate_parser.add_argument(
        '--source-root', type=Path,
        help='workspace src directory used to discover package schemas')
    validate_parser.add_argument(
        '--schema', action='append', type=Path,
        help='explicit package schema; repeat as needed (overrides schema discovery)')
    doc_parser = commands.add_parser(
        'doc', help='generate Markdown documentation from package schemas')
    doc_parser.add_argument('--output', required=True, type=Path, help='Markdown output file')
    doc_parser.add_argument(
        '--source-root', type=Path,
        help='workspace src directory used to discover package schemas')
    doc_parser.add_argument(
        '--schema', action='append', type=Path,
        help='explicit package schema; repeat as needed (overrides schema discovery)')
    check_parser = commands.add_parser('check', help='verify a prepared run against live ROS')
    check_parser.add_argument('--live', action='store_true',
                              help='explicitly enable ROS observation')
    check_parser.add_argument('--run-dir', required=True, type=Path, help='prepared run directory')
    check_parser.add_argument('--strict', action='store_true', help='return 1 on check errors')
    check_parser.add_argument('--wait-localization-only', action='store_true',
                              help='pre-launch TF wait only; not a full live-check success')
    check_parser.add_argument('--wait-timeout', type=float, default=0.0,
                              help='manual-localization wait bound; zero waits indefinitely')
    check_parser.add_argument('--sample-sec', type=float, default=1.0,
                              help='graph discovery window')
    check_parser.add_argument('--parameter-timeout', type=float, default=5.0,
                              help='global read-only parameter query deadline')
    check_parser.add_argument('--events', type=Path,
                              help='optional event log owned by the calling supervisor')
    return parser


def _render_command(args: argparse.Namespace) -> int:
    """Execute ``navcfg render``.

    Args:
        args: Parsed render arguments.

    Returns:
        Process exit status.
    """
    try:
        base = args.base or _installed_config(BASE_FILE)
        bindings = args.bindings or _installed_config(BINDINGS_FILE)
        tree = render.render_files(base, bindings, args.profile, args.mode)
        bundle = _render_behavior_trees(args, tree)
        # All input validation finishes before any output is written.
        if bundle:
            args.bt_output_dir.mkdir(parents=True, exist_ok=True)
            for name, xml in bundle.items():
                (args.bt_output_dir / name).write_text(xml, encoding='utf-8')
        render.write_parameters(args.output, tree)
    except (render.RenderError, LookupError, OSError) as error:
        print(f'navcfg render: {error}', file=sys.stderr)
        return 2
    print(f'navcfg render: wrote {args.output}')
    return 0


def _render_behavior_trees(args: argparse.Namespace, tree: Dict[str, object]) -> Dict[str, str]:
    """按需渲染 BT 数据资源；只有资产发现依赖 ament，不加载插件实现."""
    if args.bt_output_dir is None:
        if args.bt_template_dir or args.bt_bindings:
            raise render.RenderError('BT template options require --bt-output-dir')
        return {}
    template_dir, binding_file = args.bt_template_dir, args.bt_bindings
    if template_dir is None or binding_file is None:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory('nav_plugins'))
        template_dir = template_dir or share / 'behavior_trees'
        binding_file = binding_file or share / 'config' / 'bt_bindings.yaml'
    profile = render.load_yaml(args.profile)
    if not isinstance(profile, dict):
        raise render.RenderError('robot profile must be a mapping')
    if args.mode:
        mode = render.parse_mode(render.load_yaml(args.mode), args.mode)
        profile = render.deep_merge(profile, mode.profile)
    bundle = behavior_trees.render_bundle(template_dir, binding_file, profile)
    behavior_trees.bind_output_paths(tree, bundle, args.bt_output_dir)
    return bundle


def _catalog(args: argparse.Namespace) -> schema.ParameterCatalog:
    """Load explicitly selected, source-tree, or installed package schemas."""
    if args.schema:
        paths = args.schema
    elif args.source_root:
        paths = schema.find_source_schemas(args.source_root)
    else:
        paths = schema.find_installed_schemas(schema.ament_prefixes(os.environ))
    if not paths:
        raise schema.SchemaError('no package config/schema.yaml files found')
    return schema.load_catalog(paths)


def _site_map(site_path: Path) -> Dict[str, object]:
    """Load the map YAML selected by a site profile."""
    site = render.load_yaml(site_path)
    if not isinstance(site, dict):
        raise render.RenderError(f'{site_path}: the site profile must be a mapping')
    raw_path = render.profile_value(site, 'map.nav_yaml')
    if not isinstance(raw_path, str) or not raw_path:
        raise render.RenderError(f'{site_path}: map.nav_yaml must be a non-empty path')
    map_path = Path(raw_path).expanduser()
    if not map_path.is_absolute():
        map_path = site_path.parent / map_path
    map_data = render.load_yaml(map_path)
    if not isinstance(map_data, dict):
        raise render.RenderError(f'{map_path}: map YAML must be a mapping')
    return map_data


def _behavior_trees(tree: Dict[str, object], directories: Sequence[Path]) -> Dict[str, str]:
    """Resolve every BT file named by the rendered bt_navigator parameters."""
    prefix = 'bt_navigator/ros__parameters/'
    parameter_names = (
        'default_bt_xml_filename',
        'default_nav_to_pose_bt_xml',
        'default_nav_through_poses_bt_xml',
    )
    names = []
    for parameter in parameter_names:
        value = render.get_path(tree, prefix + parameter)
        if not isinstance(value, str) or not value:
            raise render.RenderError(f'{prefix}{parameter}: expected a non-empty file name')
        if value not in names:
            names.append(value)
    resolved = {}
    for name in names:
        path = Path(name).expanduser()
        candidates = (
            [path] if path.is_absolute()
            else [directory / path for directory in directories])
        existing = next((candidate for candidate in candidates if candidate.is_file()), None)
        if existing is None:
            locations = ', '.join(str(candidate) for candidate in candidates)
            raise render.RenderError(f'behavior tree {name!r} not found; checked {locations}')
        resolved[str(existing)] = existing.read_text(encoding='utf-8')
    return resolved


def _validate_command(args: argparse.Namespace) -> int:
    """Execute ``navcfg validate`` and report every issue."""
    try:
        if args.robot is not None:
            from nav_bringup.navcfg.robot_validation import run

            return run(args, _catalog(args))
        base = args.base or _installed_config(BASE_FILE)
        bindings = args.bindings or _installed_config(BINDINGS_FILE)
        profile = render.load_yaml(args.profile)
        if not isinstance(profile, dict):
            raise render.RenderError(f'{args.profile}: the robot profile must be a mapping')
        mode = render.parse_mode(render.load_yaml(args.mode), args.mode)
        tree = render.render_files(base, bindings, args.profile, args.mode)
        issues = validation.validate_configuration(
            _catalog(args),
            profile,
            mode,
            tree,
            _site_map(args.site),
            _behavior_trees(tree, args.bt_dir),
            args.urdf.read_text(encoding='utf-8'),
        )
    except (render.RenderError, schema.SchemaError, LookupError, OSError) as error:
        print(f'navcfg validate: {error}', file=sys.stderr)
        return 2
    for issue in issues:
        print(f'{issue.severity.upper()} [{issue.rule}] {issue.message}')
    errors = sum(issue.severity == 'error' for issue in issues)
    warnings = sum(issue.severity == 'warn' for issue in issues)
    print(f'navcfg validate: {errors} error(s), {warnings} warning(s)')
    return 1 if errors else 0


def _doc_command(args: argparse.Namespace) -> int:
    """Execute ``navcfg doc``.

    Args:
        args: Parsed documentation arguments.

    Returns:
        Process exit status.
    """
    try:
        documentation.write_catalog(args.output, _catalog(args))
    except (schema.SchemaError, OSError) as error:
        print(f'navcfg doc: {error}', file=sys.stderr)
        return 2
    print(f'navcfg doc: wrote {args.output}')
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run navcfg.

    Args:
        argv: Arguments without the program name; ``sys.argv[1:]`` when None.

    Returns:
        0 on success, 1 for failed validation, or 2 for malformed inputs.
    """
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command == 'validate' and args.profile is not None:
        missing = [name for name in ('site', 'urdf', 'bt_dir') if getattr(args, name) is None]
        if missing:
            parser.error('--profile requires ' + ', '.join('--' + n.replace('_', '-')
                                                           for n in missing))
    if args.command == 'render':
        return _render_command(args)
    if args.command == 'validate':
        return _validate_command(args)
    if args.command == 'check':
        from nav_bringup.navcfg.live_cli import run

        return run(args)
    return _doc_command(args)
