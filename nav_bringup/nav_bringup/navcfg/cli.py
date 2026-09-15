"""Command line of navcfg, the robot configuration tool (refactor plan §12).

``navcfg render`` writes the Nav2 parameter file of a robot profile and mode.
``navcfg validate`` applies schema and cross-file safety rules before launch.
``navcfg doc`` follows (PROGRESS P2.6).
"""

import argparse
import os
from pathlib import Path
import sys
from typing import Dict, Optional, Sequence

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
    validate_parser = commands.add_parser(
        'validate', help='check a robot profile, mode, site, Nav2 tree, BTs and URDF')
    validate_parser.add_argument('--profile', required=True, type=Path, help='robot profile')
    validate_parser.add_argument('--mode', required=True, type=Path, help='mode overlay')
    validate_parser.add_argument('--site', required=True, type=Path, help='site profile')
    validate_parser.add_argument('--urdf', required=True, type=Path, help='robot URDF')
    validate_parser.add_argument(
        '--bt-dir', required=True, action='append', type=Path,
        help='directory containing rendered BT file names; repeat to add search directories')
    validate_parser.add_argument('--base', type=Path, help='base parameter tree')
    validate_parser.add_argument('--bindings', type=Path, help='binding table')
    validate_parser.add_argument(
        '--source-root', type=Path,
        help='workspace src directory used to discover package schemas')
    validate_parser.add_argument(
        '--schema', action='append', type=Path,
        help='explicit package schema; repeat as needed (overrides schema discovery)')
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
        render.write_parameters(args.output, tree)
    except (render.RenderError, LookupError, OSError) as error:
        print(f'navcfg render: {error}', file=sys.stderr)
        return 2
    print(f'navcfg render: wrote {args.output}')
    return 0


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


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run navcfg.

    Args:
        argv: Arguments without the program name; ``sys.argv[1:]`` when None.

    Returns:
        0 on success, 1 for failed validation, or 2 for malformed inputs.
    """
    args = build_parser().parse_args(argv)
    if args.command == 'render':
        return _render_command(args)
    return _validate_command(args)
