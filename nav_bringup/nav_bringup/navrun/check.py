"""Validate frozen run configuration before an optional read-only ROS observation.

Offline checks recompute schema and cross-file safety rules using the saved
inputs, rendered Nav2 tree, BTs and URDF; checks.yaml is not trusted as evidence.
This is not hardware acceptance, a byte-integrity audit or a liveness check.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
from typing import Any, Dict, Tuple

import yaml

from nav_bringup.launching.bundle import load_options
from nav_bringup.launching.graph import GraphOptions, NAVIGATION_MODES
from nav_bringup.navcfg import render, schema, validation
from nav_bringup.navrun.record_reader import open_record
from nav_bringup.navrun.teleop_options import apply_options, TeleopOptions
from nav_bringup.recorder.run_dir import default_runs_root


def _root(args: argparse.Namespace) -> Path:
    if args.run_dir is not None:
        return args.run_dir.expanduser().resolve(strict=True)
    if args.runs_root is None and os.environ.get('NAVRUN_RUN_DIR'):
        return Path(os.environ['NAVRUN_RUN_DIR']).expanduser().resolve(strict=True)
    root = (args.runs_root or default_runs_root()).expanduser().absolute()
    with open_record(root, 'latest') as reader:
        return root / reader.run_id


def _mapping(path: Path) -> Dict[str, Any]:
    result = render.load_yaml(path)
    if not isinstance(result, dict):
        raise ValueError('expected a frozen configuration mapping')
    return result


def _catalog(args: argparse.Namespace) -> schema.ParameterCatalog:
    paths = args.schema
    if not paths:
        paths = (schema.find_source_schemas(args.source_root.expanduser())
                 if args.source_root is not None else
                 schema.find_installed_schemas(schema.ament_prefixes(os.environ)))
    if not paths:
        raise ValueError('no configuration schemas found')
    return schema.load_catalog(paths)


def _trees(root: Path, tree: Dict[str, Any]) -> Dict[str, str]:
    """读取准备目录所属的行为树 XML，供离线核对而不执行行为树.

    Args:
        root: 已准备运行目录。
        tree: 含 bt_navigator 参数的渲染树。

    Returns:
        文件名到 XML 原文的映射；三个配置键可引用同一文件。

    Raises:
        ValueError: 路径为空、不是绝对路径或解析后不在预备 BT 目录。
        OSError: 目录或文件不存在、不可读取。
    """
    directory = (root / 'params/rendered/bt').resolve(strict=True)
    result = {}
    for key in ('default_bt_xml_filename', 'default_nav_to_pose_bt_xml',
                'default_nav_through_poses_bt_xml'):
        value = render.get_path(tree, 'bt_navigator/ros__parameters/' + key)
        if not isinstance(value, str) or not value:
            raise ValueError('expected a prepared behavior tree path')
        path = Path(value)
        if not path.is_absolute() or path.resolve(strict=True).parent != directory:
            raise ValueError('behavior tree must belong to the prepared run')
        result[path.name] = path.read_text(encoding='utf-8')
    return result


def offline(
    root: Path, catalog: schema.ParameterCatalog,
) -> Tuple[GraphOptions, Tuple[validation.ValidationIssue, ...]]:
    """Recheck the selected run using its frozen artifacts and the current schema.

    Args:
        root: Trusted existing prepared directory, not an arbitrary untrusted archive.
        catalog: Current explicitly selected, installed or source-owned schemas.

    Returns:
        Launch options and recomputed safety findings, without querying ROS.

    Raises:
        ValueError: A required artifact, identity, launch graph or structure is invalid.
        OSError: Required configuration cannot be read.
    """
    options = load_options(str(root), {})
    launch = json.loads((root / 'launch.json').read_text(encoding='utf-8'))
    profile = _mapping(root / 'params/inputs/robot.yaml')
    mode_path = root / 'params/inputs/mode.yaml'
    mode = render.parse_mode(_mapping(mode_path), mode_path)
    effective = render.deep_merge(profile, mode.profile)
    if (mode.name != options.mode or render.profile_value(effective, 'robot.name')
            != launch['robot'] or type(profile.get('schema_version')) is not int
            or profile['schema_version'] != 1):
        raise ValueError('frozen profile/mode identity does not match the launch bundle')
    if options.mode in NAVIGATION_MODES:
        site = _mapping(root / 'params/inputs/site.yaml')
        if render.profile_value(site, 'site.name') != launch['site']:
            raise ValueError('frozen site identity does not match the launch bundle')
        tree = _mapping(root / 'params/rendered/nav2.yaml')
        issues = validation.validate_configuration(
            catalog, profile, mode, tree, _mapping(root / 'params/inputs/map.yaml'),
            _trees(root, tree), (root / 'params/inputs/robot.urdf').read_text(encoding='utf-8'))
    else:
        teleop_path = root / 'params/inputs/teleop_options.yaml'
        if teleop_path.exists():
            effective = apply_options(effective, options.mode,
                                      TeleopOptions(**_mapping(teleop_path)))
        issues = [validation.ValidationIssue('schema', item.severity, item.message)
                  for item in catalog.check_profile(effective)]
        if options.mode == 'teleop':
            issues.extend(validation.validate_teleop_limits(effective))
    return options, tuple(issues)


def run(args: argparse.Namespace) -> int:
    """Dispatch offline validation and opt-in live verification of the same run.

    Args:
        args: Run selector, schema sources, live flag, strict flag and timing bounds.

    Returns:
        Zero for offline success, one for safety errors, two for invalid inputs;
        online verification additionally preserves pending and signal exit statuses.
    """
    try:
        root = _root(args)
        options, issues = offline(root, _catalog(args))
        for issue in issues:
            print(f'{issue.severity.upper()} [{issue.rule}] {issue.message}')
        errors = sum(issue.severity == 'error' for issue in issues)
        warnings = sum(issue.severity == 'warn' for issue in issues)
        print(f'navrun check: offline {errors} error(s), {warnings} warning(s)')
        if errors:
            return 1
        if not args.live:
            return 0
        if options.mode not in NAVIGATION_MODES:
            print('navrun check: live localization checks require a navigation mode',
                  file=sys.stderr)
            return 2
    except (ValueError, LookupError, OSError, TypeError, yaml.YAMLError) as error:
        print('navrun check: invalid prepared configuration (' + type(error).__name__ + ')',
              file=sys.stderr)
        return 2
    from nav_bringup.navcfg import live_cli

    try:
        return live_cli.run(argparse.Namespace(
            live=True, run_dir=root, strict=args.strict, wait_timeout=args.wait_timeout,
            sample_sec=args.sample_sec, parameter_timeout=args.parameter_timeout,
            wait_localization_only=False, events=None))
    except ImportError:
        print('navrun check: live ROS dependencies unavailable', file=sys.stderr)
        return 2
