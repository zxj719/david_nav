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
    """按需渲染 BT 资源并原地绑定参数树中的输出路径.

    只发现 ament 资源，不加载插件实现，也不写输出文件。缺省路径从
    nav_plugins 取得；绑定过程不是事务，后续绑定失败可能留下部分树修改。

    Args:
        args: render 子命令参数，含档案、模式及可选 BT 资源和输出目录。
        tree: 已渲染的 Nav2 参数树；成功时三处 BT 路径指向输出目录。

    Returns:
        XML 文件名到文本的映射；未请求 BT 输出且无冲突选项时为空。

    Raises:
        render.RenderError: 选项冲突、档案、模式、模板或输出路径绑定不合法。
        LookupError: 缺省资源所需的 ament 包无法找到。
        OSError: 无法读取资源文件。
    """
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
    """按显式文件、源码根或安装前缀的优先级加载参数 schema.

    Args:
        args: 含 schema 和 source_root 的命令行参数；显式 schema 优先，
            二者均未指定时从当前进程的 ament 前缀发现。

    Returns:
        合并且校验过的包参数目录；不修改输入文件。

    Raises:
        schema.SchemaError: 没有找到声明文件，文件读取或 YAML 解析失败，
            或声明内容、合并规则不合法；文件读取异常由加载器包装。
    """
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
    """读取场地档案所选择的地图 YAML，不加载地图图像.

    Args:
        site_path: 场地档案文件；map.nav_yaml 先展开用户目录，
            相对路径再相对于此文件的父目录解析。

    Returns:
        地图 YAML 的顶层字典；不在这里检查 trinary 模式或图像可读性。

    Raises:
        render.RenderError: 场地或地图不是映射，地图路径缺失、类型错误，
            或文件读取和 YAML 解析失败；load_yaml 将读取异常包装为此类型。
    """
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
    """读取渲染参数指定的各个 BT 文件，保留搜索目录优先级.

    相同参数值先去重；绝对路径直接读取，相对路径选首个存在的候选文件。
    只读取文本，不在此校验 XML 或插件，不改参数树。

    Args:
        tree: 包含 bt_navigator 三个 BT 路径参数的已渲染树。
        directories: 相对 BT 路径的有序搜索目录。

    Returns:
        选中文件的路径字符串到 UTF-8 文本的映射。

    Raises:
        render.RenderError: 参数路径缺失、值非非空字符串或文件无法找到。
        OSError: 选中的文件无法读取。
    """
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
    """执行离线配置验证，把规则问题打印后转换为退出码.

    robot 选择器交给机器人档案工作流，显式 profile 则读取场地、BT 和 URDF。
    RenderError、SchemaError、LookupError 与 OSError 在本边界打印后返回 2，
    包括本函数显式抛出的档案格式错误；它们不是正常输入错误的对外异常接口。

    Args:
        args: 已经由命令行解析并检查必需组合的 validate 参数。

    Returns:
        0 表示没有规则错误（可有警告），1 表示规则错误，2 表示输入或读取失败；
        robot 分支沿用其工作流的退出码。
    """
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
