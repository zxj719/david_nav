"""按名字选配置，并复用与启动完全相同的离线运行包校验.

不创建运行目录，不做主机/进程发现，不查可执行文件，不建 ROS 上下文，
也不执行任何运行期命令。通过只说明选中的源资源自洽，
不说明可执行文件已安装、图能起来、硬件正常或部署可用。
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys
from typing import Callable, Tuple

import yaml

from nav_bringup.launching.graph import GraphOptions, MODES, NAVIGATION_MODES
from nav_bringup.navcfg import render, schema
from nav_bringup.navrun.preparation import BundleInputs, PreparedBundle, prepare_bundle
from nav_bringup.navrun.up_planning import discover_inputs, UpRequest


def _share(args: argparse.Namespace) -> Callable[[str], Path]:
    """创建只读包资源解析器，优先使用显式源码根目录.

    Args:
        args: 含可选 source_root 的命令行选项。

    Returns:
        包名到目录 Path 的函数；源码模式要求 package.xml 存在，
        安装模式延迟导入 ament 索引。资源不存在时由返回函数抛 LookupError。
    """
    def resolve(package: str) -> Path:
        if args.source_root is not None:
            path = args.source_root.expanduser().absolute() / package
            if not (path / 'package.xml').is_file():
                raise LookupError(f'package resources not found: {path}')
            return path
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(package))

    return resolve


def _select(args: argparse.Namespace) -> Tuple[UpRequest, BundleInputs]:
    """按机器人、模式和场地名称解析离线校验输入，不创建运行目录.

    Args:
        args: 命名资源接口的已解析选项，不允许混用显式文件覆盖。

    Returns:
        UpRequest 与 BundleInputs；无场地参数时仅在唯一场地可选时自动选择。

    Raises:
        ValueError: 名称/模式/覆盖组合无效，或导航模式场地选择不唯一。
        LookupError: 包资源不存在。
        OSError: 资源目录无法读取。
    """
    mode, robot = str(args.mode), args.robot
    site = str(args.site) if args.site is not None else None
    if mode not in MODES:
        raise ValueError('--robot requires a supported mode name, not an overlay path')
    for label, name in (('robot', robot), ('site', site)):
        if name is not None and re.fullmatch(r'[a-z][a-z0-9_]*', name) is None:
            raise ValueError(f'--{label} must be a lowercase resource name, not a path')
    if any(value is not None for value in (args.urdf, args.bt_dir, args.base, args.bindings)):
        raise ValueError('--robot cannot be mixed with explicit-file overrides; use --profile')
    navigation = mode in NAVIGATION_MODES
    if not navigation and site is not None:
        raise ValueError('--site is not used by chassis/teleop validation')
    share = _share(args)
    if navigation and site is None:
        directory = share(robot + '_config') / 'sites'
        choices = sorted(path.name for path in directory.iterdir()
                         if re.fullmatch(r'[a-z][a-z0-9_]*', path.name)
                         and (path / 'site.yaml').is_file())
        if len(choices) != 1:
            raise ValueError('use --site explicitly unless the robot has exactly one site')
        site = choices[0]
    # This is only a destination string for in-memory render bindings; no writer
    # is called, and neither existence nor write access is required here.
    request = UpRequest(mode, robot, site or 'none', Path('/__navcfg_validation__'))
    return request, discover_inputs(request, share)


def _prepare(request: UpRequest, inputs: BundleInputs,
             catalog: schema.ParameterCatalog) -> PreparedBundle:
    """以普通准备器离线渲染并交叉核对选定身份及输入快照.

    Args:
        request: 仅用于渲染绑定的机器人、模式和场地选择。
        inputs: 只读源码资源路径。
        catalog: 参数声明目录，用于常规配置验证。

    Returns:
        内存中的 PreparedBundle；不写文件、不启动进程或探测 ROS。

    Raises:
        ValueError: 档案结构、身份或渲染验证无效，或读取期间输入发生变化。
        OSError: 输入无法读取。
        yaml.YAMLError: 输入不是合法 YAML。
    """
    profile = yaml.safe_load(inputs.profile.read_bytes())
    if not isinstance(profile, dict):
        raise ValueError('robot profile must be a mapping')
    mode = render.parse_mode(yaml.safe_load(inputs.mode.read_bytes()), inputs.mode)
    options = GraphOptions(
        request.mode, str(request.run_dir), start_executor=request.mode in ('tour', 'precise'),
        supervisor_keyboard=request.mode == 'teleop')
    bundle = prepare_bundle(options, inputs, catalog)
    if bundle.robot != request.robot or (
            request.mode in NAVIGATION_MODES and bundle.site != request.site):
        raise ValueError('selected robot/site identity does not match configuration')
    artifacts = dict(bundle.artifacts)
    if (yaml.safe_load(artifacts['params/inputs/robot.yaml']) != profile
            or render.parse_mode(yaml.safe_load(artifacts['params/inputs/mode.yaml']),
                                 inputs.mode) != mode):
        raise ValueError('configuration changed during validation; retry')
    return bundle


def run(args: argparse.Namespace, catalog: schema.ParameterCatalog) -> int:
    """选定并校验一个机器人包，不执行任何启动操作.

    Args:
        args: 已解析的机器人/模式/场地选择子，以及可选的源资源根目录。
        catalog: 调用方显式选定的 schema，可以来自源码树或已安装位置。

    Returns:
        离线配置有效返回 0；配置被拒返回 1；
        选择子格式错误、资源缺失或文件不可访问返回 2。
    """
    try:
        request, inputs = _select(args)
    except (ValueError, LookupError, OSError) as error:
        print(f'navcfg validate: {error}', file=sys.stderr)
        return 2
    try:
        bundle = _prepare(request, inputs, catalog)
    except (OSError, LookupError) as error:
        print(f'navcfg validate: {error}', file=sys.stderr)
        return 2
    except (ValueError, yaml.YAMLError) as error:
        print(f'navcfg validate: configuration rejected: {error}', file=sys.stderr)
        return 1
    print(f'navcfg validate: robot={bundle.robot} mode={bundle.options.mode} '
          f'site={bundle.site} (offline)')
    for issue in bundle.issues:
        print(f'{issue.severity.upper()} [{issue.rule}] {issue.message}')
    print(f'navcfg validate: 0 error(s), {len(bundle.issues)} warning(s)')
    return 0
