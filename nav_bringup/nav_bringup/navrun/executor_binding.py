"""把冻结的场地/档案设置翻译成原样未改的旧执行器命令行.

只做翻译：旧执行器的参数名与语义保持不变，迁移期两边可以逐项对照。
"""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
from typing import Any, Mapping
from urllib.parse import urlsplit

from nav_bringup.launching.graph import GraphOptions
from nav_bringup.navcfg import render


def _path(value: Any, directory: Path) -> str:
    """将场地路径相对指定目录解析为绝对路径，并展开用户目录及链接.

    此处只解析路径，不保证文件存在或内容可信；不改变配置文件。

    Args:
        value: 非空路径字符串，空白只用于校验而不从有效路径中剔除。
        directory: 相对路径所依据的场地配置目录。

    Returns:
        resolve 后的路径字符串。

    Raises:
        ValueError: 路径不是非空字符串。
        OSError: 路径解析失败。
        RuntimeError: 解析遇到符号链接循环。
    """
    if not isinstance(value, str) or not value.strip():
        raise ValueError('executor site paths must be nonempty strings')
    path = Path(value).expanduser()
    return str((path if path.is_absolute() else directory / path).resolve())


def _duration(value: Any, name: str, allow_zero: bool = False) -> float:
    """验证有限的秒制预算，明确区分允许零和严格为正的策略.

    布尔值不作为数值接收，不从字符串推断单位。

    Args:
        value: 整数或浮点秒数。
        name: 错误信息中的预算名称。
        allow_zero: True 允许零，否则要求严格大于零。

    Returns:
        转成 float 的已检查预算。

    Raises:
        ValueError: 类型、有限性或符号不符合策略。
        OverflowError: 超大整数在有限性检查时无法转换。
    """
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0 or (value == 0 and not allow_zero)):
        requirement = 'nonnegative' if allow_zero else 'positive'
        raise ValueError(f'{name} must be finite and {requirement}')
    return float(value)


def bind_executor(
    options: GraphOptions, profile: Mapping[str, Any], site: Mapping[str, Any],
    site_directory: Path,
) -> GraphOptions:
    """绑定服务端地址、点位路径、重试间隔，以及只在精准模式启用的入位路由.

    Args:
        options: 选定的启动图；已关闭或已显式配置的执行器保持原样。
        profile: 执行器与启动时序所用的生效机器人/模式设置。
        site: 冻结的场地映射，含 agent_server 与 pois。
        site_directory: 供相对点位/存储路径使用的原场地目录。

    Returns:
        更新后的不可变启动图选项。过程中不打开也不写入任何点位/存储文件，
        不导入运行期实现，且每个值都保持为单独一个 argv 项（不靠空格再分词）。

    Raises:
        ValueError: 服务端、路径、时序、路由或选定模式非法。
    """
    if not options.start_executor or options.executor_arguments:
        return options
    if options.mode not in ('tour', 'precise'):
        raise ValueError('legacy executor requires tour or precise mode')
    uri = render.profile_value(site, 'agent_server.uri')
    if not isinstance(uri, str) or not uri.strip():
        raise ValueError('agent_server.uri must be a nonempty WebSocket URI')
    parsed = urlsplit(uri)
    if parsed.scheme not in ('ws', 'wss') or not parsed.hostname:
        raise ValueError('agent_server.uri must use ws/wss and include a host')
    # Accessing port validates malformed explicit ports without making a connection.
    if parsed.port is not None and parsed.port == 0:
        raise ValueError('agent_server.uri port must be positive')
    settings, startup = profile.get('executor', {}), profile.get('startup', {})
    if not isinstance(settings, Mapping) or not isinstance(startup, Mapping):
        raise ValueError('executor and startup settings must be mappings')
    delay = _duration(settings.get('reconnect_delay_sec', 3.0), 'reconnect delay', True)
    timeout = _duration(startup.get('nav_ready_timeout_sec', 120.0), 'navigation ready timeout')
    args = [
        '--server-uri', uri, '--reconnect-delay', f'{delay:g}',
        '--pose-file', _path(render.profile_value(site, 'pois.named_poses_file'), site_directory),
        '--poi-store-file', _path(render.profile_value(site, 'pois.store_file'), site_directory),
        '--action-name', options.ready_action,
    ]
    if options.mode == 'precise':
        identifiers = site.get('pois', {}).get('dock_poi_ids', [])
        if not isinstance(identifiers, list) or not all(
                isinstance(identifier, str) and identifier.strip() for identifier in identifiers):
            raise ValueError('pois.dock_poi_ids must be a list of nonempty strings')
        if identifiers:
            args += ['--dock-action-name', '/dock_to_pose']
            for identifier in dict.fromkeys(item.strip() for item in identifiers):
                args += ['--dock-poi-id', identifier]
    return replace(options, executor_arguments=tuple(args), ready_timeout_sec=timeout)
