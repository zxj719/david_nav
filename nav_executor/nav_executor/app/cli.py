"""保持冻结帮助及参数接口，由外层运行器负责启动 ROS."""

import argparse
import asyncio
from pathlib import Path
import sys
from typing import Sequence

from nav_tasks.api import ConfigError


def build_help_text() -> str:
    """返回逐字冻结的旧帮助文本，不导入 ROS 或读取配置.

    Returns:
        包含历史可执行文件名的 G16 帮助文本，不含 print 追加的尾换行。
    """
    return '\n'.join([
        'usage: navigation_executor.py [--server-uri URI] [--poi-store-file PATH] '
        '[--pose-file PATH] [ROS args...]',
        '',
        'Robot-side WebSocket navigation executor for david_nav.',
        '',
        'options:',
        '  --server-uri URI        Agent WebSocket server (nav WS v1). Default:',
        '                          $NAV_WS_URL or ws://127.0.0.1:8101',
        '  --poi-store-file PATH   Local YAML file used to persist executor-owned POIs',
        '  --pose-file PATH        named_poses.json to import into the POI store at startup',
        '  --action-name NAME      NavigateToPose action server to drive',
        '                          (default: /navigate_to_pose; waypoint stack uses',
        '                          /waypoint_follower/navigate)',
        '  --reconnect-delay SEC   Delay between WebSocket reconnect attempts (default: 3.0)',
        '  --nav-retry-count N     Silent re-publish attempts after a nav failure before',
        '                          reporting on_error to the server (default: 1)',
        '  --nav-retry-delay SEC   Wait between a nav failure and the re-publish (default: 1.0)',
        '  --dock-poi-id ID        Navigate this POI with two-stage precise docking through',
        '                          the docking action (repeatable; never snapped or retried)',
        '  --dock-action-name NAME Docking action server (default: /dock_to_pose)',
        '  -h, --help              Show this help message and exit',
    ])


class _Parser(argparse.ArgumentParser):
    def error(self, message):
        raise ConfigError(message)


def parse_arguments(raw_args: Sequence[str]) -> tuple[dict, list[str]]:
    """解析已知应用参数，禁用缩写并把未知参数保留给 ROS.

    只处理 CLI 语法和已声明类型转换，不执行完整配置有效性或优先级校验。

    Args:
        raw_args: 不含可执行文件名的参数序列。

    Returns:
        显式覆盖字典和保持输入顺序的未知参数列表；未提供的选项不填默认值。

    Raises:
        ConfigError: 已知选项缺少值或声明的数值类型转换失败。
    """
    from nav_executor.app.config import ExecutorConfig
    from dataclasses import fields

    parser = _Parser(add_help=False, allow_abbrev=False, argument_default=argparse.SUPPRESS)
    parser.add_argument('-h', '--help', action='store_true')
    parser.add_argument('--config')
    parser.add_argument('--robot-profile')
    for field in fields(ExecutorConfig):
        if field.name == 'dock_poi_ids':
            parser.add_argument('--dock-poi-id', dest=field.name, action='append')
        elif field.name == 'websocket_enabled':
            parser.add_argument('--websocket', dest=field.name,
                                action=argparse.BooleanOptionalAction)
        else:
            flag = '--' + field.name.replace('_', '-')
            if field.name == 'nav_retry_delay_sec':
                flag = '--nav-retry-delay'
            parser.add_argument(flag, dest=field.name,
                                type=field.type if field.type in (int, float) else str)
    parsed, remaining = parser.parse_known_args(list(raw_args))
    return vars(parsed), remaining


def main(args: Sequence[str] | None = None) -> int:
    """运行配置后的执行器，帮助在 ROS 启动或配置文件访问前返回.

    任一 -h/--help 参数都选择冻结帮助路径，不要求其它参数有效。
    非帮助路径按配置装配运行器；捕获的失败按既有日志格式报告。

    Args:
        args: 显式参数序列，None 时使用 sys.argv 去掉程序名后的部分。

    Returns:
        帮助或正常退出为 0；正常信号收尾返回 128 加信号编号，
        捕获键盘中断为 130，捕获普通异常并成功记录后为 1。
    """
    raw = list(sys.argv[1:] if args is None else args)
    if any(arg in ('-h', '--help') for arg in raw):
        print(build_help_text())
        return 0
    from nav_executor.app.config import load_configuration
    from nav_executor.app.logging import log_line
    from nav_executor.app.signals import run_with_signals

    try:
        explicit, _ = parse_arguments(raw)
        profile = explicit.get('robot_profile')
        if profile is None:
            from ament_index_python.packages import get_package_share_directory
            profile = Path(get_package_share_directory('david_config')) / 'robot' / 'david.yaml'
        resolved = load_configuration(raw, profile_path=Path(profile))
        return asyncio.run(run_with_signals(resolved))
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        log_line(f'executor failed: {type(error).__name__}: {error}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
