"""为历史独立 Dock 命令提供仅 ROS 的兼容入口，复用共享执行器根."""

from pathlib import Path
import sys
from typing import Sequence

from nav_executor.app import cli
from nav_executor.app.logging import log_line


def _default_policy():
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory('david_config')) / 'dock/standalone.yaml'


def main(args: Sequence[str] | None = None) -> int:
    """转交旧 Dock 命令给共享运行器，并追加禁用 WebSocket 的 CLI 选项.

    未指定 Dock 文件时从资源包选择既有独立策略；不重写策略文件。
    帮助不需要资源查找；后续 ROS Dock 参数仍按既有最终优先级生效。

    Args:
        args: 不含程序名的 CLI 参数，None 时读取 sys.argv。

    Returns:
        共享入口状态码；捕获策略选择或转发异常并成功记录后为 1。
    """
    raw = list(sys.argv[1:] if args is None else args)
    if any(arg in ('-h', '--help') for arg in raw):
        return cli.main(raw)
    try:
        explicit, _ = cli.parse_arguments(raw)
        if 'dock_config_file' not in explicit:
            raw = ['--dock-config-file', str(_default_policy()), *raw]
        return cli.main([*raw, '--no-websocket'])
    except Exception as error:
        log_line(f'dock entry failed: {type(error).__name__}: {error}')
        return 1
