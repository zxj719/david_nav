"""ROS-only compatibility entry for the historical standalone Dock command."""

from pathlib import Path
import sys
from typing import Sequence

from nav_executor.app import cli
from nav_executor.app.logging import log_line


def _default_policy():
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory('david_config')) / 'dock/standalone.yaml'


def main(args: Sequence[str] | None = None) -> int:
    """Forward the old Dock command to the shared root with WS explicitly disabled.

    Args:
        args: CLI values excluding the executable, or None to read sys.argv.

    Returns:
        Shared-root status, or one if the explicit/default policy cannot be selected.
        Help requires no resource lookup. ROS parameters retain final precedence.
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
