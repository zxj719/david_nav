#!/usr/bin/env python3
"""Compatibility entry; implementation has moved to nav_executor.app.dock_cli.

Keep the standalone ROS-only command; defaults live in david_config.
"""

from nav_executor.app.dock_cli import main


if __name__ == '__main__':
    raise SystemExit(main())
