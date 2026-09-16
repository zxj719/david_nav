#!/usr/bin/env python3
"""Compatibility entry; implementation has moved to nav_executor.app.cli.

Keep this executable path for headless launch and process matching.
"""

from nav_executor.app.cli import main


if __name__ == '__main__':
    raise SystemExit(main())
