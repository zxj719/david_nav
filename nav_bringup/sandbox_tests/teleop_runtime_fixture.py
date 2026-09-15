"""Run installed keyboard code with a sandbox-owned lock, never the host navrun lock."""

from pathlib import Path
import sys

from chassis_teleop import node
from nav_bringup.navrun.locking import RunLock


if __name__ == '__main__':
    lock_path = Path(sys.argv[1])
    node.RunLock = lambda: RunLock(lock_path)
    sys.exit(node.main(sys.argv[2:]))
