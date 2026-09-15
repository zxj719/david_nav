"""Use the installed delegated keyboard with a private domain-99 lease path."""

import os
from pathlib import Path
import sys

from chassis_teleop import node


if __name__ == '__main__':
    private_lock = Path(sys.argv[1])
    if os.environ.get('ROS_DOMAIN_ID') != '99' or not private_lock.is_absolute():
        raise SystemExit('fixture requires domain 99 and a private absolute lease path')
    node.DEFAULT_LOCK_PATH = private_lock
    sys.exit(node.managed_main(sys.argv[2:]))
