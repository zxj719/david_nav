#!/usr/bin/env bash
# Read-only observations plus a private zero-only probe; never submit a goal.
set -eo pipefail
source /opt/ros/humble/setup.bash
source "${SANDBOX_INSTALL_SETUP:-$HOME/ros2_ws_refactor/install/setup.bash}"
set -o allexport
source "${SANDBOX_ROS_ENV_FILE:-$HOME/ros2_ws_refactor/src/david_system_env/config/ros_env.conf}"
set +o allexport
export ROS_DOMAIN_ID=99
cd "$(dirname "$0")/.."
exec nice -n 19 ionice -c3 python3 -m pytest -q -s \
  sandbox_tests/sandbox_selftest_startup_checks.py "$@"
