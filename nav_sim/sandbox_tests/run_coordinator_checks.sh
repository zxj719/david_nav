#!/usr/bin/env bash
# Actual CLI and read-only coordinator; never enables goal dispatch.
set -eo pipefail
source /opt/ros/humble/setup.bash
source "${SANDBOX_INSTALL_SETUP:-$HOME/ros2_ws_refactor/install/setup.bash}"
set -o allexport
source "${SANDBOX_ROS_ENV_FILE:-$HOME/ros2_ws_refactor/src/david_system_env/config/ros_env.conf}"
set +o allexport
export ROS_DOMAIN_ID=99
cd "$(dirname "$0")/.."
exec nice -n 19 ionice -c3 python3 -m pytest -q -s \
  sandbox_tests/sandbox_selftest_cli_checks.py "$@"
