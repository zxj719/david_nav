#!/usr/bin/env bash
# Explicit no-motion ROS checks; the Python guard refuses a live user stack.
set -e -o pipefail
source /opt/ros/humble/setup.bash
source "${REFACTOR_WS:-$HOME/ros2_ws_refactor}/install/setup.bash"
set -o allexport
source "${SANDBOX_ROS_ENV_FILE:?set the refactor ros_env.conf path explicitly}"
set +o allexport
export ROS_DOMAIN_ID=99
cd "$(dirname "$0")"
python3 -m pytest -p no:cacheprovider "$@" sandbox_*_checks.py
