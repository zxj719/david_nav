#!/usr/bin/env bash
# Run the david_nav sandbox integration checks in an isolated DDS domain.
#
# Usage:
#   run_sandbox_checks.sh [pytest arguments]
#
# Environment:
#   REFACTOR_WS             workspace root with install/ (default: ~/ros2_ws_refactor)
#   SANDBOX_ROS_DOMAIN_ID   isolated DDS domain (default: 99)
#   SANDBOX_ROS_ENV_FILE    optional DDS environment file to load first
#
# Safety: refuses to run while a navigation, localization or chassis stack is
# running. The checks stop only the processes they started and never publish
# velocity commands.

set -o pipefail

readonly WS="${REFACTOR_WS:-$HOME/ros2_ws_refactor}"
readonly LIVE_PATTERN='run_loc_online|controller_server|bt_navigator|wheeltec_robot_node|wheeltec_driver_node|navigation_executor|dock_navigator|rviz2'

# pgrep -f matches whole command lines, so the shells that started this script
# (for example `ssh host "... <process name> ..."`) would match; ignore them.
ancestors=" $$ "
pid=$$
while [[ -n "$pid" && "$pid" -gt 1 ]]; do
  pid="$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')"
  ancestors+="${pid} "
done
live=""
for pid in $(pgrep -f "${LIVE_PATTERN}"); do
  [[ "$ancestors" == *" ${pid} "* ]] || live+="${pid},"
done
if [[ -n "$live" ]]; then
  ps -o pid=,args= -p "${live%,}"
  echo "refusing to run: a navigation/localization/chassis stack is running" >&2
  exit 3
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
if [[ -n "${SANDBOX_ROS_ENV_FILE:-}" ]]; then
  set -o allexport
  # shellcheck disable=SC1090
  source "${SANDBOX_ROS_ENV_FILE}"
  set +o allexport
fi
export SANDBOX_ROS_DOMAIN_ID="${SANDBOX_ROS_DOMAIN_ID:-99}"
export ROS_DOMAIN_ID="${SANDBOX_ROS_DOMAIN_ID}"

# Keep the log files of these ROS processes out of ~/.ros/log.
ROS_LOG_DIR="$(mktemp -d -t david_nav_sandbox_ros_log.XXXXXX)" || exit 2
export ROS_LOG_DIR
trap 'rm -rf "${ROS_LOG_DIR}"' EXIT

cd "$(dirname "$0")" || exit 2
python3 -m pytest -p no:cacheprovider "$@" sandbox_*_checks.py
