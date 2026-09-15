#!/usr/bin/env bash
# Only this explicit launcher runs ROS-node tests; colcon tests remain ROS-free.
set -eo pipefail

readonly REFACTOR_WS="${REFACTOR_WS:-$HOME/ros2_ws_refactor}"
readonly LIVE_PATTERN='run_loc_online|controller_server|bt_navigator|wheeltec_robot_node|wheeltec_driver_node|navigation_executor|dock_navigator|nav_task_manager|rviz2'

ancestors=" $$ "
pid=$$
while [[ -n "$pid" && "$pid" -gt 1 ]]; do
  pid="$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')"
  ancestors+="${pid} "
done
live=""
for pid in $(pgrep -f "$LIVE_PATTERN" || true); do
  [[ "$ancestors" == *" ${pid} "* ]] || live+="${pid},"
done
if [[ -n "$live" ]]; then
  ps -o pid=,args= -p "${live%,}"
  echo 'refusing ROS checks: navigation/localization/chassis stack is running' >&2
  exit 3
fi

source /opt/ros/humble/setup.bash
source "${REFACTOR_WS}/install/setup.bash"
set -o allexport
source "${SANDBOX_ROS_ENV_FILE:-${REFACTOR_WS}/src/david_nav/config/ros_env.conf}"
set +o allexport
export ROS_DOMAIN_ID=99
ROS_LOG_DIR="$(mktemp -d -t nav_plugins_ros_log.XXXXXX)"
export ROS_LOG_DIR
# Keep diagnostic logs for review; never remove an inherited/broad path.
echo "ROS sandbox logs: ${ROS_LOG_DIR}"
timeout --signal=INT --kill-after=5s 40s \
  "${REFACTOR_WS}/build/nav_plugins/scan_priority_layer_checks" "$@"
