#!/usr/bin/env bash
# Explicit no-hardware ROS integration; never part of automatic colcon test discovery.
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
  echo 'refusing ROS checks: user navigation/localization/chassis stack is running' >&2
  exit 3
fi

source /opt/ros/humble/setup.bash
source "${REFACTOR_WS}/install/setup.bash"
set -o allexport
source "${SANDBOX_ROS_ENV_FILE:-${REFACTOR_WS}/src/david_nav/config/ros_env.conf}"
set +o allexport
export ROS_DOMAIN_ID=99
ROS_LOG_DIR="$(mktemp -d -t wheeltec_driver_ros_log.XXXXXX)"
export ROS_LOG_DIR
echo "ROS sandbox logs: ${ROS_LOG_DIR}"
# The executable injects memory-only ByteStream implementations. It does not run
# the hardware driver entry point, open a device, or publish a nonzero Twist.
setsid timeout --signal=INT --kill-after=5s 45s \
  "${REFACTOR_WS}/build/wheeltec_driver/driver_node_checks" "$@"
for signal in INT TERM; do
  setsid timeout --signal=INT --kill-after=5s 10s \
    "${REFACTOR_WS}/build/wheeltec_driver/driver_signal_checks" "$signal"
done
