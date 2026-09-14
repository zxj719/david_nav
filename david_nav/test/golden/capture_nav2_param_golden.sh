#!/usr/bin/env bash
# Capture Nav2 parameter golden files from the pre-refactor bringup.
#
# Starts 2d_david_nav2_bringup.launch.py in an isolated DDS domain with a
# static fake TF tree (map -> odom -> base_link) and no chassis, waits until
# /navigate_to_pose is available, dumps the effective parameters of every
# node, and then stops only the processes this script started.
#
# The dumps are the reference for PROGRESS P2.4: parameters rendered from the
# robot profile must be key-by-key equal to these files.
#
# Usage:
#   capture_nav2_param_golden.sh <tour|precise> <output_dir>
#
# Environment:
#   REFACTOR_WS            refactor workspace root (default: ~/ros2_ws_refactor)
#   GOLDEN_ROS_DOMAIN_ID   isolated DDS domain (default: 99)
#   READY_TIMEOUT_SEC      max wait for /navigate_to_pose (default: 120)
#
# Safety: refuses to run while a navigation, localization or chassis stack is
# running; never sends motion commands; never signals foreign processes.

set -o pipefail

readonly MODE="${1:?usage: $0 <tour|precise> <output_dir>}"
readonly OUT="${2:?usage: $0 <tour|precise> <output_dir>}"
readonly WS="${REFACTOR_WS:-$HOME/ros2_ws_refactor}"
readonly DOMAIN="${GOLDEN_ROS_DOMAIN_ID:-99}"
readonly READY_TIMEOUT_SEC="${READY_TIMEOUT_SEC:-120}"
readonly LIVE_PATTERN='run_loc_online|controller_server|bt_navigator|wheeltec_robot_node|navigation_executor|dock_navigator|rviz2'

case "${MODE}" in
  tour) readonly PARAMS_FILE="nav2_params_2d.yaml" ;;
  precise) readonly PARAMS_FILE="nav2_params_2d_precise.yaml" ;;
  *) echo "unknown mode: ${MODE} (expected tour or precise)" >&2; exit 2 ;;
esac

if pgrep -af "${LIVE_PATTERN}" | grep -v pgrep; then
  echo "refusing to run: a navigation/localization/chassis stack is running" >&2
  exit 3
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
set -o allexport
# shellcheck disable=SC1091
source "${WS}/src/david_nav/config/ros_env.conf"
set +o allexport
export ROS_DOMAIN_ID="${DOMAIN}"

mkdir -p "${OUT}/params"
readonly LOG_DIR="${OUT}/logs"
mkdir -p "${LOG_DIR}"

SESSIONS=()

start_in_session() {
  # Start a command as the leader of a new session and remember its id.
  local log_file="$1"
  shift
  setsid "$@" >"${log_file}" 2>&1 &
  SESSIONS+=("$!")
}

session_pids() {
  pgrep -s "$1" 2>/dev/null || true
}

cleanup() {
  local sid pid
  for sid in "${SESSIONS[@]}"; do
    for pid in $(session_pids "${sid}"); do
      kill -INT "${pid}" 2>/dev/null || true
    done
  done
  for _ in $(seq 1 30); do
    local alive=0
    for sid in "${SESSIONS[@]}"; do
      [[ -n "$(session_pids "${sid}")" ]] && alive=1
    done
    [[ "${alive}" == 0 ]] && break
    sleep 0.5
  done
  for sid in "${SESSIONS[@]}"; do
    for pid in $(session_pids "${sid}"); do
      kill -KILL "${pid}" 2>/dev/null || true
    done
  done
}
trap cleanup EXIT

start_in_session "${LOG_DIR}/tf_map_odom.log" \
  ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom
start_in_session "${LOG_DIR}/tf_odom_base_link.log" \
  ros2 run tf2_ros static_transform_publisher --frame-id odom --child-frame-id base_link
start_in_session "${LOG_DIR}/launch.log" \
  ros2 launch david_nav 2d_david_nav2_bringup.launch.py \
  "params_file:=${WS}/install/david_nav/share/david_nav/config/${PARAMS_FILE}" \
  cmd_shaping:=false
readonly LAUNCH_SID="${SESSIONS[2]}"

deadline=$((SECONDS + READY_TIMEOUT_SEC))
ready=0
while (( SECONDS < deadline )); do
  if [[ -z "$(session_pids "${LAUNCH_SID}")" ]]; then
    echo "launch exited before nav2 became ready" >&2
    tail -60 "${LOG_DIR}/launch.log" >&2
    exit 4
  fi
  if python3 "${WS}/src/david_nav/scripts/wait_nav_ready.py" --timeout 5 --quiet; then
    ready=1
    break
  fi
done
if [[ "${ready}" != 1 ]]; then
  echo "nav2 not ready within ${READY_TIMEOUT_SEC}s" >&2
  tail -80 "${LOG_DIR}/launch.log" >&2
  exit 5
fi
sleep 3

ros2 node list --no-daemon --spin-time 3 | sort >"${OUT}/nodes.txt"
ps -o pid=,args= -s "${LAUNCH_SID}" >"${LOG_DIR}/processes.txt"

dump_failures=0
while read -r node; do
  # Helper nodes created inside other processes (bt_navigator's *_rclcpp_node,
  # tf2 transform_listener_impl_*) expose no parameter services; dumping them
  # blocks forever. Our own fake TF publishers are not part of the golden set.
  case "${node}" in
    */static_transform_publisher* | */_ros2cli_* | */wait_nav_ready_probe) continue ;;
    *_rclcpp_node | */transform_listener_impl_*) continue ;;
  esac
  name="$(echo "${node#/}" | sed 's#/#__#g')"
  # Discovery without the daemon is occasionally too slow ("Node not found"),
  # so retry a few times and require a non-empty dump.
  dumped=0
  for attempt in 1 2 3; do
    if timeout 30 ros2 param dump --no-daemon --spin-time 5 "${node}" \
        >"${OUT}/params/${name}.yaml" 2>"${LOG_DIR}/dump_${name}.err" \
        && [[ -s "${OUT}/params/${name}.yaml" ]]; then
      dumped=1
      break
    fi
    echo "param dump attempt ${attempt} failed: ${node}" >&2
    sleep 2
  done
  if [[ "${dumped}" != 1 ]]; then
    echo "param dump failed: ${node}" >&2
    dump_failures=$((dump_failures + 1))
  fi
done <"${OUT}/nodes.txt"

{
  echo "mode: ${MODE}"
  echo "params_file: david_nav/config/${PARAMS_FILE}"
  echo "params_file_md5: $(md5sum "${WS}/src/david_nav/config/${PARAMS_FILE}" | cut -d' ' -f1)"
  echo "launch: david_nav/launch/2d_david_nav2_bringup.launch.py cmd_shaping:=false"
  echo "fake_tf: map->odom->base_link identity (static)"
  echo "ros_domain_id: ${DOMAIN}"
  echo "git_head: $(git -C "${WS}/src" rev-parse HEAD)"
  echo "captured_at: $(date --iso-8601=seconds)"
  echo "dump_failures: ${dump_failures}"
} >"${OUT}/MANIFEST.yaml"

echo "captured $(ls "${OUT}/params" | wc -l) parameter files into ${OUT} (failures: ${dump_failures})"
exit "${dump_failures}"
