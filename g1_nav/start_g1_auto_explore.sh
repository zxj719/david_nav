#!/usr/bin/env bash

set -euo pipefail

SCRIPT_NAME="$(basename "$0")"
RVIZ=true
REALSENSE=true
DRY_RUN=false
SUDO_PASSWORD="${SUDO_PASSWORD:-123}"
SLAM_START_DELAY="${SLAM_START_DELAY:-5}"
SHUTDOWN_GRACE_SECONDS="${SHUTDOWN_GRACE_SECONDS:-5}"
ENABLE_CPU_ONLINE="${ENABLE_CPU_ONLINE:-true}"
ROS_DOMAIN_ID_DEFAULT="${ROS_DOMAIN_ID_DEFAULT:-30}"
ROS_SETUP="/opt/ros/humble/setup.bash"
SLAM_WORKSPACE="/home/unitree/lightning-lm-livox"
AUTO_WORKSPACE="/home/unitree/ros2_ws"
STOP_WORKSPACE="/home/unitree/ros2_ws/src/g1_cmd"
UNITREE_CHANNEL_NAME="${UNITREE_CHANNEL_NAME:-enP8p1s0}"
FALLBACK_KILL_REGEX="${FALLBACK_KILL_REGEX:-ros2 launch g1_nav g1_auto_explore.launch.py|run_slam_online|static_transform_publisher 0 0 0 0 0 0 map odom|frontier_explorer_cpp|collision_monitor|g1_move|g1_sdk_worker|controller_server|planner_server|bt_navigator|behavior_server|lifecycle_manager|map_warmup_spin|realsense_depth_to_scan|depthimage_to_laserscan_node|pointcloud_to_laserscan_node|rviz2}"
TF_COMMAND="${TF_COMMAND:-}"
SLAM_COMMAND="${SLAM_COMMAND:-}"
AUTO_COMMAND="${AUTO_COMMAND:-}"
STOP_COMMAND="${STOP_COMMAND:-}"

declare -a CHILD_PIDS=()
declare -a CHILD_NAMES=()
declare -a CHILD_PGIDS=()
CLEANUP_DONE=false

usage() {
  cat <<'EOF'
Usage:
  start_g1_auto_explore.sh [--no-rviz] [--no-realsense] [--dry-run]
  start_g1_auto_explore.sh [-r] [-s] [-d]

Behavior:
  Start TF, slam2, and auto in one shell as managed background processes.
  Press Ctrl+C in this shell to stop every managed process cleanly.

Options:
  --rviz            Enable RViz (default).
  --no-rviz         Disable RViz.
  -r                Disable RViz.
  --realsense       Enable Realsense obstacle avoidance (default).
  --no-realsense    Disable Realsense depth-to-scan bridge.
  -s                Disable Realsense depth-to-scan bridge.
  --dry-run         Print commands without starting them.
  -d                Print commands without starting them.
  -h, --help        Show this help.

Shortcuts:
  g1go              Same as start_g1_auto_explore.sh
  g1go -r           No RViz
  g1go -s           No Realsense
  g1go -rs          No RViz and no Realsense
  g1go -d           Dry run

Environment variables:
  ROS_DOMAIN_ID_DEFAULT     Default ROS 2 domain ID when ROS_DOMAIN_ID is unset. Default: 30
  UNITREE_CHANNEL_NAME      Unitree SDK channel for zero-velocity stop. Default: enP8p1s0
  FALLBACK_KILL_REGEX       Extra pkill regex for shutdown cleanup.
  SUDO_PASSWORD            Password used for sudo. Default: 123
  SLAM_START_DELAY         Seconds to wait after starting slam2 before auto. Default: 5
  SHUTDOWN_GRACE_SECONDS   Seconds to wait before force kill. Default: 5
  ENABLE_CPU_ONLINE        true/false. Whether to online cpu4-cpu7. Default: true
EOF
}

log() {
  printf '[%s] %s\n' "$SCRIPT_NAME" "$*"
}

parse_short_flags() {
  local arg="$1"
  local idx flag

  for ((idx = 1; idx < ${#arg}; idx++)); do
    flag="${arg:idx:1}"
    case "$flag" in
      r)
        RVIZ=false
        ;;
      s)
        REALSENSE=false
        ;;
      d)
        DRY_RUN=true
        ;;
      h)
        usage
        exit 0
        ;;
      *)
        printf 'Unknown short option: -%s\n' "$flag" >&2
        usage >&2
        exit 1
        ;;
    esac
  done
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --rviz)
        RVIZ=true
        ;;
      --no-rviz)
        RVIZ=false
        ;;
      --realsense)
        REALSENSE=true
        ;;
      --no-realsense)
        REALSENSE=false
        ;;
      --dry-run)
        DRY_RUN=true
        ;;
      -[!-]*)
        parse_short_flags "$1"
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        printf 'Unknown argument: %s\n' "$1" >&2
        usage >&2
        exit 1
        ;;
    esac
    shift
  done
}

require_command() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    printf 'Missing required command: %s\n' "$cmd" >&2
    exit 1
  fi
}

require_path() {
  local path="$1"
  if [[ ! -e "$path" ]]; then
    printf 'Missing required path: %s\n' "$path" >&2
    exit 1
  fi
}

set_default_commands() {
  if [[ -z "$TF_COMMAND" ]]; then
    TF_COMMAND="source ${ROS_SETUP} && exec ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom"
  fi

  if [[ -z "$SLAM_COMMAND" ]]; then
    SLAM_COMMAND="cd ${SLAM_WORKSPACE} && source install/setup.bash && exec ros2 run lightning run_slam_online --config ./config/default_livox.yaml"
  fi

  if [[ -z "$AUTO_COMMAND" ]]; then
    AUTO_COMMAND="cd ${AUTO_WORKSPACE} && source install/setup.bash && exec ros2 launch g1_nav g1_auto_explore.launch.py use_rviz:=${RVIZ} enable_realsense_scan_bridge:=${REALSENSE}"
  fi

  if [[ -z "$STOP_COMMAND" ]]; then
    STOP_COMMAND=$(cat <<EOF
cd ${STOP_WORKSPACE} && python3 - <<'PY'
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

ChannelFactoryInitialize(0, "${UNITREE_CHANNEL_NAME}")
client = LocoClient()
client.SetTimeout(1.0)
client.Init()
for _ in range(10):
    client.Move(0.0, 0.0, 0.0)
    time.sleep(0.05)
print("sent_stop")
PY
EOF
)
  fi
}

enable_all_cpus() {
  local sudo_script
  read -r -d '' sudo_script <<'EOF' || true
for cpu in 4 5 6 7; do
  path="/sys/devices/system/cpu/cpu${cpu}/online"
  if [ -e "$path" ]; then
    echo 1 > "$path"
    echo "enabled ${path}"
  else
    echo "skip missing ${path}" >&2
  fi
done
EOF

  if [[ "$ENABLE_CPU_ONLINE" != true ]]; then
    log 'Skipping CPU online step because ENABLE_CPU_ONLINE is not true'
    return
  fi

  log 'Enabling CPU cores cpu4-cpu7'
  if [[ "$DRY_RUN" == true ]]; then
    printf "printf '***' | sudo -S bash -c '<enable cpu4..cpu7>'\n"
    return
  fi

  printf '%s\n' "$SUDO_PASSWORD" | sudo -S -p '' bash -c "$sudo_script"
}

record_child() {
  local name="$1"
  local pid="$2"
  local pgid="$3"
  CHILD_NAMES+=("$name")
  CHILD_PIDS+=("$pid")
  CHILD_PGIDS+=("$pgid")
}

is_pid_alive() {
  local pid="$1"
  kill -0 "$pid" >/dev/null 2>&1
}

is_pgid_alive() {
  local pgid="$1"
  pgrep -g "$pgid" >/dev/null 2>&1
}

pid_name() {
  local pid="$1"
  local idx
  for idx in "${!CHILD_PIDS[@]}"; do
    if [[ "${CHILD_PIDS[$idx]}" == "$pid" ]]; then
      printf '%s' "${CHILD_NAMES[$idx]}"
      return 0
    fi
  done
  printf 'pid-%s' "$pid"
}

start_managed_process() {
  local name="$1"
  local command="$2"
  local pid
  local pgid

  if [[ "$DRY_RUN" == true ]]; then
    printf '[dry-run] %s\n' "$command"
    return
  fi

  log "Starting ${name}"
  setsid bash -lc "$command" &
  pid=$!
  pgid="$(ps -o pgid= -p "$pid" | tr -d '[:space:]')"
  if [[ -z "$pgid" ]]; then
    pgid="$pid"
  fi
  record_child "$name" "$pid" "$pgid"
  log "${name} started with pid ${pid}, pgid ${pgid}"
}

terminate_process_groups() {
  local signal="$1"
  local idx pgid name

  for idx in "${!CHILD_PGIDS[@]}"; do
    pgid="${CHILD_PGIDS[$idx]}"
    name="${CHILD_NAMES[$idx]}"
    if is_pgid_alive "$pgid"; then
      log "Sending SIG${signal} to ${name} (pgid ${pgid})"
      kill "-${signal}" -- "-${pgid}" >/dev/null 2>&1 || true
    fi
  done
}

wait_for_shutdown() {
  local deadline=$((SECONDS + SHUTDOWN_GRACE_SECONDS))
  local pgid any_alive

  while (( SECONDS < deadline )); do
    any_alive=false
    for pgid in "${CHILD_PGIDS[@]}"; do
      if is_pgid_alive "$pgid"; then
        any_alive=true
        break
      fi
    done
    if [[ "$any_alive" == false ]]; then
      return 0
    fi
    sleep 0.2
  done

  return 1
}

reap_children() {
  local pid
  for pid in "${CHILD_PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
}

send_robot_stop() {
  if [[ "$DRY_RUN" == true ]]; then
    printf '[dry-run] %s\n' "$STOP_COMMAND"
    return
  fi

  log "Sending explicit zero-velocity stop on ${UNITREE_CHANNEL_NAME}"
  if ! bash -lc "$STOP_COMMAND"; then
    log 'Zero-velocity stop command failed'
  fi
}

sweep_known_processes() {
  local signal="$1"

  if [[ -z "$FALLBACK_KILL_REGEX" ]]; then
    return
  fi

  if [[ "$DRY_RUN" == true ]]; then
    printf '[dry-run] pkill -%s -f %q || true\n' "$signal" "$FALLBACK_KILL_REGEX"
    return
  fi

  log "Running fallback sweep with SIG${signal}"
  pkill "-${signal}" -f "$FALLBACK_KILL_REGEX" >/dev/null 2>&1 || true
}

cleanup() {
  if [[ "$CLEANUP_DONE" == true ]]; then
    return
  fi
  CLEANUP_DONE=true

  log 'Stopping managed processes'
  send_robot_stop
  if ((${#CHILD_PIDS[@]} > 0)); then
    terminate_process_groups TERM
    if ! wait_for_shutdown; then
      log "Some processes are still alive after ${SHUTDOWN_GRACE_SECONDS}s"
      terminate_process_groups KILL
    fi
    reap_children
  fi
  sweep_known_processes TERM
  sleep 1
  sweep_known_processes KILL
  send_robot_stop
}

handle_signal() {
  local signal_name="$1"
  log "Received ${signal_name}, shutting down all managed processes"
  cleanup
  exit 130
}

monitor_children() {
  local exited_pid=0
  local exit_code=0
  local name

  if ((${#CHILD_PIDS[@]} == 0)); then
    return 0
  fi

  set +e
  wait -n -p exited_pid "${CHILD_PIDS[@]}"
  exit_code=$?
  set -e

  name="$(pid_name "$exited_pid")"
  log "${name} exited with status ${exit_code}"
  return "$exit_code"
}

main() {
  parse_args "$@"
  export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_DEFAULT"
  set_default_commands

  require_command bash
  require_command sudo
  require_command setsid
  require_command python3
  require_path "$ROS_SETUP"
  require_path "${SLAM_WORKSPACE}/install/setup.bash"
  require_path "${SLAM_WORKSPACE}/config/default_livox.yaml"
  require_path "${AUTO_WORKSPACE}/install/setup.bash"
  require_path "$STOP_WORKSPACE"

  trap 'handle_signal INT' INT
  trap 'handle_signal TERM' TERM
  trap cleanup EXIT

  enable_all_cpus

  log "Using ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  log 'Starting all processes in this shell'
  start_managed_process 'map->odom tf' "$TF_COMMAND"
  sleep 1
  start_managed_process 'slam2' "$SLAM_COMMAND"

  if [[ "$DRY_RUN" == true ]]; then
    log "Skipping ${SLAM_START_DELAY}s wait in dry-run mode"
  else
    log "Waiting ${SLAM_START_DELAY}s before starting auto"
    sleep "$SLAM_START_DELAY"
  fi

  start_managed_process 'auto' "$AUTO_COMMAND"

  if [[ "$DRY_RUN" == true ]]; then
    return 0
  fi

  log 'All processes are running in the background of this shell'
  log 'Press Ctrl+C here to stop tf, slam2, and auto together'

  if ! monitor_children; then
    log 'A managed process exited unexpectedly, stopping the rest'
    return 1
  fi
}

main "$@"
