#!/usr/bin/env bash

set -euo pipefail

SCRIPT_NAME="$(basename "$0")"
RVIZ=false
REALSENSE=true
EXPLORATION=true
DRY_RUN=false
SERVER_URI="ws://172.16.21.205:8100/ws/navigation/executor"
SUDO_PASSWORD="${SUDO_PASSWORD:-123}"
SLAM_START_DELAY="${SLAM_START_DELAY:-5}"
SHUTDOWN_GRACE_SECONDS="${SHUTDOWN_GRACE_SECONDS:-5}"
ENABLE_CPU_ONLINE="${ENABLE_CPU_ONLINE:-true}"
ROS_DOMAIN_ID_DEFAULT="${ROS_DOMAIN_ID_DEFAULT:-30}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
SLAM_WORKSPACE="${SLAM_WORKSPACE:-/home/unitree/lightning-lm-livox}"
AUTO_WORKSPACE="${AUTO_WORKSPACE:-/home/unitree/ros2_ws}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
EXECUTOR_POI_STORE_FILE="${EXECUTOR_POI_STORE_FILE:-}"
DEFAULT_EXECUTOR_POI_STORE_FILE="${HOME}/.ros/g1_nav/poi_store.yaml"
TF_COMMAND="${TF_COMMAND:-}"
SLAM_COMMAND="${SLAM_COMMAND:-}"
AUTO_COMMAND="${AUTO_COMMAND:-}"
EXECUTOR_COMMAND="${EXECUTOR_COMMAND:-}"

declare -a CHILD_PIDS=()
declare -a CHILD_NAMES=()
declare -a CHILD_PGIDS=()
declare -a CLEANUP_PIDS=()
declare -a CLEANUP_PGIDS=()
CLEANUP_DONE=false

usage() {
  cat <<'EOF'
Usage:
  start_navigation_headless.sh [--rviz] [--no-realsense] [--no-exploration] [--server-uri URI] [--dry-run]
  start_navigation_headless.sh [-r] [-s] [-e] [-u URI] [-d]

Behavior:
  Start TF, slam2, g1_auto_explore, and navigation_executor in one shell as managed background processes.
  Press Ctrl+C in this shell to stop every managed process cleanly.

Options:
  --rviz               Enable RViz. Default: disabled.
  -r                   Enable RViz.
  --realsense          Enable Realsense depth-to-scan bridge. Default: enabled.
  --no-realsense       Disable Realsense depth-to-scan bridge.
  -s                   Disable Realsense depth-to-scan bridge.
  --exploration        Enable frontier exploration. Default: enabled.
  --no-exploration     Disable frontier exploration.
  -e                   Disable frontier exploration.
  --server-uri URI     WebSocket server URI for navigation_executor.py.
  -u URI               WebSocket server URI for navigation_executor.py.
  --dry-run            Print commands without starting them.
  -d                   Print commands without starting them.
  -h, --help           Show this help.

Environment variables:
  ROS_DOMAIN_ID_DEFAULT     Default ROS 2 domain ID when ROS_DOMAIN_ID is unset. Default: 30
  SUDO_PASSWORD            Password used for sudo. Default: 123
  SLAM_START_DELAY         Seconds to wait after starting slam2 before auto/executor. Default: 5
  SHUTDOWN_GRACE_SECONDS   Seconds to wait before force kill. Default: 5
  ENABLE_CPU_ONLINE        true/false. Whether to online cpu4-cpu7. Default: true
  PYTHON_BIN               Python interpreter used for executor preflight checks. Default: python3
  ROS_SETUP                ROS 2 setup.bash path. Default: /opt/ros/humble/setup.bash
  SLAM_WORKSPACE           Lightning SLAM workspace. Default: /home/unitree/lightning-lm-livox
  AUTO_WORKSPACE           ROS 2 workspace containing g1_nav. Default: /home/unitree/ros2_ws
  EXECUTOR_POI_STORE_FILE  Optional path passed to navigation_executor.py --poi-store-file
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
        RVIZ=true
        ;;
      s)
        REALSENSE=false
        ;;
      e)
        EXPLORATION=false
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
      --realsense)
        REALSENSE=true
        ;;
      --no-realsense)
        REALSENSE=false
        ;;
      --exploration)
        EXPLORATION=true
        ;;
      --no-exploration)
        EXPLORATION=false
        ;;
      --server-uri)
        SERVER_URI="${2:-}"
        shift 2
        continue
        ;;
      --server-uri=*)
        SERVER_URI="${1#*=}"
        ;;
      -u)
        SERVER_URI="${2:-}"
        shift 2
        continue
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

require_python_module() {
  local module_name="$1"
  local package_hint="$2"

  if ! "$PYTHON_BIN" -c "import ${module_name}" >/dev/null 2>&1; then
    printf 'Missing required Python module: %s\n' "$module_name" >&2
    printf 'Install it with: sudo apt install %s\n' "$package_hint" >&2
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
    AUTO_COMMAND="cd ${AUTO_WORKSPACE} && source install/setup.bash && exec ros2 launch g1_nav g1_auto_explore.launch.py use_rviz:=${RVIZ} enable_realsense_scan_bridge:=${REALSENSE} enable_exploration:=${EXPLORATION}"
  fi

  if [[ -z "$EXECUTOR_COMMAND" ]]; then
    EXECUTOR_COMMAND="cd ${AUTO_WORKSPACE} && source install/setup.bash && exec ros2 run g1_nav navigation_executor.py --server-uri ${SERVER_URI}"
    if [[ -n "${EXECUTOR_POI_STORE_FILE}" ]]; then
      EXECUTOR_COMMAND="${EXECUTOR_COMMAND} --poi-store-file ${EXECUTOR_POI_STORE_FILE}"
    fi
  fi
}

executor_poi_store_file() {
  if [[ -n "${EXECUTOR_POI_STORE_FILE}" ]]; then
    printf '%s\n' "${EXECUTOR_POI_STORE_FILE}"
    return 0
  fi

  printf '%s\n' "${DEFAULT_EXECUTOR_POI_STORE_FILE}"
}

reset_poi_store_file() {
  local poi_store_file

  poi_store_file="$(executor_poi_store_file)"
  mkdir -p "$(dirname "${poi_store_file}")"

  if [[ "$DRY_RUN" == true ]]; then
    log "Would clear POI store: ${poi_store_file}"
    return 0
  fi

  : > "${poi_store_file}"
  log "Cleared POI store: ${poi_store_file}"
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

append_unique() {
  local array_name="$1"
  local value="$2"
  local existing
  local -n target_array="$array_name"

  for existing in "${target_array[@]:-}"; do
    if [[ "$existing" == "$value" ]]; then
      return 0
    fi
  done

  target_array+=("$value")
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

collect_descendant_pids() {
  local pid="$1"
  local child

  while read -r child; do
    [[ -n "$child" ]] || continue
    printf '%s\n' "$child"
    collect_descendant_pids "$child"
  done < <(pgrep -P "$pid" 2>/dev/null || true)
}

snapshot_cleanup_targets() {
  local root_pid child_pid pid pgid

  CLEANUP_PIDS=()
  CLEANUP_PGIDS=()

  for root_pid in "${CHILD_PIDS[@]}"; do
    append_unique CLEANUP_PIDS "$root_pid"
    if ! is_pid_alive "$root_pid"; then
      continue
    fi

    while read -r child_pid; do
      [[ -n "$child_pid" ]] || continue
      append_unique CLEANUP_PIDS "$child_pid"
    done < <(collect_descendant_pids "$root_pid")
  done

  for pid in "${CLEANUP_PIDS[@]}"; do
    if ! is_pid_alive "$pid"; then
      continue
    fi

    pgid="$(ps -o pgid= -p "$pid" | tr -d '[:space:]')"
    if [[ -n "$pgid" ]]; then
      append_unique CLEANUP_PGIDS "$pgid"
    fi
  done
}

signal_cleanup_targets() {
  local signal="$1"
  local pgid pid

  for pgid in "${CLEANUP_PGIDS[@]}"; do
    if is_pgid_alive "$pgid"; then
      log "Sending SIG${signal} to process group ${pgid}"
      kill "-${signal}" -- "-${pgid}" >/dev/null 2>&1 || true
    fi
  done

  for pid in "${CLEANUP_PIDS[@]}"; do
    if is_pid_alive "$pid"; then
      log "Sending SIG${signal} to pid ${pid}"
      kill "-${signal}" "$pid" >/dev/null 2>&1 || true
    fi
  done
}

wait_for_shutdown() {
  local deadline=$((SECONDS + SHUTDOWN_GRACE_SECONDS))
  local pid any_alive

  while (( SECONDS < deadline )); do
    any_alive=false
    for pid in "${CLEANUP_PIDS[@]}"; do
      if is_pid_alive "$pid"; then
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

cleanup() {
  if [[ "$CLEANUP_DONE" == true ]]; then
    return
  fi
  CLEANUP_DONE=true

  if ((${#CHILD_PIDS[@]} == 0)); then
    return
  fi

  log 'Stopping managed processes'
  snapshot_cleanup_targets
  signal_cleanup_targets TERM
  if ! wait_for_shutdown; then
    log "Some processes are still alive after ${SHUTDOWN_GRACE_SECONDS}s"
    signal_cleanup_targets KILL
  fi
  reap_children
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
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$ROS_DOMAIN_ID_DEFAULT}"
  set_default_commands

  require_command bash
  require_command pgrep
  require_command ps
  require_command setsid
  require_command sudo
  require_command "$PYTHON_BIN"
  require_path "$ROS_SETUP"
  require_path "${SLAM_WORKSPACE}/install/setup.bash"
  require_path "${SLAM_WORKSPACE}/config/default_livox.yaml"
  require_path "${AUTO_WORKSPACE}/install/setup.bash"
  require_python_module yaml python3-yaml
  require_python_module websockets python3-websockets

  trap 'handle_signal INT' INT
  trap 'handle_signal TERM' TERM
  trap cleanup EXIT

  enable_all_cpus
  reset_poi_store_file

  log "Using ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  log "RViz enabled: ${RVIZ}"
  log "Realsense enabled: ${REALSENSE}"
  log "Frontier exploration enabled: ${EXPLORATION}"
  log "Executor target URI: ${SERVER_URI}"
  log 'Starting all processes in this shell'
  start_managed_process 'map->odom tf' "$TF_COMMAND"
  sleep 1
  start_managed_process 'slam2' "$SLAM_COMMAND"

  if [[ "$DRY_RUN" == true ]]; then
    log "Skipping ${SLAM_START_DELAY}s wait in dry-run mode"
  else
    log "Waiting ${SLAM_START_DELAY}s before starting auto and navigation executor"
    sleep "$SLAM_START_DELAY"
  fi

  start_managed_process 'auto' "$AUTO_COMMAND"
  start_managed_process 'navigation executor' "$EXECUTOR_COMMAND"

  if [[ "$DRY_RUN" == true ]]; then
    return 0
  fi

  log 'All processes are running in the background of this shell'
  log 'Press Ctrl+C here to stop tf, slam2, auto, and navigation executor together'

  if ! monitor_children; then
    log 'A managed process exited unexpectedly, stopping the rest'
    return 1
  fi
}

main "$@"
