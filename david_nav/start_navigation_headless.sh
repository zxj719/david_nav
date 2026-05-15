#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

resolve_workspace_dir() {
  local candidate="${SCRIPT_DIR}"

  while [[ "${candidate}" != "/" ]]; do
    if [[ -d "${candidate}/src/david_nav" ]]; then
      echo "${candidate}"
      return 0
    fi
    candidate="$(dirname "${candidate}")"
  done

  echo "/home/unitree/ros2_ws"
}

WS_DIR="${DAVID_WS_DIR:-$(resolve_workspace_dir)}"
PACKAGE_SRC_DIR="${WS_DIR}/src/david_nav"
ROS_SETUP_FILE="${DAVID_ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}"
INSTALL_SETUP_FILE="${DAVID_INSTALL_SETUP_FILE:-${WS_DIR}/install/setup.bash}"
CONFIG_HELPER_BIN="${DAVID_CONFIG_HELPER_BIN:-${SCRIPT_DIR}/scripts/navigation_headless_config.py}"
EXECUTOR_BIN="${DAVID_EXECUTOR_BIN:-}"
EXECUTOR_LOCK_FILE="/tmp/david_navigation_executor.lock"
HEADLESS_CONFIG_FILE="${DAVID_HEADLESS_CONFIG_FILE:-${PACKAGE_SRC_DIR}/config/navigation_headless.yaml}"
ROS2_BIN="${DAVID_ROS2_BIN:-ros2}"
PYTHON_BIN="${DAVID_PYTHON_BIN:-python3}"
HEADLESS_PROFILE="${DAVID_HEADLESS_PROFILE:-}"
SUDO_PASSWORD="${SUDO_PASSWORD:-123}"
EXECUTOR_POI_STORE_FILE="${EXECUTOR_POI_STORE_FILE:-}"
DEFAULT_EXECUTOR_POI_STORE_FILE="${HOME}/.ros/david_nav/poi_store.yaml"
SEMANTIC_MAP_DIR="${SEMANTIC_MAP_DIR:-/home/unitree/semantic_map}"
NAMED_POSES_FILE="${NAMED_POSES_FILE:-${SEMANTIC_MAP_DIR}/named_poses.json}"
NAV2_WAIT_TIMEOUT_SEC=120
STOP_TERM_TIMEOUT_SEC=5
STOP_KILL_TIMEOUT_SEC=2
ROS_DOMAIN_ID_DEFAULT="${ROS_DOMAIN_ID_DEFAULT:-30}"

safe_source() {
  local source_file="$1"
  if [[ ! -f "${source_file}" ]]; then
    echo "[start_nav_headless] source file not found: ${source_file}" >&2
    exit 1
  fi

  set +u
  # shellcheck disable=SC1090
  source "${source_file}"
  set -u
}

wait_for_pid_exit() {
  local pid="$1"
  local timeout_sec="${2:-5}"
  local deadline=$((SECONDS + timeout_sec))

  while kill -0 "${pid}" 2>/dev/null; do
    if (( SECONDS >= deadline )); then
      return 1
    fi
    sleep 0.2
  done

  return 0
}

wait_for_process_group_exit() {
  local process_group_id="$1"
  local timeout_sec="${2:-5}"
  local deadline=$((SECONDS + timeout_sec))

  while kill -0 -- "-${process_group_id}" 2>/dev/null; do
    if (( SECONDS >= deadline )); then
      return 1
    fi
    sleep 0.2
  done

  return 0
}

get_process_group_id() {
  local pid="$1"
  ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]'
}

stop_pid_if_running() {
  local pid="$1"
  local label="${2:-process}"
  local signal_name="${3:-TERM}"
  local term_timeout_sec="${4:-$STOP_TERM_TIMEOUT_SEC}"
  local kill_timeout_sec="${5:-$STOP_KILL_TIMEOUT_SEC}"
  local use_process_group="${6:-0}"
  local process_group_id="${pid}"

  if [[ "${use_process_group}" == "1" ]]; then
    if [[ -z "${process_group_id}" ]] || ! kill -0 -- "-${process_group_id}" 2>/dev/null; then
      return 0
    fi
  else
    if [[ -z "${pid}" ]] || ! kill -0 "${pid}" 2>/dev/null; then
      return 0
    fi
  fi

  if [[ "${use_process_group}" == "1" ]]; then
    echo "[start_nav_headless] stopping ${label} (pgid=${process_group_id}, signal=${signal_name})"
    kill "-${signal_name}" -- "-${process_group_id}" 2>/dev/null || kill "-${signal_name}" "${pid}" 2>/dev/null || true
    if ! wait_for_process_group_exit "${process_group_id}" "${term_timeout_sec}"; then
      echo "[start_nav_headless] ${label} did not exit after SIG${signal_name}, sending SIGKILL (pgid=${process_group_id})" >&2
      kill -9 -- "-${process_group_id}" 2>/dev/null || kill -9 "${pid}" 2>/dev/null || true
      wait_for_process_group_exit "${process_group_id}" "${kill_timeout_sec}" || true
    fi
  else
    echo "[start_nav_headless] stopping ${label} (pid=${pid}, signal=${signal_name})"
    kill "-${signal_name}" "${pid}" 2>/dev/null || true
    if ! wait_for_pid_exit "${pid}" "${term_timeout_sec}"; then
      echo "[start_nav_headless] ${label} did not exit after SIG${signal_name}, sending SIGKILL (pid=${pid})" >&2
      kill -9 "${pid}" 2>/dev/null || true
      wait_for_pid_exit "${pid}" "${kill_timeout_sec}" || true
    fi
  fi
}

cleanup_executor_lock() {
  local lock_pid=""

  if [[ ! -f "${EXECUTOR_LOCK_FILE}" ]]; then
    return 0
  fi

  lock_pid="$(tr -d '[:space:]' < "${EXECUTOR_LOCK_FILE}" 2>/dev/null || true)"
  if [[ -z "${lock_pid}" ]] || ! kill -0 "${lock_pid}" 2>/dev/null; then
    rm -f "${EXECUTOR_LOCK_FILE}"
  fi
}

is_zombie() {
  local pid="$1"
  [[ -f "/proc/${pid}/status" ]] && grep -q '^State:.*Z' "/proc/${pid}/status" 2>/dev/null
}

stop_stale_executor() {
  local pid="" pgid=""

  cleanup_executor_lock

  while read -r pid; do
    [[ -z "${pid}" ]] && continue
    is_zombie "${pid}" && continue
    pgid="$(get_process_group_id "${pid}")"
    if [[ -n "${pgid}" ]]; then
      stop_pid_if_running "${pgid}" "stale navigation_executor" TERM "${STOP_TERM_TIMEOUT_SEC}" "${STOP_KILL_TIMEOUT_SEC}" 1
    else
      stop_pid_if_running "${pid}" "stale navigation_executor"
    fi
  done < <(pgrep -f "navigation_executor.py" 2>/dev/null || true)

  if [[ -f "${EXECUTOR_LOCK_FILE}" ]]; then
    pid="$(tr -d '[:space:]' < "${EXECUTOR_LOCK_FILE}" 2>/dev/null || true)"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null && ! is_zombie "${pid}"; then
      pgid="$(get_process_group_id "${pid}")"
      if [[ -n "${pgid}" ]]; then
        stop_pid_if_running "${pgid}" "locked navigation_executor" TERM "${STOP_TERM_TIMEOUT_SEC}" "${STOP_KILL_TIMEOUT_SEC}" 1
      else
        stop_pid_if_running "${pid}" "locked navigation_executor"
      fi
    fi
  fi

  cleanup_executor_lock
}

pin_jetson_clocks() {
  if command -v nvpmodel >/dev/null 2>&1; then
    echo '[start_nav_headless] [cold-start-fix] Setting nvpmodel to MAXN (mode 0)...'
    printf '%s\n' "$SUDO_PASSWORD" | sudo -S -p '' nvpmodel -m 0 || \
      echo '[start_nav_headless] [cold-start-fix] WARN: nvpmodel failed (need sudo NOPASSWD?)'
  fi

  if command -v jetson_clocks >/dev/null 2>&1; then
    echo '[start_nav_headless] [cold-start-fix] Pinning CPU/GPU/EMC clocks to max...'
    printf '%s\n' "$SUDO_PASSWORD" | sudo -S -p '' jetson_clocks || \
      echo '[start_nav_headless] [cold-start-fix] WARN: jetson_clocks failed (need sudo NOPASSWD?)'
  fi
}

executor_poi_store_file() {
  if [[ -n "${EXECUTOR_POI_STORE_FILE}" ]]; then
    printf '%s\n' "${EXECUTOR_POI_STORE_FILE}"
    return 0
  fi

  printf '%s\n' "${DEFAULT_EXECUTOR_POI_STORE_FILE}"
}

load_named_poses_into_poi_store() {
  local poi_store_file
  poi_store_file="$(executor_poi_store_file)"
  mkdir -p "$(dirname "${poi_store_file}")"

  if [[ ! -f "${NAMED_POSES_FILE}" ]]; then
    echo "[start_nav_headless] named_poses.json not found: ${NAMED_POSES_FILE}, starting with empty POI store" >&2
    : > "${poi_store_file}"
    return
  fi

  "${PYTHON_BIN}" -c "
import json, yaml, sys
with open(sys.argv[1]) as f:
    poses = json.load(f)
pois = []
for p in poses:
    pose = {
        'frame_id': p.get('frame_id', 'map'),
        'x': float(p['position']['x']),
        'y': float(p['position']['y']),
        'z': float(p['position']['z']),
        'yaw': float(p['yaw']),
        'stamp_sec': 0.0,
    }
    pois.append({
        'id': p['id'],
        'name': p.get('name', p['id']),
        'map_pose': pose,
        'odom_pose': dict(pose, frame_id='odom'),
        'slam_session_id': 'named_poses',
    })
with open(sys.argv[2], 'w') as f:
    yaml.safe_dump({'pois': pois}, f, allow_unicode=True, sort_keys=False)
print(f'Loaded {len(pois)} POIs from {sys.argv[1]}')
" "${NAMED_POSES_FILE}" "${poi_store_file}"

  echo "[start_nav_headless] POI store: ${poi_store_file}"
}

print_help() {
  cat <<'EOF'
Usage: start_navigation_headless.sh [options]

Starts Nav2 bringup + navigation executor in headless mode.

Lightning SLAM is expected to be running independently. This script:
  1. pins Jetson clocks
  2. launches 2d_david_nav2_bringup.launch.py (Nav2, no exploration)
  3. waits for bt_navigator readiness
  4. starts navigation_executor.py

Options:
  --config PATH              Unified headless config YAML
  --profile NAME             Headless config profile name
  --no-rviz                  Disable RViz in Nav2 bringup (default: enabled)
  --no-realsense             Disable RealSense scan bridge
  --realsense                Enable RealSense scan bridge
  -h, --help                 Show this help

Environment overrides:
  DAVID_HEADLESS_CONFIG_FILE
  DAVID_HEADLESS_PROFILE
  SUDO_PASSWORD
  ROS_DOMAIN_ID_DEFAULT
EOF
}

USE_RVIZ=true
USE_REALSENSE=true

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      print_help
      exit 0
      ;;
    --config)
      HEADLESS_CONFIG_FILE="${2:-}"
      shift 2
      ;;
    --config=*)
      HEADLESS_CONFIG_FILE="${1#*=}"
      shift
      ;;
    --profile)
      HEADLESS_PROFILE="${2:-}"
      shift 2
      ;;
    --profile=*)
      HEADLESS_PROFILE="${1#*=}"
      shift
      ;;
    --no-rviz)
      USE_RVIZ=false
      shift
      ;;
    --rviz)
      USE_RVIZ=true
      shift
      ;;
    --realsense)
      USE_REALSENSE=true
      shift
      ;;
    --no-realsense)
      USE_REALSENSE=false
      shift
      ;;
    *)
      if [[ "$1" != -* && -z "${HEADLESS_PROFILE}" ]]; then
        HEADLESS_PROFILE="$1"
        shift
      else
        echo "[start_nav_headless] unknown option: $1" >&2
        print_help >&2
        exit 1
      fi
      ;;
  esac
done

CONFIG_RESOLVE_CMD=(
  "${PYTHON_BIN}" "${CONFIG_HELPER_BIN}"
  --config "${HEADLESS_CONFIG_FILE}"
  --format shell
)
if [[ -n "${HEADLESS_PROFILE}" ]]; then
  CONFIG_RESOLVE_CMD+=(--profile "${HEADLESS_PROFILE}")
fi

eval "$("${CONFIG_RESOLVE_CMD[@]}")"

SERVER_URI="${HEADLESS_SERVER_URI}"
SHOW_EXECUTOR_LOGS="${HEADLESS_SHOW_LOGS}"
NAV2_WAIT_TIMEOUT_SEC="${HEADLESS_NAV2_WAIT_TIMEOUT_SEC}"
STOP_TERM_TIMEOUT_SEC="${HEADLESS_TERM_TIMEOUT_SEC}"
STOP_KILL_TIMEOUT_SEC="${HEADLESS_KILL_TIMEOUT_SEC}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$ROS_DOMAIN_ID_DEFAULT}"

if [[ -z "${EXECUTOR_BIN}" ]]; then
  EXECUTOR_BIN="${ROS2_BIN} run david_nav navigation_executor.py"
fi

for required_file in \
  "${HEADLESS_CONFIG_PATH}" \
  "${CONFIG_HELPER_BIN}" \
  "${ROS_SETUP_FILE}" \
  "${INSTALL_SETUP_FILE}"; do
  if [[ ! -e "${required_file}" ]]; then
    echo "[start_nav_headless] missing required file: ${required_file}" >&2
    exit 1
  fi
done

safe_source "${ROS_SETUP_FILE}"
safe_source "${INSTALL_SETUP_FILE}"

LOG_DIR="${WS_DIR}/log/start_navigation_headless_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${LOG_DIR}"

BRINGUP_PID=""
BRINGUP_USE_PROCESS_GROUP="0"

cleanup() {
  local exit_code=$?
  trap - EXIT HUP INT QUIT TERM

  cleanup_executor_lock

  stop_pid_if_running "${BRINGUP_PID}" "Nav2 bringup" INT "${STOP_TERM_TIMEOUT_SEC}" "${STOP_KILL_TIMEOUT_SEC}" "${BRINGUP_USE_PROCESS_GROUP}"

  exit "${exit_code}"
}

trap cleanup EXIT HUP INT QUIT TERM

echo "[start_nav_headless] profile: ${HEADLESS_PROFILE}"
echo "[start_nav_headless] server uri: ${SERVER_URI}"
echo "[start_nav_headless] rviz: ${USE_RVIZ}"
echo "[start_nav_headless] realsense: ${USE_REALSENSE}"
echo "[start_nav_headless] show executor logs: ${SHOW_EXECUTOR_LOGS}"
echo "[start_nav_headless] ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "[start_nav_headless] logs: ${LOG_DIR}"

stop_stale_executor

pin_jetson_clocks
load_named_poses_into_poi_store

# --- Nav2 bringup (no exploration) ---
BRINGUP_CMD=(
  "${ROS2_BIN}" launch david_nav 2d_david_nav2_bringup.launch.py
  use_rviz:="${USE_RVIZ}"
  enable_realsense_scan_bridge:="${USE_REALSENSE}"
  enable_exploration:=false
)
if command -v setsid >/dev/null 2>&1; then
  setsid "${BRINGUP_CMD[@]}" > >(tee "${LOG_DIR}/nav2_bringup.log") 2>&1 &
  BRINGUP_USE_PROCESS_GROUP="1"
else
  "${BRINGUP_CMD[@]}" > >(tee "${LOG_DIR}/nav2_bringup.log") 2>&1 &
fi
BRINGUP_PID=$!
echo "[start_nav_headless] started Nav2 bringup (pid=${BRINGUP_PID})"

is_bringup_alive() {
  kill -0 "${BRINGUP_PID}" 2>/dev/null || return 1
  if [[ -f "/proc/${BRINGUP_PID}/status" ]]; then
    grep -q '^State:.*Z' "/proc/${BRINGUP_PID}/status" 2>/dev/null && return 1
  fi
  return 0
}

echo "[start_nav_headless] waiting for Nav2 bt_navigator node..."
deadline=$((SECONDS + NAV2_WAIT_TIMEOUT_SEC))
NAV2_READY=false
while (( SECONDS < deadline )); do
  if ! is_bringup_alive; then
    echo "[start_nav_headless] Nav2 bringup exited early, check ${LOG_DIR}/nav2_bringup.log" >&2
    exit 1
  fi

  if "${ROS2_BIN}" node list 2>/dev/null | grep -qE '(^|/)bt_navigator$'; then
    NAV2_READY=true
    break
  fi

  sleep 1
done

if [[ "${NAV2_READY}" != true ]]; then
  echo "[start_nav_headless] Nav2 did not become ready within ${NAV2_WAIT_TIMEOUT_SEC}s" >&2
  exit 1
fi
echo "[start_nav_headless] bt_navigator detected, waiting 10s for lifecycle activation..."
sleep 10
if ! is_bringup_alive; then
  echo "[start_nav_headless] Nav2 bringup died during activation, check ${LOG_DIR}/nav2_bringup.log" >&2
  exit 1
fi
echo "[start_nav_headless] Nav2 is ready."

# --- Navigation executor (foreground) ---
EXECUTOR_CMD=(
  ${EXECUTOR_BIN}
  --server-uri "${SERVER_URI}"
)

POI_FILE="$(executor_poi_store_file)"
if [[ -n "${POI_FILE}" ]]; then
  EXECUTOR_CMD+=(--poi-store-file "${POI_FILE}")
fi

echo "[start_nav_headless] starting navigation_executor."
echo "[start_nav_headless] Press Ctrl+C to stop everything."

"${EXECUTOR_CMD[@]}"
