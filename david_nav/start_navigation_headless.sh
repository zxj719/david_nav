#!/usr/bin/env bash

set -euo pipefail

SCRIPT_NAME="$(basename "$0")"
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
  echo "${HOME}/ros2_ws"
}

WS_DIR="${DAVID_WS_DIR:-$(resolve_workspace_dir)}"
PACKAGE_SRC_DIR="${WS_DIR}/src/david_nav"
ROS_SETUP="${DAVID_ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}"
INSTALL_SETUP="${DAVID_INSTALL_SETUP_FILE:-${WS_DIR}/install/setup.bash}"
CONFIG_HELPER_BIN="${DAVID_CONFIG_HELPER_BIN:-${SCRIPT_DIR}/scripts/navigation_headless_config.py}"
HEADLESS_CONFIG_FILE="${DAVID_HEADLESS_CONFIG_FILE:-${PACKAGE_SRC_DIR}/config/navigation_headless.yaml}"
ROS2_BIN="${DAVID_ROS2_BIN:-ros2}"
PYTHON_BIN="${DAVID_PYTHON_BIN:-python3}"
HEADLESS_PROFILE="${DAVID_HEADLESS_PROFILE:-}"
SUDO_PASSWORD="${SUDO_PASSWORD:-}"

EXECUTOR_LOCK_FILE="/tmp/david_navigation_executor.lock"
EXECUTOR_BIN="${DAVID_EXECUTOR_BIN:-}"

WHEELTEC_WS_DIR="${WHEELTEC_WS_DIR:-${HOME}/wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2}"
WHEELTEC_LAUNCH_PACKAGE="${WHEELTEC_LAUNCH_PACKAGE:-turn_on_wheeltec_robot}"
WHEELTEC_LAUNCH_FILE="${WHEELTEC_LAUNCH_FILE:-turn_on_wheeltec_robot.launch.py}"
CHASSIS_STARTUP_DELAY_SEC="${CHASSIS_STARTUP_DELAY_SEC:-3}"
START_CHASSIS=true

SEMANTIC_MAP_DIR="${SEMANTIC_MAP_DIR:-${HOME}/semantic_map}"
POSE_FILE="${POSE_FILE:-${SEMANTIC_MAP_DIR}/named_poses.json}"
BAG_ROOT="${BAG_ROOT:-${HOME}/rosbags}"
BAG_NAME="${BAG_NAME:-}"

# 2026-05-20: default changed 30 → 0 to match the rest of the robot stack.
# Lightning loc / wheeltec / lslidar / IMU all run in domain 0 by default.
# Whole-stack domain comes from the single source config/ros_env.conf (change
# the domain in ONE place). nav2 / lightning / chassis / arm SDK must all share
# it or nav2 + navigation_executor can't hear lightning's map→odom TF.
# Applied in main() via config/load_ros_env.sh; override per-run with
# `export ROS_DOMAIN_ID=… ` before invoking.

NAV2_WAIT_TIMEOUT_SEC="${NAV2_WAIT_TIMEOUT_SEC:-120}"
SHUTDOWN_GRACE_SECONDS="${SHUTDOWN_GRACE_SECONDS:-5}"
EXPECTED_NAV2_FRAME="${EXPECTED_NAV2_FRAME:-map}"
EXPECTED_DWB_TRANSFORM_TOLERANCE="${EXPECTED_DWB_TRANSFORM_TOLERANCE:-2.0}"

DRY_RUN=false
FULL_BAG=false
FORCE_NO_BAG=false
FORCE_RECORD_BAG=false
CHECK_RUNTIME_CONFIG=true

declare -a CHILD_PIDS=()
declare -a CHILD_NAMES=()
declare -a CHILD_PGIDS=()
declare -a CLEANUP_PIDS=()
declare -a CLEANUP_PGIDS=()
declare -a EXTRA_LAUNCH_ARGS=()
declare -a EXECUTOR_EXTRA_ARGS=()
CLEANUP_DONE=false
LAST_STARTED_PID=""
LAST_STARTED_PGID=""
BRINGUP_PID=""

# 2026-05-23: chassis liveness watcher + zero-cmd-vel-on-shutdown defenses.
# See docs/reports/2026-05-23-chassis-driver-death-mid-run.md. wheeltec_robot
# crashing mid-run leaves STM32 holding the last frame (e.g. v=0.5), so Ctrl+C
# can't stop the robot. We poll chassis pid and trigger normal cleanup on death;
# cleanup itself first publishes (0,0) to /cmd_vel so any healthy STM32 stops.
CHASSIS_PID=""
CHASSIS_LIVENESS_PID=""
CHASSIS_LIVENESS_INTERVAL_SEC="${CHASSIS_LIVENESS_INTERVAL_SEC:-5}"

# 2026-08-28: Nav2 bringup liveness watcher。此前只有 wait_for_nav_ready 在
# 启动阶段检查 BRINGUP_PID，启动成功之后没有任何东西盯着它。实测
# 20260828_040511 那轮：bringup.log 停在 04:23、chassis.log 停在 04:12，
# 而 rosbag 录制进程一直活到 04:25 之后，bag 涨到 5.9GB。
# 与底盘看护不同，nav2 死掉不尝试重启（重启后位姿/BT 状态都不可信），
# 直接走标准 cleanup 收摊。
BRINGUP_LIVENESS_PID=""
BRINGUP_LIVENESS_INTERVAL_SEC="${BRINGUP_LIVENESS_INTERVAL_SEC:-5}"
BRINGUP_EXECUTOR_KILL_GRACE_SEC="${BRINGUP_EXECUTOR_KILL_GRACE_SEC:-5}"
ZERO_CMD_VEL_ON_SHUTDOWN="${ZERO_CMD_VEL_ON_SHUTDOWN:-true}"
ZERO_CMD_VEL_DURATION_SEC="${ZERO_CMD_VEL_DURATION_SEC:-0.6}"

# Chassis auto-restart budget. When wheeltec_robot dies mid-run we try
# bringing it back up CHASSIS_MAX_RESTARTS times before triggering full
# shutdown. The watcher fires SIGUSR1 → handle_chassis_death, which calls
# restart_chassis_once. Budget is per-script-invocation; not reset on
# successful uptime (a chassis that crashes twice in one session is almost
# certainly a hardware problem worth stopping for).
CHASSIS_MAX_RESTARTS="${CHASSIS_MAX_RESTARTS:-1}"
CHASSIS_RESTART_COUNT=0
CHASSIS_LAUNCH_CMD_STR=""
CHASSIS_LOG_FILE=""

EXPLICIT_TOPICS=(
  /tf
  /tf_static
  /parameter_events
  /rosout
  /cmd_vel
  /cmd_vel_nav
  /cmd_vel_safe
  /cmd_control_command
  /scan
  /map
  /lightning/odometry
  /lightning/grid_map
  /odommodestate
  /sportmodestate
  /lf/odommodestate
  /lf/sportmodestate
  /planner_map
  /gridmap
  /collision_clouds
  /pre_collision_clouds
  /safe_clouds
  /warning_clouds
  /utlidar/cloud_livox_mid360
  /utlidar/imu_livox_mid360
)

LIGHT_BAG_TOPICS=(
  /tf
  /tf_static
  /rosout
  /cmd_vel
  /cmd_vel_nav
  /cmd_vel_safe
  /cmd_control_command
  /scan
  /map
  /lightning/odometry
  # /lightning/grid_map —— 2026-07-23 移出默认录制：~10Hz 全图，3.4h 实测吃掉
  # 107GB 并把 IO 压到 TF 频率从 92Hz 掉到 11Hz（定位反馈变慢=额外环路时延）。
  # 静态环境回放用 /global_costmap/costmap 足够；确需原图用 --full-bag。
  /local_costmap/costmap
  /local_costmap/costmap_raw
  /local_costmap/published_footprint
  # /global_costmap/costmap + /global_costmap/costmap_raw —— 2026-08-28 移出默认
  # 录制，与上面 /lightning/grid_map 同一个原因，当时只换了来源没降量级：
  # 全局 costmap 被 static_layer 对齐到 948x813@0.05m，单帧 770KB，两个话题
  # 3.1Hz 双发 = 4.8MB/s，占本清单全部字节量的 99.6%（实测 440s 导览窗口），
  # 22 分钟就写出 5.9GB。回放定位/轨迹问题只需 /tf + /plan + /cmd_vel*。
  # 另：nav2 的 Costmap2DPublisher 仅在 subscription_count>0 时才序列化发布，
  # 所以移出录制 = 生产环境里这两个话题彻底不产生开销。
  # 需要全局 costmap 时用 --full-bag。
  /global_costmap/costmap_updates
  /global_costmap/published_footprint
  /navigate_to_pose/_action/status
  /navigate_to_pose/_action/feedback
  /navigate_to_pose/_action/result
  /plan
)

REGEX='^/((local_costmap|global_costmap|frontier|explore|collision_monitor|navigate_to_pose|navigate_through_poses|plan|goal_pose|clicked_point|behavior_tree_log|map_live_ready)(/.*)?)$'

usage() {
  cat <<EOF
Usage: ${SCRIPT_NAME} [PROFILE | --profile NAME] [options]

Headless variant of navigate_semantic_pose_with_debug_bag.sh: same Nav2 bringup
(2d_david_nav2_bringup.launch.py), same JSON pose source, same debug rosbag,
same runtime config check; only the foreground process differs --- this one
runs navigation_executor.py which exposes a WebSocket service for remote
navigation calls instead of an interactive terminal selection.

nav WS v1.1: the executor is interruptible at any time --- an
abort_navigation frame from the agent cancels the in-flight nav2 goal from
ANY state (starting / navigating / retry-debounce), confirms with
on_progress status="paused" and returns to idle. Repeated / stale / idle
aborts are idempotent no-ops.

Lightning SLAM is expected to be running independently. This script does not
start SLAM, localization, or relocation.

Options:
  --config PATH                   Unified headless config YAML
  --profile NAME                  Profile name inside the YAML
  --pose-file PATH                Override named_poses.json path
  -o, --output NAME               Bag folder name. Default: david_nav_headless_YYYYmmdd_HHMMSS
  --no-bag                        Skip debug rosbag (overrides profile.record_bag)
  --bag                           Force debug rosbag on (overrides profile.record_bag)
  --full-bag                      Full debug bag with hidden topics + zstd
  --no-chassis                    Skip wheeltec chassis launch (for bench testing)
  --skip-runtime-config-check     Skip Nav2 global_frame / transform_tolerance check
  -d, --dry-run                   Print commands without starting processes
  -h, --help                      Show this help
  --                              Forward remaining args to navigation_executor.py

Environment overrides:
  DAVID_HEADLESS_CONFIG_FILE      Headless config YAML
  DAVID_HEADLESS_PROFILE          Profile name
  SUDO_PASSWORD                   Used to run nvpmodel/jetson_clocks via sudo -S
  ROS_DOMAIN_ID                   Override domain for this run (default from config/ros_env.conf)
  POSE_FILE                       named_poses.json path
  BAG_ROOT                        Output root for rosbag folders
  WHEELTEC_WS_DIR                 Path to wheeltec_robot_ros2 workspace
  WHEELTEC_LAUNCH_PACKAGE         Launch package (default: turn_on_wheeltec_robot)
  WHEELTEC_LAUNCH_FILE            Launch file (default: turn_on_wheeltec_robot.launch.py)
  CHASSIS_STARTUP_DELAY_SEC       Seconds to wait after chassis launch before Nav2 (default: 3)
EOF
}

log() {
  printf '[%s] %s\n' "$SCRIPT_NAME" "$*"
}

quote_command() {
  printf '%q ' "$@"
  printf '\n'
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

safe_source() {
  local source_file="$1"
  if [[ ! -f "${source_file}" ]]; then
    printf '[%s] source file not found: %s\n' "$SCRIPT_NAME" "$source_file" >&2
    exit 1
  fi
  set +u
  # shellcheck disable=SC1090
  source "$source_file"
  set -u
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

is_pid_alive() {
  local pid="$1"
  kill -0 "$pid" >/dev/null 2>&1
}

is_pgid_alive() {
  local pgid="$1"
  pgrep -g "$pgid" >/dev/null 2>&1
}

is_zombie() {
  local pid="$1"
  [[ -f "/proc/${pid}/status" ]] && grep -q '^State:.*Z' "/proc/${pid}/status" 2>/dev/null
}

record_child() {
  local name="$1"
  local pid="$2"
  local pgid="$3"
  CHILD_NAMES+=("$name")
  CHILD_PIDS+=("$pid")
  CHILD_PGIDS+=("$pgid")
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
  local root_pid child_pid pid pgid recorded_pgid

  CLEANUP_PIDS=()
  CLEANUP_PGIDS=()

  for recorded_pgid in "${CHILD_PGIDS[@]}"; do
    [[ -n "$recorded_pgid" ]] || continue
    append_unique CLEANUP_PGIDS "$recorded_pgid"
  done

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

stop_stale_executor() {
  local pid="" pgid=""

  cleanup_executor_lock

  while read -r pid; do
    [[ -z "${pid}" ]] && continue
    is_zombie "${pid}" && continue
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"
    log "Stopping stale navigation_executor (pid=${pid}, pgid=${pgid:-?})"
    if [[ -n "${pgid}" ]]; then
      kill -TERM -- "-${pgid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    else
      kill -TERM "${pid}" 2>/dev/null || true
    fi
  done < <(pgrep -f "navigation_executor.py" 2>/dev/null || true)

  cleanup_executor_lock
}

start_chassis_liveness_watch() {
  # Poll the wheeltec chassis pid every CHASSIS_LIVENESS_INTERVAL_SEC. If
  # it dies the watcher fires SIGUSR1 on the main script — handle_chassis_death
  # then decides whether to restart or give up. We deliberately do NOT send
  # SIGINT here because we want one restart attempt before bailing out.
  if [[ "$START_CHASSIS" != true ]] || [[ -z "$CHASSIS_PID" ]]; then
    return 0
  fi
  if [[ "$DRY_RUN" == true ]]; then
    return 0
  fi
  local script_pid="$$"
  local chassis_pid="$CHASSIS_PID"
  local interval="$CHASSIS_LIVENESS_INTERVAL_SEC"
  log "Starting chassis liveness watch (poll ${interval}s, chassis pid=${chassis_pid})"
  (
    while kill -0 "$chassis_pid" 2>/dev/null; do
      sleep "$interval"
    done
    printf '[%s] WARN: Wheeltec chassis (pid=%s) died. Notifying main.\n' \
      "$(date '+%Y-%m-%dT%H:%M:%S')" "$chassis_pid" >&2
    kill -USR1 "$script_pid" 2>/dev/null || true
  ) &
  CHASSIS_LIVENESS_PID=$!
  disown "$CHASSIS_LIVENESS_PID" 2>/dev/null || true
}

restart_chassis_once() {
  # Bring the wheeltec_robot launch back up once. Caller is the SIGUSR1
  # trap so we're already in the main shell and can mutate CHILD_* arrays
  # via start_managed_process. Returns 0 on success, 1 if budget exhausted
  # or the new process exits during CHASSIS_STARTUP_DELAY_SEC.
  if (( CHASSIS_RESTART_COUNT >= CHASSIS_MAX_RESTARTS )); then
    log "Chassis restart budget exhausted (${CHASSIS_RESTART_COUNT}/${CHASSIS_MAX_RESTARTS}). Giving up."
    return 1
  fi
  if [[ -z "$CHASSIS_LAUNCH_CMD_STR" || -z "$CHASSIS_LOG_FILE" ]]; then
    log "Cannot restart chassis — original launch context not captured."
    return 1
  fi
  CHASSIS_RESTART_COUNT=$(( CHASSIS_RESTART_COUNT + 1 ))
  log "Restarting wheeltec chassis (attempt ${CHASSIS_RESTART_COUNT}/${CHASSIS_MAX_RESTARTS})"
  start_managed_process "Wheeltec chassis (restart ${CHASSIS_RESTART_COUNT})" \
    "$CHASSIS_LOG_FILE" bash -c "$CHASSIS_LAUNCH_CMD_STR"
  CHASSIS_PID="$LAST_STARTED_PID"
  sleep "$CHASSIS_STARTUP_DELAY_SEC"
  if ! is_pid_alive "$CHASSIS_PID"; then
    log "Wheeltec chassis restart failed (pid ${CHASSIS_PID} died within ${CHASSIS_STARTUP_DELAY_SEC}s)."
    return 1
  fi
  log "Wheeltec chassis restarted: new pid=${CHASSIS_PID}"
  start_chassis_liveness_watch
  return 0
}

handle_chassis_death() {
  # SIGUSR1 from the liveness watcher. Try one restart; if it fails fall
  # through to the standard cleanup path so the operator gets the same
  # shutdown behaviour as Ctrl+C.
  log 'Chassis death notification (SIGUSR1). Attempting restart.'
  # Old watcher has already exited (its target pid is gone). Clear the
  # stored PID so stop_chassis_liveness_watch in cleanup is a no-op.
  CHASSIS_LIVENESS_PID=""
  if restart_chassis_once; then
    return
  fi
  log 'Chassis restart failed — triggering full shutdown.'
  cleanup
  exit 1
}

stop_chassis_liveness_watch() {
  if [[ -n "$CHASSIS_LIVENESS_PID" ]] && kill -0 "$CHASSIS_LIVENESS_PID" 2>/dev/null; then
    kill "$CHASSIS_LIVENESS_PID" 2>/dev/null || true
  fi
  CHASSIS_LIVENESS_PID=""
}

start_bringup_liveness_watch() {
  # 与 start_chassis_liveness_watch 同构，但发 SIGUSR2 且不尝试重启。
  if [[ -z "$BRINGUP_PID" ]] || [[ "$DRY_RUN" == true ]]; then
    return 0
  fi
  local script_pid="$$"
  local bringup_pid="$BRINGUP_PID"
  local interval="$BRINGUP_LIVENESS_INTERVAL_SEC"
  log "Starting nav2 bringup liveness watch (poll ${interval}s, bringup pid=${bringup_pid})"
  (
    while kill -0 "$bringup_pid" 2>/dev/null; do
      sleep "$interval"
    done
    printf '[%s] WARN: Nav2 bringup (pid=%s) died. Notifying main.\n' \
      "$(date '+%Y-%m-%dT%H:%M:%S')" "$bringup_pid" >&2
    kill -USR2 "$script_pid" 2>/dev/null || true
    # ⚠ 只发信号不够。bash 在等待前台命令时收到已设 trap 的信号，会把
    # handler 推迟到该命令结束之后才执行（实测：前台 `sleep 6 | tee` 期间
    # 第 2 秒发 USR2，handler 到第 6 秒才跑）。主脚本此刻正阻塞在
    # `run_foreground ... | tee executor.log` 上，而 executor 在 nav2 死后
    # 只会一直重连、永不退出 —— 不动它的话 trap 永远轮不上，rosbag 继续写盘。
    # 所以这里主动终止 executor，让前台管道结束、被推迟的 USR2 handler 得以
    # 执行（handle_bringup_death -> cleanup）。
    # 只针对 executor 自身的 pid：它与主脚本同属一个进程组，按进程组 kill
    # 会把主脚本和本 watcher 一起带走。
    # 必须 TERM -> 等待 -> KILL 升级：2026-08-28 实测只发 TERM 时，
    # `ros2 run` 包装进程死了但真正的 python 子进程活了下来，它继承着写向
    # tee 的 stdout 管道 —— 只要它不退出，tee 的 stdin 就不关闭，前台管道
    # 就不结束，被推迟的 trap 永远等不到（rosbag 继续写盘）。
    ex_pids=$(pgrep -f "navigation_executor.py" 2>/dev/null || true)
    for ex_pid in $ex_pids; do
      printf '[%s] Terminating foreground executor (pid=%s) to unblock main.\n' \
        "$(date '+%Y-%m-%dT%H:%M:%S')" "$ex_pid" >&2
      kill -TERM "$ex_pid" 2>/dev/null || true
    done
    for _ in $(seq 1 "$BRINGUP_EXECUTOR_KILL_GRACE_SEC"); do
      pgrep -f "navigation_executor.py" >/dev/null 2>&1 || break
      sleep 1
    done
    for ex_pid in $(pgrep -f "navigation_executor.py" 2>/dev/null || true); do
      printf '[%s] Executor pid=%s survived SIGTERM; sending SIGKILL.\n' \
        "$(date '+%Y-%m-%dT%H:%M:%S')" "$ex_pid" >&2
      kill -KILL "$ex_pid" 2>/dev/null || true
    done
  ) &
  BRINGUP_LIVENESS_PID=$!
  disown "$BRINGUP_LIVENESS_PID" 2>/dev/null || true
}

stop_bringup_liveness_watch() {
  if [[ -n "$BRINGUP_LIVENESS_PID" ]] && kill -0 "$BRINGUP_LIVENESS_PID" 2>/dev/null; then
    kill "$BRINGUP_LIVENESS_PID" 2>/dev/null || true
  fi
  BRINGUP_LIVENESS_PID=""
}

handle_bringup_death() {
  # SIGUSR2 from the bringup watcher. Nav2 死了之后继续跑没有意义 ——
  # executor 会一直重连、rosbag 会一直写盘（改前实测 22 分钟 5.9GB）。
  # 不重启：BT / costmap / 位姿状态在 nav2 重启后都不可信。
  log 'Nav2 bringup death notification (SIGUSR2). Triggering full shutdown.'
  BRINGUP_LIVENESS_PID=""
  cleanup
  exit 1
}

publish_zero_cmd_vel() {
  # Send a short burst of zero Twists on /cmd_vel right before tearing down
  # nav2 / separator. STM32 has no cmd_vel watchdog (yet), so the last frame
  # before kill is what it executes; without this it can stay at the last
  # commanded non-zero velocity until something else stops it.
  if [[ "$ZERO_CMD_VEL_ON_SHUTDOWN" != true ]]; then
    return 0
  fi
  if [[ "$DRY_RUN" == true ]]; then
    return 0
  fi
  if ! command -v "$ROS2_BIN" >/dev/null 2>&1; then
    return 0
  fi
  log "Publishing zero /cmd_vel for ${ZERO_CMD_VEL_DURATION_SEC}s before shutdown"
  timeout "$ZERO_CMD_VEL_DURATION_SEC" \
    "$ROS2_BIN" topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
    >/dev/null 2>&1 || true
}

cleanup() {
  if [[ "$CLEANUP_DONE" == true ]]; then
    return
  fi
  CLEANUP_DONE=true

  # Stop watchers first — otherwise they can re-trigger us during cleanup.
  stop_chassis_liveness_watch
  stop_bringup_liveness_watch

  cleanup_executor_lock

  if ((${#CHILD_PIDS[@]} == 0)); then
    return
  fi

  # Send zero cmd_vel BEFORE killing nav2 / separator. Once they're dead
  # nothing publishes to /cmd_vel and STM32 keeps its last frame.
  publish_zero_cmd_vel

  log 'Stopping managed processes'
  snapshot_cleanup_targets
  signal_cleanup_targets INT
  if ! wait_for_shutdown; then
    log "Some processes are still alive after ${SHUTDOWN_GRACE_SECONDS}s"
    signal_cleanup_targets KILL
  fi
  reap_children
}

handle_signal() {
  local signal_name="$1"
  log "Received ${signal_name}, shutting down managed processes"
  cleanup
  exit 130
}

start_managed_process() {
  local name="$1"
  local log_file="$2"
  shift 2
  local pid
  local pgid

  if [[ "$DRY_RUN" == true ]]; then
    printf '[dry-run] '
    if [[ -n "$log_file" ]]; then
      printf '[log=%s] ' "$log_file"
    fi
    quote_command "$@"
    return 0
  fi

  log "Starting ${name}"
  if [[ -n "$log_file" ]]; then
    mkdir -p "$(dirname "$log_file")"
    setsid "$@" >>"$log_file" 2>&1 &
  else
    setsid "$@" &
  fi
  pid=$!
  pgid="$(ps -o pgid= -p "$pid" | tr -d '[:space:]')"
  if [[ -z "$pgid" ]]; then
    pgid="$pid"
  fi
  record_child "$name" "$pid" "$pgid"
  LAST_STARTED_PID="$pid"
  LAST_STARTED_PGID="$pgid"
  log "${name} started with pid ${pid}, pgid ${pgid}"
}

run_foreground() {
  if [[ "$DRY_RUN" == true ]]; then
    printf '[dry-run] '
    quote_command "$@"
    return 0
  fi
  "$@"
}

wait_for_nav_ready() {
  local deadline=$((SECONDS + NAV2_WAIT_TIMEOUT_SEC))

  log "Waiting for bt_navigator for up to ${NAV2_WAIT_TIMEOUT_SEC}s"
  while (( SECONDS < deadline )); do
    if [[ -n "$BRINGUP_PID" ]] && ! is_pid_alive "$BRINGUP_PID"; then
      printf 'Nav2 bringup exited before bt_navigator became ready\n' >&2
      return 1
    fi
    if "$ROS2_BIN" node list 2>/dev/null | grep -qE '(^|/)bt_navigator$'; then
      log 'bt_navigator is ready'
      return 0
    fi
    sleep 1
  done

  printf 'bt_navigator did not become ready within %ss\n' "$NAV2_WAIT_TIMEOUT_SEC" >&2
  return 1
}

param_value_text() {
  local node="$1"
  local param="$2"
  local output

  if ! output="$("$ROS2_BIN" param get "$node" "$param" 2>/dev/null)"; then
    return 1
  fi

  case "$output" in
    "String value is: "*)  printf '%s\n' "${output#String value is: }" ;;
    "Double value is: "*)  printf '%s\n' "${output#Double value is: }" ;;
    "Integer value is: "*) printf '%s\n' "${output#Integer value is: }" ;;
    "Boolean value is: "*) printf '%s\n' "${output#Boolean value is: }" ;;
    *)                     printf '%s\n' "$output" ;;
  esac
}

verify_nav2_runtime_config() {
  local local_frame global_frame transform_tolerance

  if [[ "$CHECK_RUNTIME_CONFIG" != true ]]; then
    return 0
  fi

  log "Checking Nav2 runtime config before launching executor"

  # 2026-05-20: verify is best-effort + warning-only. Previously this
  # function returned 1 on any mismatch which, combined with `set -e`,
  # killed the script + triggered cleanup → SIGINT to all nav2 nodes
  # before the executor could even connect. Per REP-105 local_costmap.
  # global_frame is `odom`, not `map`, so the old hardcoded check was
  # wrong anyway. Now warn and continue.
  if ! local_frame="$(param_value_text /local_costmap/local_costmap global_frame)"; then
    log "WARN: could not read /local_costmap/local_costmap global_frame (nav2 still configuring?)"
    return 0
  fi
  if ! global_frame="$(param_value_text /global_costmap/global_costmap global_frame)"; then
    log "WARN: could not read /global_costmap/global_costmap global_frame"
    return 0
  fi
  if ! transform_tolerance="$(param_value_text /controller_server FollowPath.transform_tolerance)"; then
    log "WARN: could not read /controller_server FollowPath.transform_tolerance"
    return 0
  fi

  log "Nav2 runtime config: local_costmap.global_frame=${local_frame}, global_costmap.global_frame=${global_frame}, FollowPath.transform_tolerance=${transform_tolerance}"

  # Sanity: per REP-105, local=odom, global=map. Warn but don't exit.
  if [[ "$local_frame" != "odom" ]]; then
    log "WARN: local_costmap.global_frame=${local_frame} (expected odom per REP-105)"
  fi
  if [[ "$global_frame" != "map" ]]; then
    log "WARN: global_costmap.global_frame=${global_frame} (expected map per REP-105)"
  fi
  if [[ -z "$transform_tolerance" ]]; then
    log "WARN: FollowPath.transform_tolerance empty"
  fi
  return 0
}

pin_jetson_clocks() {
  if command -v nvpmodel >/dev/null 2>&1; then
    log '[cold-start-fix] Setting nvpmodel to MAXN (mode 0)'
    printf '%s\n' "$SUDO_PASSWORD" | sudo -S -p '' nvpmodel -m 0 || \
      log '[cold-start-fix] WARN: nvpmodel failed (need sudo NOPASSWD?)'
  fi

  if command -v jetson_clocks >/dev/null 2>&1; then
    log '[cold-start-fix] Pinning CPU/GPU/EMC clocks to max'
    printf '%s\n' "$SUDO_PASSWORD" | sudo -S -p '' jetson_clocks || \
      log '[cold-start-fix] WARN: jetson_clocks failed (need sudo NOPASSWD?)'
  fi
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --config)
        HEADLESS_CONFIG_FILE="${2:-}"
        shift 2
        continue
        ;;
      --config=*)
        HEADLESS_CONFIG_FILE="${1#*=}"
        ;;
      --profile)
        HEADLESS_PROFILE="${2:-}"
        shift 2
        continue
        ;;
      --profile=*)
        HEADLESS_PROFILE="${1#*=}"
        ;;
      --pose-file)
        POSE_FILE="${2:-}"
        shift 2
        continue
        ;;
      --pose-file=*)
        POSE_FILE="${1#*=}"
        ;;
      -o|--output)
        BAG_NAME="${2:-}"
        shift 2
        continue
        ;;
      --output=*)
        BAG_NAME="${1#*=}"
        ;;
      --no-bag)
        FORCE_NO_BAG=true
        FORCE_RECORD_BAG=false
        ;;
      --bag)
        FORCE_RECORD_BAG=true
        FORCE_NO_BAG=false
        ;;
      --full-bag)
        FULL_BAG=true
        ;;
      --no-chassis)
        START_CHASSIS=false
        ;;
      --chassis)
        START_CHASSIS=true
        ;;
      --skip-runtime-config-check)
        CHECK_RUNTIME_CONFIG=false
        ;;
      -d|--dry-run)
        DRY_RUN=true
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      --)
        shift
        EXECUTOR_EXTRA_ARGS+=("$@")
        break
        ;;
      *)
        if [[ "$1" == *":="* ]]; then
          EXTRA_LAUNCH_ARGS+=("$1")
        elif [[ "$1" != -* && -z "$HEADLESS_PROFILE" ]]; then
          HEADLESS_PROFILE="$1"
        else
          EXECUTOR_EXTRA_ARGS+=("$1")
        fi
        ;;
    esac
    shift
  done
}

main() {
  parse_args "$@"
  source "${SCRIPT_DIR}/config/load_ros_env.sh"   # single-source ROS_DOMAIN_ID (config/ros_env.conf)

  if [[ -z "$BAG_NAME" ]]; then
    BAG_NAME="david_nav_headless_$(date +%Y%m%d_%H%M%S)"
  fi

  require_command bash
  require_command pgrep
  require_command ps
  require_command setsid
  require_path "$ROS_SETUP"
  require_path "$INSTALL_SETUP"
  require_path "$HEADLESS_CONFIG_FILE"
  require_path "$CONFIG_HELPER_BIN"
  require_path "$POSE_FILE"

  if [[ "$START_CHASSIS" == true ]]; then
    require_path "${WHEELTEC_WS_DIR}/install/setup.bash"
  fi

  local -a config_resolve_cmd=(
    "$PYTHON_BIN" "$CONFIG_HELPER_BIN"
    --config "$HEADLESS_CONFIG_FILE"
    --format shell
  )
  if [[ -n "$HEADLESS_PROFILE" ]]; then
    config_resolve_cmd+=(--profile "$HEADLESS_PROFILE")
  fi

  local headless_config_output
  if ! headless_config_output="$("${config_resolve_cmd[@]}")"; then
    printf '[%s] failed to resolve headless config (config=%s, profile=%s)\n' \
      "$SCRIPT_NAME" "$HEADLESS_CONFIG_FILE" "$HEADLESS_PROFILE" >&2
    exit 1
  fi
  eval "$headless_config_output"

  local use_rviz="false"
  local use_scan_bridge="false"
  local use_exploration="false"
  [[ "${HEADLESS_USE_RVIZ:-0}" == "1" ]] && use_rviz="true"
  [[ "${HEADLESS_ENABLE_SCAN_BRIDGE:-0}" == "1" ]] && use_scan_bridge="true"
  [[ "${HEADLESS_ENABLE_EXPLORATION:-0}" == "1" ]] && use_exploration="true"

  local record_bag=false
  if [[ "$FORCE_NO_BAG" == true ]]; then
    record_bag=false
  elif [[ "$FORCE_RECORD_BAG" == true ]]; then
    record_bag=true
  elif [[ "${HEADLESS_RECORD_BAG:-0}" == "1" ]]; then
    record_bag=true
  fi

  NAV2_WAIT_TIMEOUT_SEC="${HEADLESS_NAV2_WAIT_TIMEOUT_SEC:-$NAV2_WAIT_TIMEOUT_SEC}"

  safe_source "$ROS_SETUP"
  safe_source "$INSTALL_SETUP"
  require_command "$ROS2_BIN"

  if [[ -z "$EXECUTOR_BIN" ]]; then
    if [[ -x "${SCRIPT_DIR}/david_nav/navigation_executor.py" ]]; then
      EXECUTOR_BIN="${SCRIPT_DIR}/david_nav/navigation_executor.py"
    else
      EXECUTOR_BIN="$ROS2_BIN run david_nav navigation_executor.py"
    fi
  fi

  if [[ "$DRY_RUN" != true ]]; then
    mkdir -p "$BAG_ROOT"
  fi

  local bag_output="${BAG_ROOT}/${BAG_NAME}"
  local log_dir="${bag_output}/logs"
  # 与 navigate_semantic_pose_with_debug_bag.sh 一致：rosbag2 要求 --output
  # 目录不存在，而 logs/ 会先在 ${bag_output} 下建目录，导致 bag 静默录制
  # 失败（2026-07-17 复盘）。bag 落独立子目录，不与 logs/ 共享父目录。
  local bag_dir="${bag_output}/bag"
  if [[ -e "$bag_dir" ]]; then
    bag_dir="${bag_output}/bag_$(date +%H%M%S)"
  fi

  local -a bringup_cmd=(
    "$ROS2_BIN" launch david_nav 2d_david_nav2_bringup.launch.py
    use_rviz:="$use_rviz"
    enable_scan_bridge:="$use_scan_bridge"
    enable_exploration:="$use_exploration"
    # 临时（2026-07-23）：STM32 已重调位置环/速度环，软件侧速度整形停用
    # （smoother OPEN_LOOP 直通@20Hz + david_move 死区抬升归零），与
    # launch_nav_with_log.sh 保持一致，避免双重补偿。撤销=删下面这行；
    # 单次覆盖=EXTRA_LAUNCH_ARGS 传 cmd_shaping:=true（后传者胜）。
    cmd_shaping:=false
  )
  bringup_cmd+=("${EXTRA_LAUNCH_ARGS[@]}")

  local -a bag_cmd=(
    "$ROS2_BIN" bag record
    --max-cache-size 104857600
    --output "$bag_dir"
  )
  if [[ "$FULL_BAG" == true ]]; then
    bag_cmd+=(
      --include-unpublished-topics
      --include-hidden-topics
      --compression-mode file
      --compression-format zstd
      "${EXPLICIT_TOPICS[@]}"
      -e "$REGEX"
    )
  else
    bag_cmd+=("${LIGHT_BAG_TOPICS[@]}")
  fi

  local -a executor_cmd=()
  # EXECUTOR_BIN may be either an absolute path or 'ros2 run david_nav ...'; split on spaces.
  read -r -a executor_cmd <<<"$EXECUTOR_BIN"
  executor_cmd+=(
    --server-uri "${HEADLESS_SERVER_URI}"
    --reconnect-delay "${HEADLESS_RECONNECT_DELAY}"
    --pose-file "$POSE_FILE"
  )
  if (( ${#EXECUTOR_EXTRA_ARGS[@]} > 0 )); then
    executor_cmd+=("${EXECUTOR_EXTRA_ARGS[@]}")
  fi

  trap 'handle_signal INT' INT
  trap 'handle_signal TERM' TERM
  trap 'handle_chassis_death' USR1
  trap 'handle_bringup_death' USR2
  trap cleanup EXIT

  log "Using ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  log "Profile: ${HEADLESS_PROFILE:-${HEADLESS_PROFILE}} (rviz=${use_rviz}, scan_bridge=${use_scan_bridge}, exploration=${use_exploration})"
  log "Server URI: ${HEADLESS_SERVER_URI}"
  log "Pose file: ${POSE_FILE}"
  log "Bag output: ${bag_output} (record=${record_bag}, full=${FULL_BAG})"
  log "Chassis: $([[ "$START_CHASSIS" == true ]] && echo "${WHEELTEC_LAUNCH_PACKAGE}/${WHEELTEC_LAUNCH_FILE}" || echo skipped)"
  log "Process logs: ${log_dir}"

  stop_stale_executor
  pin_jetson_clocks

  # Pre-flight: kill MoveIt residues + stale robot_state_publishers that
  # would publish a *conflicting* URDF (e.g. one with `world → base_link`
  # from david_moveit_config) OR a duplicate chassis URDF (from a prior
  # nav2 run that didn't exit cleanly). Either case puts two parents on
  # base_link ⇒ tf2 splits the tree ⇒ "no map frame" / "two unconnected
  # trees" ⇒ executor never gets goal feedback. Mirrors the same logic
  # in scripts/launch_nav_with_log.sh and navigate_semantic_pose_with_debug_bag.sh.
  #
  # 2026-07-24 扩展（当晨三连败复盘）：
  #   04:13 TF 两棵树      = 残留 RSP/上代 URDF（原逻辑已覆盖）
  #   04:19 controller_server lifecycle SIGABRT("publisher's context is
  #        invalid", 07-23 04:48 同款) = 上代 nav2 栈还在退出中就重启新代
  # 新增：① 清上代 nav2 全家 + bringup launch 包装 + david_move；
  #      ② START_CHASSIS=true 时清残留 wheeltec（抢串口 + 双发 odom）；
  #      ③ TERM 后轮询等待全部退净(≤8s)，残余再 SIGKILL —— 只 sleep 1 秒
  #        不等退净正是 lifecycle race 的来源。
  # DAVID_SKIP_PREFLIGHT_CLEANUP=true：跳过整个残留清理（供测试沙箱使用——
  # 测试在真机上跑脚本时，pgrep 清理会误杀正在运行的真实 nav2/wheeltec 栈）。
  if [[ "${DAVID_SKIP_PREFLIGHT_CLEANUP:-false}" == "true" ]]; then
    log 'Pre-flight: skipped (DAVID_SKIP_PREFLIGHT_CLEANUP=true)'
  elif [[ "$DRY_RUN" != true ]]; then
    local -a pre_kill_patterns=(
      "moveit_ros_move_group/move_group"
      "moveit_simple_controller_manager"
      "david_sdk_bridge/lib/david_sdk_bridge/arm_node"
      # 上代 nav2 栈（先杀 launch 包装，让它自己 TERM 子进程，再逐个补刀）。
      # pattern 刻意用"空格分隔的 launch 形态"或"安装路径形态"，避免误伤
      # 恰好在编辑这些文件的 vim/grep（其 cmdline 只含 路径/文件名 形态）。
      "launch david_nav 2d_david_nav2_bringup"
      "nav2_controller/controller_server"
      "nav2_planner/planner_server"
      "nav2_smoother/smoother_server"
      "nav2_behaviors/behavior_server"
      "nav2_bt_navigator/bt_navigator"
      "nav2_waypoint_follower/waypoint_follower"
      "nav2_velocity_smoother/velocity_smoother"
      "nav2_lifecycle_manager/lifecycle_manager"
      "lib/david_cmd/david_move"
    )
    if [[ "$START_CHASSIS" == true ]]; then
      # 本脚本将自起 wheeltec；残留的会占住串口（PortNotOpenedException 复盘
      # 见 2026-07-24）并双发 odom。--no-chassis 时不清，尊重外部 alias 5。
      pre_kill_patterns+=(
        "launch turn_on_wheeltec_robot"
        "wheeltec_robot_node"
        "twist_transform"
      )
    fi
    log 'Pre-flight: killing MoveIt residues, stale RSPs and stale nav2/chassis generations'
    local script_self=$$
    local script_parent=$PPID
    local pat pid
    for pat in "${pre_kill_patterns[@]}"; do
      for pid in $(pgrep -f -- "$pat" 2>/dev/null); do
        [[ "$pid" == "$script_self" ]] && continue
        [[ "$pid" == "$script_parent" ]] && continue
        if kill "$pid" 2>/dev/null; then
          log "  killed pid=$pid (pattern=$pat)"
        fi
      done
    done
    # Kill every existing robot_state_publisher — the nav2 launch starts
    # its own with the chassis URDF; any pre-existing RSP is necessarily
    # a leftover (chassis or MoveIt-side) and will cause TF tree splits.
    for pid in $(pgrep -f robot_state_publisher 2>/dev/null); do
      [[ "$pid" == "$script_self" ]] && continue
      log "  killed stale robot_state_publisher pid=$pid"
      kill -9 "$pid" 2>/dev/null || true
    done
    # 等待所有残留真正退出（上代 nav2 处于 lifecycle 关闭中时，新代立刻
    # 起会撞出 rcl context invalid / SIGABRT）。8s 超时后 SIGKILL 补刀。
    local wait_deadline=$((SECONDS + 8))
    local leftover=""
    while (( SECONDS < wait_deadline )); do
      leftover=""
      for pat in "${pre_kill_patterns[@]}"; do
        for pid in $(pgrep -f -- "$pat" 2>/dev/null); do
          [[ "$pid" == "$script_self" || "$pid" == "$script_parent" ]] && continue
          leftover+="$pid "
        done
      done
      [[ -z "$leftover" ]] && break
      sleep 0.3
    done
    if [[ -n "$leftover" ]]; then
      log "  Pre-flight: force-killing leftovers after 8s: $leftover"
      for pid in $leftover; do
        kill -9 "$pid" 2>/dev/null || true
      done
      sleep 0.5
    fi
    log 'Pre-flight done (all stale generations gone).'
  fi

  if [[ "$START_CHASSIS" == true ]]; then
    local chassis_cmd_str
    printf -v chassis_cmd_str 'source %q && source %q && exec ros2 launch %q %q' \
      "$ROS_SETUP" "${WHEELTEC_WS_DIR}/install/setup.bash" \
      "$WHEELTEC_LAUNCH_PACKAGE" "$WHEELTEC_LAUNCH_FILE"
    # Cache the launch context so restart_chassis_once can reuse it from
    # the SIGUSR1 trap without re-deriving WHEELTEC_* env.
    CHASSIS_LAUNCH_CMD_STR="$chassis_cmd_str"
    CHASSIS_LOG_FILE="${log_dir}/chassis.log"
    start_managed_process 'Wheeltec chassis' "$CHASSIS_LOG_FILE" bash -c "$chassis_cmd_str"
    CHASSIS_PID="$LAST_STARTED_PID"
    if [[ "$DRY_RUN" != true ]]; then
      sleep "$CHASSIS_STARTUP_DELAY_SEC"
      if ! is_pid_alive "$CHASSIS_PID"; then
        printf 'Wheeltec chassis exited early; see %s\n' "$CHASSIS_LOG_FILE" >&2
        exit 1
      fi
      start_chassis_liveness_watch
    fi
  else
    log 'Skipping wheeltec chassis launch'
  fi

  start_managed_process 'Nav2 bringup' "${log_dir}/bringup.log" "${bringup_cmd[@]}"
  if [[ "$DRY_RUN" != true ]]; then
    BRINGUP_PID="$LAST_STARTED_PID"
    sleep 2
    if ! is_pid_alive "$BRINGUP_PID"; then
      printf 'Nav2 bringup exited early; see %s and ~/.ros/log for details\n' "${log_dir}/bringup.log" >&2
      exit 1
    fi
    wait_for_nav_ready
    verify_nav2_runtime_config
    # 2026-08-28: 起在 rosbag 之前，这样 nav2 一死 rosbag 立刻被 cleanup 收掉。
    start_bringup_liveness_watch
  fi

  if [[ "$record_bag" == true ]]; then
    start_managed_process 'debug rosbag' "${log_dir}/rosbag.log" "${bag_cmd[@]}"
    if [[ "$DRY_RUN" != true ]]; then
      sleep 1
    fi
  else
    log 'Skipping debug rosbag'
  fi

  log 'Starting navigation_executor (foreground). Press Ctrl+C to stop everything.'
  # 2026-07-24: 导航失败原因必须落盘可查。executor 前台输出(含
  # nav2_action_bridge 的逐次失败详情 + 1s 消抖重发日志 + 最终上报 server
  # 的 on_error 帧)tee 到 executor.log,终端照常可见。
  local executor_log_file="${log_dir}/executor.log"
  if [[ "$DRY_RUN" == true ]]; then
    run_foreground "${executor_cmd[@]}"
  else
    mkdir -p "$log_dir"
    log "Executor log: ${executor_log_file}"
    run_foreground env PYTHONUNBUFFERED=1 "${executor_cmd[@]}" 2>&1 | tee -a "$executor_log_file"
  fi
}

main "$@"
