#!/usr/bin/env bash

set -euo pipefail

SCRIPT_NAME="$(basename "$0")"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 2026-05-20: default changed 30 → 0 to match the rest of the robot
# stack (lightning loc, wheeltec, lslidar all run in domain 0 by default).
# Whole-stack domain comes from the single source config/ros_env.conf (change
# the domain in ONE place). A mismatch means nav2 + RViz can't hear lightning's
# map→odom TF and the user sees "frame map does not exist" / no map.
# Applied in main() via config/load_ros_env.sh; set ROS_DOMAIN_ID in your shell
# BEFORE invoking to override for a single run.
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
AUTO_WORKSPACE="${AUTO_WORKSPACE:-${HOME}/ros2_ws}"
SEMANTIC_MAP_DIR="${SEMANTIC_MAP_DIR:-${HOME}/semantic_map}"
POSE_FILE="${POSE_FILE:-${SEMANTIC_MAP_DIR}/named_poses.json}"
BAG_ROOT="${BAG_ROOT:-${HOME}/rosbags}"
BAG_NAME="${BAG_NAME:-}"
TARGET="${TARGET:-}"
RVIZ=true
# 2026-05-21: scan_bridge (pointcloud_to_laserscan) now defaults OFF in .sh,
# matching the user's preferred direct `ros2 launch ... enable_scan_bridge:=false`.
# Turning the bridge on subscribes to /lightning/loc/aligned_scan (PointCloud2),
# republishes /scan (LaserScan). When SLAM isn't actively publishing the source
# cloud, the bridge just idles; but in some runs it adds CPU and TF traffic for
# no benefit. Enable explicitly with --scan-bridge when /scan is needed by some
# downstream (e.g. enabled obstacle_layer + scan source).
SCAN_BRIDGE=false
START_BRINGUP=true
RECORD_BAG=true
WAIT_FOR_NAV_READY=true
CHECK_RUNTIME_CONFIG=true
FULL_BAG=false
NAV_READY_TIMEOUT_SEC="${NAV_READY_TIMEOUT_SEC:-120}"
SHUTDOWN_GRACE_SECONDS="${SHUTDOWN_GRACE_SECONDS:-5}"
SERVER_TIMEOUT_SEC="${SERVER_TIMEOUT_SEC:-10.0}"
RESULT_TIMEOUT_SEC="${RESULT_TIMEOUT_SEC:-300.0}"
GOAL_PAUSE_SEC="${GOAL_PAUSE_SEC:-0.0}"
ROBOT_BASE_FRAME="${ROBOT_BASE_FRAME:-base}"
ARRIVAL_CHECK_DISTANCE="${ARRIVAL_CHECK_DISTANCE:-0.75}"
EXPECTED_NAV2_FRAME="${EXPECTED_NAV2_FRAME:-map}"
EXPECTED_DWB_TRANSFORM_TOLERANCE="${EXPECTED_DWB_TRANSFORM_TOLERANCE:-2.0}"
DRY_RUN=false
ROS2_BIN="${ROS2_BIN:-ros2}"
NAVIGATOR_BIN="${NAVIGATOR_BIN:-}"

WHEELTEC_WS_DIR="${WHEELTEC_WS_DIR:-${HOME}/wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2}"
WHEELTEC_LAUNCH_PACKAGE="${WHEELTEC_LAUNCH_PACKAGE:-turn_on_wheeltec_robot}"
WHEELTEC_LAUNCH_FILE="${WHEELTEC_LAUNCH_FILE:-turn_on_wheeltec_robot.launch.py}"
CHASSIS_STARTUP_DELAY_SEC="${CHASSIS_STARTUP_DELAY_SEC:-3}"
START_CHASSIS=true

declare -a CHILD_PIDS=()
declare -a CHILD_NAMES=()
declare -a CHILD_PGIDS=()
declare -a CLEANUP_PIDS=()
declare -a CLEANUP_PGIDS=()
declare -a NAVIGATOR_ARGS=()
declare -a EXTRA_LAUNCH_ARGS=()
CLEANUP_DONE=false
LAST_STARTED_PID=""
LAST_STARTED_PGID=""
BRINGUP_PID=""

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
  # /lightning/grid_map —— 2026-07-23 移出 light 录制（107GB/3.4h 实测教训，
  # 还把 TF 压到 11Hz）。确需原图用 --full-bag（EXPLICIT_TOPICS 仍含它）。
  /local_costmap/costmap
  /local_costmap/costmap_raw
  /local_costmap/published_footprint
  /global_costmap/costmap
  /global_costmap/costmap_raw
  /global_costmap/published_footprint
  /navigate_to_pose/_action/status
  /navigate_to_pose/_action/feedback
  /navigate_to_pose/_action/result
  /plan
)

REGEX='^/((local_costmap|global_costmap|frontier|explore|collision_monitor|navigate_to_pose|navigate_through_poses|plan|goal_pose|clicked_point|behavior_tree_log|map_live_ready)(/.*)?)$'

usage() {
  cat <<'EOF'
Usage:
  navigate_semantic_pose_with_debug_bag.sh [options]
  navigate_semantic_pose_with_debug_bag.sh --target POI_ID [options]
  navigate_semantic_pose_with_debug_bag.sh POI_ID [options]

Behavior:
  Source the local ROS environment, optionally launch david_nav 2D Nav2 bringup,
  start a debug rosbag, then navigate to a pose from ${HOME}/semantic_map/named_poses.json.
  Without a TARGET, the terminal will show a numbered pose list for selection.

  Lightning SLAM is expected to be running independently. This script does not start
  SLAM, localization, or relocation.

Options:
  --target TARGET           Pose id, pose name, or all
  --pose-file PATH          Named pose JSON. Default: ${HOME}/semantic_map/named_poses.json
  --no-bringup              Skip david_nav Nav2 bringup and only do bag + navigation
  --rviz                    Launch RViz with 2d_david_nav2_bringup.launch.py. Default: enabled.
  --scan-bridge             Run pointcloud_to_laserscan (PC2 → /scan). Default: OFF.
  --no-scan-bridge          Explicitly disable scan_bridge (already the default).
  --no-rviz                 Disable RViz in bringup
  --nav-ready-timeout SEC   Wait timeout for bt_navigator readiness. Default: 120
  --skip-nav-ready-wait     Do not wait for bt_navigator before bag + navigation
  --result-timeout-sec SEC  Timeout for each Nav2 navigation result. Default: 300
  --goal-pause-sec SEC      Delay between goals when TARGET is all. Default: 0
  --arrival-check-distance M
                            Reject Nav2 success if robot is farther than this
                            from the target in map frame. Default: 0.75
  --robot-base-frame FRAME  Robot frame used for arrival verification. Default: base
  --skip-runtime-config-check
                            Skip Nav2 runtime frame/tolerance sanity checks
  --no-bag                  Do not start rosbag recording. Lowest CPU.
  --full-bag                Record full debug bag with hidden/unpublished topics,
                            zstd compression, lidar/diagnostic topics, and regex
                            matches. Higher CPU.
  --no-chassis              Skip wheeltec chassis launch (for bench testing)
  -o, --output NAME         Bag folder name. Default: david_semantic_nav_YYYYmmdd_HHMMSS
  -d, --dry-run             Print commands without starting processes
  -h, --help                Show this help
  --                        Forward the remaining args to semantic_pose_navigator.py

Environment variables:
  ROS_DOMAIN_ID             Override domain for this run (default from config/ros_env.conf)
  ROS_SETUP                 ROS setup.bash path. Default: /opt/ros/humble/setup.bash
  AUTO_WORKSPACE            ROS 2 workspace containing david_nav. Default: ${HOME}/ros2_ws
  SEMANTIC_MAP_DIR          Directory containing named_poses.json. Default: ${HOME}/semantic_map
  POSE_FILE                 Full named pose JSON path
  BAG_ROOT                  Output root for rosbag folders. Default: ${HOME}/rosbags
  NAVIGATOR_BIN             Optional executable path for semantic_pose_navigator.py
  EXPECTED_NAV2_FRAME       Expected controller/costmap frame. Default: map
  EXPECTED_DWB_TRANSFORM_TOLERANCE
                            Expected DWB transform tolerance. Default: 2.0
  WHEELTEC_WS_DIR           Path to wheeltec_robot_ros2 workspace
  WHEELTEC_LAUNCH_PACKAGE   Launch package (default: turn_on_wheeltec_robot)
  WHEELTEC_LAUNCH_FILE      Launch file (default: turn_on_wheeltec_robot.launch.py)
  CHASSIS_STARTUP_DELAY_SEC Seconds to wait after chassis launch before Nav2. Default: 3
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

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --target)
        TARGET="${2:-}"
        shift 2
        continue
        ;;
      --target=*)
        TARGET="${1#*=}"
        ;;
      --pose-file)
        POSE_FILE="${2:-}"
        shift 2
        continue
        ;;
      --pose-file=*)
        POSE_FILE="${1#*=}"
        ;;
      --no-bringup)
        START_BRINGUP=false
        ;;
      --no-bag)
        RECORD_BAG=false
        ;;
      --rviz)
        RVIZ=true
        ;;
      --no-rviz)
        RVIZ=false
        ;;
      --scan-bridge)
        SCAN_BRIDGE=true
        ;;
      --no-scan-bridge)
        SCAN_BRIDGE=false
        ;;
      --nav-ready-timeout)
        NAV_READY_TIMEOUT_SEC="${2:-}"
        shift 2
        continue
        ;;
      --nav-ready-timeout=*)
        NAV_READY_TIMEOUT_SEC="${1#*=}"
        ;;
      --skip-nav-ready-wait)
        WAIT_FOR_NAV_READY=false
        ;;
      --skip-runtime-config-check)
        CHECK_RUNTIME_CONFIG=false
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
      --result-timeout-sec)
        RESULT_TIMEOUT_SEC="${2:-}"
        shift 2
        continue
        ;;
      --result-timeout-sec=*)
        RESULT_TIMEOUT_SEC="${1#*=}"
        ;;
      --goal-pause-sec)
        GOAL_PAUSE_SEC="${2:-}"
        shift 2
        continue
        ;;
      --goal-pause-sec=*)
        GOAL_PAUSE_SEC="${1#*=}"
        ;;
      --arrival-check-distance)
        ARRIVAL_CHECK_DISTANCE="${2:-}"
        shift 2
        continue
        ;;
      --arrival-check-distance=*)
        ARRIVAL_CHECK_DISTANCE="${1#*=}"
        ;;
      --robot-base-frame)
        ROBOT_BASE_FRAME="${2:-}"
        shift 2
        continue
        ;;
      --robot-base-frame=*)
        ROBOT_BASE_FRAME="${1#*=}"
        ;;
      -o|--output)
        BAG_NAME="${2:-}"
        shift 2
        continue
        ;;
      --output=*)
        BAG_NAME="${1#*=}"
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
        NAVIGATOR_ARGS+=("$@")
        break
        ;;
      *)
        if [[ "$1" == *":="* ]]; then
          EXTRA_LAUNCH_ARGS+=("$1")
        elif [[ "$1" != -* && -z "$TARGET" ]]; then
          TARGET="$1"
        else
          NAVIGATOR_ARGS+=("$1")
        fi
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

safe_source() {
  local source_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "$source_file"
  set -u
}

is_pid_alive() {
  local pid="$1"
  kill -0 "$pid" >/dev/null 2>&1
}

is_pgid_alive() {
  local pgid="$1"
  pgrep -g "$pgid" >/dev/null 2>&1
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
  local deadline=$((SECONDS + NAV_READY_TIMEOUT_SEC))

  if [[ "$WAIT_FOR_NAV_READY" != true ]]; then
    return 0
  fi

  log "Waiting for bt_navigator for up to ${NAV_READY_TIMEOUT_SEC}s"
  while (( SECONDS < deadline )); do
    if [[ -n "$BRINGUP_PID" ]] && ! is_pid_alive "$BRINGUP_PID"; then
      printf 'David Nav2 bringup exited before bt_navigator became ready\n' >&2
      return 1
    fi
    if "$ROS2_BIN" node list 2>/dev/null | grep -qE '(^|/)bt_navigator$'; then
      log 'bt_navigator is ready'
      return 0
    fi
    sleep 1
  done

  printf 'bt_navigator did not become ready within %ss\n' "$NAV_READY_TIMEOUT_SEC" >&2
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
    "String value is: "*)
      printf '%s\n' "${output#String value is: }"
      ;;
    "Double value is: "*)
      printf '%s\n' "${output#Double value is: }"
      ;;
    "Integer value is: "*)
      printf '%s\n' "${output#Integer value is: }"
      ;;
    "Boolean value is: "*)
      printf '%s\n' "${output#Boolean value is: }"
      ;;
    *)
      printf '%s\n' "$output"
      ;;
  esac
}

verify_nav2_runtime_config() {
  local local_frame
  local global_frame
  local transform_tolerance

  if [[ "$CHECK_RUNTIME_CONFIG" != true || "$START_BRINGUP" != true ]]; then
    return 0
  fi

  log "Checking Nav2 runtime config before sending goals"

  # 2026-05-20: verify is best-effort + warning-only. Previously this
  # function returned 1 on any mismatch which, combined with `set -e`,
  # killed the script + triggered cleanup → SIGINT to all nav2 nodes
  # within ~15 s of bringup. That left RViz with no map and looked
  # like a costmap-display bug. Per REP-105 local_costmap.global_frame
  # is `odom`, not `map`, so the old hardcoded check was wrong anyway.
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

main() {
  parse_args "$@"
  source "${SCRIPT_DIR}/config/load_ros_env.sh"   # single-source ROS_DOMAIN_ID (config/ros_env.conf)

  if [[ -z "$BAG_NAME" ]]; then
    BAG_NAME="david_semantic_nav_$(date +%Y%m%d_%H%M%S)"
  fi

  require_command bash
  require_command pgrep
  require_command ps
  require_command setsid
  require_path "$ROS_SETUP"
  require_path "${AUTO_WORKSPACE}/install/setup.bash"
  require_path "$POSE_FILE"

  if [[ "$START_CHASSIS" == true ]]; then
    require_path "${WHEELTEC_WS_DIR}/install/setup.bash"
  fi

  safe_source "$ROS_SETUP"
  safe_source "${AUTO_WORKSPACE}/install/setup.bash"
  require_command "$ROS2_BIN"

  if [[ "$DRY_RUN" != true ]]; then
    mkdir -p "$BAG_ROOT"
  fi

  # Pre-flight: kill MoveIt residues that would publish a conflicting
  # URDF (world→base_link from david_moveit_config). Two RSPs publishing
  # different parents for base_link splits tf2 into two trees → nav2
  # can't lookup base_footprint↔odom → costmaps/RViz fail silently.
  # Mirrors the logic in scripts/launch_nav_with_log.sh.
  if [[ "$DRY_RUN" != true ]]; then
    local -a pre_kill_patterns=(
      "moveit_ros_move_group/move_group"
      "moveit_simple_controller_manager"
      "david_sdk_bridge/lib/david_sdk_bridge/arm_node"
    )
    log 'Pre-flight: killing MoveIt residues that would conflict with nav2 TF tree'
    local script_self=$$
    local script_parent=$PPID
    local pat pid pfile is_moveit
    for pat in "${pre_kill_patterns[@]}"; do
      for pid in $(pgrep -f -- "$pat" 2>/dev/null); do
        [[ "$pid" == "$script_self" ]] && continue
        [[ "$pid" == "$script_parent" ]] && continue
        if kill "$pid" 2>/dev/null; then
          log "  killed pid=$pid (pattern=$pat)"
        fi
      done
    done
    # Kill EVERY existing robot_state_publisher. nav2 launch starts its
    # own RSP with the chassis URDF; any pre-existing RSP — whether
    # MoveIt-side (conflicting `world -> base_link`) or a stale chassis
    # RSP from a previous run (duplicate `base_link -> wheel_link`
    # publishes) — corrupts the tf2 tree and makes map/odom lookups fail.
    for pid in $(pgrep -f robot_state_publisher 2>/dev/null); do
      [[ "$pid" == "$script_self" ]] && continue
      log "  killed stale robot_state_publisher pid=$pid"
      kill -9 "$pid" 2>/dev/null || true
    done
    sleep 1
    log 'Pre-flight done.'
  fi

  local bag_output="${BAG_ROOT}/${BAG_NAME}"
  local log_dir="${bag_output}/logs"
  # rosbag2 要求 --output 目录**不存在**，而 logs/ 会先在 ${bag_output} 下建目录，
  # 曾导致所有 run 的 bag 静默录制失败（2026-07-17 复盘：rosbag.log 只有一行
  # "Output folder already exists"）。bag 落到独立子目录，绝不与 logs/ 共享父目录。
  local bag_dir="${bag_output}/bag"
  if [[ -e "$bag_dir" ]]; then
    bag_dir="${bag_output}/bag_$(date +%H%M%S)"
  fi
  local -a nav_cmd
  local -a bringup_cmd=(
    "$ROS2_BIN" launch david_nav 2d_david_nav2_bringup.launch.py
    use_rviz:="$RVIZ"
    enable_scan_bridge:="$SCAN_BRIDGE"
    enable_exploration:=false
  )
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
    )
  fi

  bringup_cmd+=("${EXTRA_LAUNCH_ARGS[@]}")

  if [[ -n "$NAVIGATOR_BIN" ]]; then
    require_path "$NAVIGATOR_BIN"
    nav_cmd=("$NAVIGATOR_BIN")
  elif [[ -x "${SCRIPT_DIR}/david_nav/semantic_pose_navigator.py" ]]; then
    nav_cmd=("${SCRIPT_DIR}/david_nav/semantic_pose_navigator.py")
  else
    nav_cmd=("$ROS2_BIN" run david_nav semantic_pose_navigator.py)
  fi
  if [[ -n "$TARGET" ]]; then
    nav_cmd+=("$TARGET")
  fi
  nav_cmd+=(
    --pose-file "$POSE_FILE"
    --server-timeout-sec "$SERVER_TIMEOUT_SEC"
    --result-timeout-sec "$RESULT_TIMEOUT_SEC"
    --goal-pause-sec "$GOAL_PAUSE_SEC"
    --robot-base-frame "$ROBOT_BASE_FRAME"
    --arrival-check-distance "$ARRIVAL_CHECK_DISTANCE"
  )

  if [[ "$FULL_BAG" == true ]]; then
    bag_cmd+=("${EXPLICIT_TOPICS[@]}")
    bag_cmd+=(-e "$REGEX")
  else
    bag_cmd+=("${LIGHT_BAG_TOPICS[@]}")
  fi
  nav_cmd+=("${NAVIGATOR_ARGS[@]}")

  trap 'handle_signal INT' INT
  trap 'handle_signal TERM' TERM
  trap cleanup EXIT

  log "Using ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  log "Pose file: ${POSE_FILE}"
  if [[ -n "$TARGET" ]]; then
    log "Target: ${TARGET}"
  else
    log 'Target: interactive terminal selection'
  fi
  log "Bag output: ${bag_dir}"
  log "Chassis: $([[ "$START_CHASSIS" == true ]] && echo "${WHEELTEC_LAUNCH_PACKAGE}/${WHEELTEC_LAUNCH_FILE}" || echo skipped)"
  log "Process logs: ${log_dir}"

  if [[ "$START_CHASSIS" == true ]]; then
    local chassis_cmd_str
    printf -v chassis_cmd_str 'source %q && source %q && exec ros2 launch %q %q' \
      "$ROS_SETUP" "${WHEELTEC_WS_DIR}/install/setup.bash" \
      "$WHEELTEC_LAUNCH_PACKAGE" "$WHEELTEC_LAUNCH_FILE"
    start_managed_process 'Wheeltec chassis' "${log_dir}/chassis.log" bash -c "$chassis_cmd_str"
    if [[ "$DRY_RUN" != true ]]; then
      sleep "$CHASSIS_STARTUP_DELAY_SEC"
      if ! is_pid_alive "$LAST_STARTED_PID"; then
        printf 'Wheeltec chassis exited early; see %s\n' "${log_dir}/chassis.log" >&2
        exit 1
      fi
    fi
  else
    log 'Skipping wheeltec chassis launch'
  fi

  if [[ "$START_BRINGUP" == true ]]; then
    start_managed_process 'David Nav2 bringup' "${log_dir}/bringup.log" "${bringup_cmd[@]}"
    if [[ "$DRY_RUN" != true ]]; then
      BRINGUP_PID="$LAST_STARTED_PID"
      sleep 2
      if ! is_pid_alive "$BRINGUP_PID"; then
        printf 'David Nav2 bringup exited early; see %s and ~/.ros/log for details\n' "${log_dir}/bringup.log" >&2
        exit 1
      fi
      wait_for_nav_ready
      verify_nav2_runtime_config
    fi
  else
    log 'Skipping David Nav2 bringup'
  fi

  if [[ "$RECORD_BAG" == true ]]; then
    start_managed_process 'debug rosbag' "${log_dir}/rosbag.log" "${bag_cmd[@]}"
    if [[ "$DRY_RUN" != true ]]; then
      sleep 1
    fi
  else
    log 'Skipping debug rosbag'
  fi

  run_foreground "${nav_cmd[@]}"
}

main "$@"
