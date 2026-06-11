#!/usr/bin/env bash
# Headless 坐标点导航(nav 协议 v2,2026-06-10)。
#
# start_navigation_headless.sh 的 waypoint 版:
#   * bringup 用 scripts/launch_waypoint_nav.sh(planner-only Nav2 +
#     position_controller + waypoint_follower,pre-flight 清理/日志都在那边)
#   * 前台跑 navigation_executor.py,WebSocket client 连 server,
#     navigate_to 直接带 target_pose {x,y,theta} —— 不再查 named_poses.json
#   * action 指向 /waypoint_follower/navigate(NavigateToPose 同类型,
#     不是 bt_navigator)
#
# 协议差异(v2 vs v1):仅 navigate_to 的目标从 target_id(POI 字符串)变成
# target_pose 坐标三元组;progress/arrived/error/abort/心跳 全部不变。
# theta 默认按弧度;server 确认是度的话加 --theta-degrees。
#
# 本脚本只管导航侧:jetson_clocks 锁频 → wheeltec 底盘(默认带起,启动前
# 查重、旧进程杀掉再起)→ waypoint 导航栈 → 前台执行器。
#
# SLAM 侧(IMU wit_imu / 雷达 lslidar c16 / 重定位 run_loc_online)由 SLAM
# 那边的流程负责,本脚本不碰 —— 跑导航前确保 Lightning loc 已在出
# /lightning/grid_map + map->odom->base TF(同一 ROS_DOMAIN_ID)。
#
# 用法:
#   start_waypoint_nav_headless.sh [options] [arg:=val …] [-- executor-args…]
#     --server-uri URI     WebSocket server(默认 ws://172.16.22.205:8101/ws/navigation/executor)
#     --theta-degrees      v2 target_pose.theta 按度解释(默认弧度)
#     --no-chassis         不代起 wheeltec 底盘
#     --use-rviz           bringup 带 RViz(默认 headless 不带)
#     -d, --dry-run        只打印命令
#     arg:=val             透传给 waypoint_nav_bringup.launch.py
#     -- …                 透传给 navigation_executor.py
#
# 环境变量: SUDO_PASSWORD(默认 1234,jetson_clocks 用)
#
# 日志: bringup → /tmp/waypoint_nav_latest.log(symlink,launch_waypoint_nav.sh 维护)
#       底盘   → /tmp/waypoint_nav_chassis.log
#       executor 在前台,Ctrl+C 收导航栈退出(底盘留着,下次跑会杀旧重启)。

set -euo pipefail

SCRIPT_NAME="$(basename "$0")"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

WS_DIR="${DAVID_WS_DIR:-${HOME}/ros2_ws}"
ROS_SETUP="${DAVID_ROS_SETUP_FILE:-/opt/ros/humble/setup.bash}"
INSTALL_SETUP="${DAVID_INSTALL_SETUP_FILE:-${WS_DIR}/install/setup.bash}"
LAUNCH_WRAPPER="${SCRIPT_DIR}/scripts/launch_waypoint_nav.sh"
EXECUTOR_SRC="${SCRIPT_DIR}/david_nav/navigation_executor.py"

SERVER_URI="${NAV_SERVER_URI:-ws://172.16.22.205:8101/ws/navigation/executor}"
ACTION_NAME="${NAV_ACTION_NAME:-/waypoint_follower/navigate}"
RECONNECT_DELAY="${NAV_RECONNECT_DELAY:-3.0}"
WAYPOINT_NODE_WAIT_SEC="${WAYPOINT_NODE_WAIT_SEC:-60}"

WHEELTEC_WS_DIR="${WHEELTEC_WS_DIR:-${HOME}/wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2}"
WHEELTEC_LAUNCH_PACKAGE="${WHEELTEC_LAUNCH_PACKAGE:-turn_on_wheeltec_robot}"
WHEELTEC_LAUNCH_FILE="${WHEELTEC_LAUNCH_FILE:-turn_on_wheeltec_robot.launch.py}"
CHASSIS_STARTUP_DELAY_SEC="${CHASSIS_STARTUP_DELAY_SEC:-3}"

SUDO_PASSWORD="${SUDO_PASSWORD:-1234}"

# 与机器人全栈一致:domain 0(见 start_navigation_headless.sh 2026-05-20 注)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

START_CHASSIS=true
USE_RVIZ=false
DRY_RUN=false
THETA_DEGREES=false
declare -a EXTRA_LAUNCH_ARGS=()
declare -a EXECUTOR_EXTRA_ARGS=()

CLEANUP_DONE=false

log() { printf '[%s] %s\n' "$SCRIPT_NAME" "$*"; }

usage() { awk 'NR>1 { if ($0 !~ /^#/) exit; sub(/^# ?/, ""); print }' "$0"; }

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --server-uri)   SERVER_URI="${2:?}"; shift 2; continue ;;
      --server-uri=*) SERVER_URI="${1#*=}" ;;
      --theta-degrees) THETA_DEGREES=true ;;
      --chassis)      START_CHASSIS=true ;;
      --no-chassis)   START_CHASSIS=false ;;
      --use-rviz)     USE_RVIZ=true ;;
      -d|--dry-run)   DRY_RUN=true ;;
      -h|--help)      usage; exit 0 ;;
      --)             shift; EXECUTOR_EXTRA_ARGS+=("$@"); break ;;
      *:=*)           EXTRA_LAUNCH_ARGS+=("$1") ;;
      *)              printf '未知参数: %s\n' "$1" >&2; usage >&2; exit 1 ;;
    esac
    shift
  done
}

publish_zero_cmd_vel() {
  # 栈被杀之前给 STM32 一串零速:它没有 cmd_vel watchdog,最后一帧是什么就
  # 一直执行什么(同 start_navigation_headless.sh 2026-05-23 的教训)。
  command -v ros2 >/dev/null 2>&1 || return 0
  log '关栈前发 0.6s 零速 /cmd_vel'
  timeout 0.6 ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
    >/dev/null 2>&1 || true
}

cleanup() {
  [[ "$CLEANUP_DONE" == true ]] && return
  CLEANUP_DONE=true
  [[ "$DRY_RUN" == true ]] && return
  log '清理:停 waypoint 栈'
  publish_zero_cmd_vel
  pkill -INT -f 'waypoint_nav_bringup.launch' 2>/dev/null || true
  sleep 2
  pkill -9 -f 'waypoint_nav_bringup.launch' 2>/dev/null || true
  # launch 起的子节点(平台上 launch 死后偶有残留)
  for pat in 'david_nav/position_controller' 'david_nav/waypoint_follower' \
             'david_nav/cmd_vel_mux' nav2_planner/planner_server \
             lifecycle_manager_waypoint_nav; do
    pkill -9 -f "$pat" 2>/dev/null || true
  done
  # 驱动(IMU/雷达/loc/底盘)不在这里杀:共用前置,下次跑脚本会查重重启。
}

run_sudo() {
  printf '%s\n' "$SUDO_PASSWORD" | sudo -S -p '' "$@"
}

restart_driver() {
  # 启动前查重:同 pattern 的旧进程先 INT、等退、再 KILL 兜底,然后起新的。
  local name="$1" pattern="$2" log_file="$3" cmd="$4" startup_delay="${5:-2}"
  local pid deadline

  for pid in $(pgrep -f "$pattern" 2>/dev/null); do
    [[ "$pid" == "$$" ]] && continue
    log "$name 已在跑(pid=$pid),杀掉重启"
    kill -INT "$pid" 2>/dev/null || true
  done
  deadline=$((SECONDS + 5))
  while pgrep -f "$pattern" >/dev/null 2>&1 && (( SECONDS < deadline )); do
    sleep 0.3
  done
  pkill -9 -f "$pattern" 2>/dev/null || true

  log "起 $name (log: $log_file)"
  : >"$log_file" || true
  setsid bash -c "$cmd" >>"$log_file" 2>&1 &
  pid=$!
  sleep "$startup_delay"
  if ! kill -0 "$pid" 2>/dev/null; then
    printf '%s 启动后立刻退出,看 %s\n' "$name" "$log_file" >&2
    exit 1
  fi
  log "$name pid=$pid"
}

pin_jetson_clocks() {
  command -v jetson_clocks >/dev/null 2>&1 || return 0
  log '锁频: jetson_clocks'
  run_sudo jetson_clocks || log 'WARN: jetson_clocks 失败(sudo 密码不对?)'
}

handle_signal() { log "收到 $1,退出"; cleanup; exit 130; }

stop_stale_executor() {
  local pid
  for pid in $(pgrep -f 'navigation_executor.py' 2>/dev/null); do
    [[ "$pid" == "$$" ]] && continue
    log "杀掉残留 navigation_executor pid=$pid"
    kill -TERM "$pid" 2>/dev/null || true
  done
}

wait_for_waypoint_follower() {
  local deadline=$((SECONDS + WAYPOINT_NODE_WAIT_SEC))
  log "等 /waypoint_follower 节点(最多 ${WAYPOINT_NODE_WAIT_SEC}s)"
  while (( SECONDS < deadline )); do
    if ! pgrep -f 'waypoint_nav_bringup.launch' >/dev/null 2>&1; then
      printf 'bringup 提前退出,看 /tmp/waypoint_nav_latest.log\n' >&2
      return 1
    fi
    if ros2 node list 2>/dev/null | grep -qE '(^|/)waypoint_follower$'; then
      log '/waypoint_follower 就绪'
      return 0
    fi
    sleep 1
  done
  printf '/waypoint_follower %ss 内未就绪\n' "$WAYPOINT_NODE_WAIT_SEC" >&2
  return 1
}

main() {
  parse_args "$@"

  for p in "$ROS_SETUP" "$INSTALL_SETUP" "$LAUNCH_WRAPPER" "$EXECUTOR_SRC"; do
    [[ -e "$p" ]] || { printf '缺文件: %s\n' "$p" >&2; exit 1; }
  done
  [[ "$START_CHASSIS" == true ]] && \
    { [[ -e "${WHEELTEC_WS_DIR}/install/setup.bash" ]] || \
      { printf '缺 wheeltec workspace: %s\n' "$WHEELTEC_WS_DIR" >&2; exit 1; }; }

  set +u; # shellcheck disable=SC1090
  source "$ROS_SETUP"; source "$INSTALL_SETUP"; set -u

  local -a executor_cmd=(
    python3 "$EXECUTOR_SRC"
    --server-uri "$SERVER_URI"
    --action-name "$ACTION_NAME"
    --reconnect-delay "$RECONNECT_DELAY"
  )
  [[ "$THETA_DEGREES" == true ]] && executor_cmd+=(--theta-degrees)
  (( ${#EXECUTOR_EXTRA_ARGS[@]} > 0 )) && executor_cmd+=("${EXECUTOR_EXTRA_ARGS[@]}")

  local -a bringup_cmd=("$LAUNCH_WRAPPER" "use_rviz:=${USE_RVIZ}")
  (( ${#EXTRA_LAUNCH_ARGS[@]} > 0 )) && bringup_cmd+=("${EXTRA_LAUNCH_ARGS[@]}")

  local chassis_cmd
  chassis_cmd="source '$ROS_SETUP' && source '${WHEELTEC_WS_DIR}/install/setup.bash' \
    && exec ros2 launch '$WHEELTEC_LAUNCH_PACKAGE' '$WHEELTEC_LAUNCH_FILE'"

  log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  log "server: $SERVER_URI"
  log "action: $ACTION_NAME (theta=$([[ "$THETA_DEGREES" == true ]] && echo 度 || echo 弧度))"
  log "chassis: $([[ "$START_CHASSIS" == true ]] && echo "${WHEELTEC_LAUNCH_PACKAGE}/${WHEELTEC_LAUNCH_FILE}(查重杀旧后重启)" || echo '跳过(--no-chassis)')"
  log 'SLAM 侧(IMU/雷达/重定位)本脚本不管,需已在跑'

  if [[ "$DRY_RUN" == true ]]; then
    printf '[dry-run] sudo jetson_clocks\n'
    [[ "$START_CHASSIS" == true ]] && printf '[dry-run] 底盘:  %s\n' "$chassis_cmd"
    printf '[dry-run] BG=1 %s\n' "${bringup_cmd[*]}"
    printf '[dry-run] %s\n' "${executor_cmd[*]}"
    return 0
  fi

  trap 'handle_signal INT' INT
  trap 'handle_signal TERM' TERM
  trap cleanup EXIT

  stop_stale_executor
  pin_jetson_clocks

  if [[ "$START_CHASSIS" == true ]]; then
    restart_driver '底盘(wheeltec)' 'wheeltec_robot' \
      /tmp/waypoint_nav_chassis.log "$chassis_cmd" "$CHASSIS_STARTUP_DELAY_SEC"
  fi

  # launch_waypoint_nav.sh 自带 pre-flight(杀残留 RSP/旧栈)+ 日志 symlink。
  # BG=1 放后台;它内部 nohup 了真正的 launch 进程。
  log "起 waypoint 栈: ${bringup_cmd[*]}"
  BG=1 "${bringup_cmd[@]}"

  wait_for_waypoint_follower

  log '前台起 navigation_executor(v2 坐标协议)。Ctrl+C 全栈退出。'
  "${executor_cmd[@]}"
}

main "$@"
