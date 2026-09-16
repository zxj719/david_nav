#!/usr/bin/env bash
# Headless waypoint 导航(nav 协议 v1,2026-07-22 回退定稿)。
#
# start_navigation_headless.sh 的 waypoint 版:
#   * bringup 用 scripts/launch_waypoint_nav.sh(planner-only Nav2 +
#     position_controller + waypoint_follower,pre-flight 清理/日志都在那边)
#   * 前台跑 navigation_executor.py,WebSocket client 连 server,
#     navigate_to 带 target_id(POI id),由执行器查表解析坐标;
#     v2 的 target_pose {x,y,theta} 已废弃,消息里残留该字段会被忽略
#   * action 指向 /waypoint_follower/navigate(NavigateToPose 同类型,
#     不是 bt_navigator)
#
# 本脚本只管导航侧:jetson_clocks 锁频 → wheeltec 底盘(默认带起,启动前
# 查重、旧进程杀掉再起)→ waypoint 导航栈 → TF 守卫 → 前台执行器。
#
# SLAM 侧(IMU wit_imu / 雷达 lslidar c16 / 重定位 run_loc_online)由 SLAM
# 那边的流程负责,本脚本不碰 —— 跑导航前确保 Lightning loc 已在出
# /lightning/grid_map + map->odom->base TF(同一 ROS_DOMAIN_ID)。
#
# TF 守卫(2026-06-11):导航期间每 TF_GUARD_INTERVAL_SEC(默认 3s)扫一遍
# robot_state_publisher / joint_state_publisher / static_transform_publisher,
# 凡祖先链里不含 waypoint_nav_bringup 的一律 kill —— 上肢/lerobot 录制流程
# 的全身 URDF RSP 会给 base_link 另立父节点,TF 裂成两棵树,执行器报
# ConnectivityException(2026-06-09 与 2026-06-11 15:17 两次实锅)。
# 导航和上肢任务互斥;上肢那边下次启动会自己再拉起这些发布者。
#
# 用法:
#   start_waypoint_nav_headless.sh [options] [arg:=val …] [-- executor-args…]
#     --server-uri URI     WebSocket server(默认 $NAV_WS_URL,否则 ws://127.0.0.1:8101)
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

# nav WS v1:server 监听根路径,无 /ws/navigation/executor 子路径。
SERVER_URI="${NAV_SERVER_URI:-${NAV_WS_URL:-ws://127.0.0.1:8101}}"
ACTION_NAME="${NAV_ACTION_NAME:-/waypoint_follower/navigate}"
RECONNECT_DELAY="${NAV_RECONNECT_DELAY:-3.0}"
WAYPOINT_NODE_WAIT_SEC="${WAYPOINT_NODE_WAIT_SEC:-60}"

WHEELTEC_WS_DIR="${WHEELTEC_WS_DIR:-${HOME}/wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2}"
WHEELTEC_LAUNCH_PACKAGE="${WHEELTEC_LAUNCH_PACKAGE:-turn_on_wheeltec_robot}"
WHEELTEC_LAUNCH_FILE="${WHEELTEC_LAUNCH_FILE:-turn_on_wheeltec_robot.launch.py}"
CHASSIS_STARTUP_DELAY_SEC="${CHASSIS_STARTUP_DELAY_SEC:-3}"

SUDO_PASSWORD="${SUDO_PASSWORD:-1234}"
TF_GUARD_INTERVAL_SEC="${TF_GUARD_INTERVAL_SEC:-3}"

# 与机器人全栈一致:domain 0(见 start_navigation_headless.sh 2026-05-20 注)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

START_CHASSIS=true
USE_RVIZ=false
DRY_RUN=false
declare -a EXTRA_LAUNCH_ARGS=()
declare -a EXECUTOR_EXTRA_ARGS=()

CLEANUP_DONE=false
TF_GUARD_PID=""

log() { printf '[%s] %s\n' "$SCRIPT_NAME" "$*"; }

usage() { awk 'NR>1 { if ($0 !~ /^#/) exit; sub(/^# ?/, ""); print }' "$0"; }

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --server-uri)   SERVER_URI="${2:?}"; shift 2; continue ;;
      --server-uri=*) SERVER_URI="${1#*=}" ;;
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
  stop_tf_guard
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

is_our_stack_process() {
  # 沿祖先链往上找 waypoint_nav_bringup;找到 → 是本导航栈自己的节点。
  local pid="$1" depth=0
  while [[ -n "$pid" && "$pid" != "0" && "$pid" != "1" && $depth -lt 12 ]]; do
    if grep -aq 'waypoint_nav_bringup' "/proc/$pid/cmdline" 2>/dev/null; then
      return 0
    fi
    pid="$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')"
    depth=$((depth + 1))
  done
  return 1
}

kill_conflicting_tf_publishers() {
  # 2026-06-12 改造:base_link 的父现在由 SLAM 的 odom 唯一发布。机械臂把
  # 上半身挂 base_link 下,天然同树,其全身 RSP / joint_state_publisher /
  # 相机静态 tf **一律不杀**。只清两类真正打架的:
  #   1. 残留底盘 RSP(URDF 含 chassis_root 且不是本栈子进程)—— 重复发
  #      base_link 子树,来自上一次没退干净的导航;
  #   2. 任何把 base_link 当 child 的静态 tf —— 给 base_link 另立父节点,
  #      跟 SLAM 的 odom→base_link 双父冲突(旧式 base→base_link、已废弃的
  #      base→world/world→base_link 残留),必杀。
  local pid pf cl
  for pid in $(pgrep -f robot_state_publisher 2>/dev/null); do
    [[ "$pid" == "$$" ]] && continue
    is_our_stack_process "$pid" && continue
    pf="$(tr '\0' '\n' < "/proc/$pid/cmdline" 2>/dev/null | grep -A1 -- '--params-file' | tail -1)"
    if [[ -n "$pf" && -r "$pf" ]] && grep -q chassis_root "$pf" 2>/dev/null; then
      log "WARN: 残留底盘 RSP pid=$pid(非本栈)→ kill"
      kill -9 "$pid" 2>/dev/null || true
    fi
  done
  for pid in $(pgrep -f static_transform_publisher 2>/dev/null); do
    [[ "$pid" == "$$" ]] && continue
    is_our_stack_process "$pid" && continue
    cl="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)"
    case "$cl" in
      *base_to_base_link_tf*|*base_to_world_tf*|*world_to_base_link_tf*|*"--child-frame-id base_link"*)
        log "WARN: base_link 被另立父节点的静态 tf pid=$pid(与 SLAM odom→base_link 冲突)→ kill"
        kill -9 "$pid" 2>/dev/null || true ;;
    esac
  done
}

start_tf_guard() {
  # 常驻守卫:只盯"会打架"的发布者(残留底盘 RSP / 给 base_link 另立父的 tf);
  # base_link 的父由 SLAM 唯一发,机械臂树挂其下,守卫不碰机械臂。
  log "起 TF 守卫(每 ${TF_GUARD_INTERVAL_SEC}s 清残留底盘 RSP / base_link 另立父的 tf;机械臂 TF 不动)"
  (
    while true; do
      sleep "$TF_GUARD_INTERVAL_SEC"
      kill_conflicting_tf_publishers
    done
  ) &
  TF_GUARD_PID=$!
}

stop_tf_guard() {
  if [[ -n "$TF_GUARD_PID" ]] && kill -0 "$TF_GUARD_PID" 2>/dev/null; then
    kill "$TF_GUARD_PID" 2>/dev/null || true
  fi
  TF_GUARD_PID=""
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
  (( ${#EXECUTOR_EXTRA_ARGS[@]} > 0 )) && executor_cmd+=("${EXECUTOR_EXTRA_ARGS[@]}")

  local -a bringup_cmd=("$LAUNCH_WRAPPER" "use_rviz:=${USE_RVIZ}")
  (( ${#EXTRA_LAUNCH_ARGS[@]} > 0 )) && bringup_cmd+=("${EXTRA_LAUNCH_ARGS[@]}")

  local chassis_cmd
  chassis_cmd="source '$ROS_SETUP' && source '${WHEELTEC_WS_DIR}/install/setup.bash' \
    && exec ros2 launch '$WHEELTEC_LAUNCH_PACKAGE' '$WHEELTEC_LAUNCH_FILE'"

  log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  log "server: $SERVER_URI"
  log "action: $ACTION_NAME"
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

  # 导航前先扫一遍会打架的 TF 发布者,然后守卫常驻(导航中途出现也兜得住)。
  kill_conflicting_tf_publishers
  start_tf_guard

  log '前台起 navigation_executor(nav WS v1 协议)。Ctrl+C 全栈退出。'
  "${executor_cmd[@]}"
}

main "$@"
