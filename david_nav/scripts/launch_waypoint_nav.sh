#!/usr/bin/env bash
# One-shot bringup for the planner-only Nav2 + position-controller waypoint
# follower, with RViz and logging. Wraps:
#     ros2 launch david_nav waypoint_nav_bringup.launch.py …
# and tees all output to a timestamped log, keeping /tmp/waypoint_nav_latest.log
# pointing at the latest run (tail -F it from anywhere).
#
# Prereqs (start FIRST, same ROS_DOMAIN_ID): Lightning SLAM/loc
# (/lightning/grid_map + map->odom TF) and the wheeltec chassis driver
# (/odom + odom->base_footprint TF, consumes /cmd_vel).
#
# Usage:
#   launch_waypoint_nav.sh [arg:=val …]        # args forwarded to ros2 launch
#   BG=1 launch_waypoint_nav.sh                 # background, return to shell
#
# Then in RViz: "2D Pose Estimate" to seed Lightning loc (if needed), wait for
# the global costmap, then "2D Nav Goal" — the waypoint_follower plans,
# sparsifies, and drives each segment. The dense plan + waypoint markers show
# up as the "Global Plan (dense)" and "Waypoints" displays.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

LOG_DIR="${LOG_DIR:-/tmp/david_nav_logs}"
LOG_LATEST="${LOG_LATEST:-/tmp/waypoint_nav_latest.log}"
BG="${BG:-0}"
# domain 唯一真源 config/ros_env.conf（必须与 Lightning + chassis + arm SDK 一致；
# mismatch 是经典的 "RViz 啥都不显示" 坑，见 CLAUDE.md §5）。
# 外部已 export 的 ROS_DOMAIN_ID 仍优先（单次运行覆盖）。
source "${SCRIPT_DIR}/../config/load_ros_env.sh"

mkdir -p "$LOG_DIR"
stamp="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/waypoint_nav_${stamp}.log"

# Pre-flight (2026-06-12 改造:base_link 父由 SLAM 唯一发布,不再无差别杀):
# SLAM 现在直接发 map→odom→base_link;机械臂全身 RSP 把上半身挂 base_link 下,
# 天然同一棵树 —— 机械臂的 RSP / joint_state_publisher / 相机静态 tf 全部
# **保留不杀**。只清真正会打架的:
#   1. 上一次没退干净的导航栈(重名节点、旧 planner 等);
#   2. 残留的**底盘 URDF** RSP(URDF 含 chassis_root 的才是导航自己的,杀;
#      机械臂全身 URDF 没有 chassis_root,留);
#   3. 任何把 base_link 当 child 的静态 tf(base_link 的父现在只能由 SLAM
#      的 odom 发;旧式 base→base_link / 已废弃的 base→world / world→base_link
#      残留都会给 base_link 双父 → 裂树,一律清掉)。
SELF_PID=$$
pkill -INT -f 'waypoint_nav_bringup.launch' 2>/dev/null && sleep 2
pkill -INT -f '2d_david_nav2_bringup.launch' 2>/dev/null && sleep 2
for pat in 'david_nav/position_controller' 'david_nav/waypoint_follower' \
           'david_nav/cmd_vel_mux' nav2_planner/planner_server \
           lifecycle_manager_waypoint_nav; do
  for pid in $(pgrep -f "$pat" 2>/dev/null); do
    [[ "$pid" == "$SELF_PID" ]] && continue
    echo "[launch_waypoint_nav]   killed stale '$pat' pid=$pid" >&2
    kill -9 "$pid" 2>/dev/null || true
  done
done
# 2. 残留底盘 RSP:从 cmdline 取 --params-file,内容含 chassis_root 才杀
for pid in $(pgrep -f robot_state_publisher 2>/dev/null); do
  [[ "$pid" == "$SELF_PID" ]] && continue
  pf=$(tr '\0' '\n' < "/proc/$pid/cmdline" 2>/dev/null | grep -A1 -- '--params-file' | tail -1)
  if [[ -n "$pf" && -r "$pf" ]] && grep -q chassis_root "$pf" 2>/dev/null; then
    echo "[launch_waypoint_nav]   killed stale chassis RSP pid=$pid (params=$pf)" >&2
    kill -9 "$pid" 2>/dev/null || true
  fi
done
# 3. 任何把 base_link 当 child 的静态 tf —— base_link 父只能由 SLAM odom 发,
#    其余(旧式 base→base_link、已废弃的 base→world/world→base_link)都会双父裂树
for pid in $(pgrep -f static_transform_publisher 2>/dev/null); do
  [[ "$pid" == "$SELF_PID" ]] && continue
  cl=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null)
  case "$cl" in
    *base_to_base_link_tf*|*base_to_world_tf*|*world_to_base_link_tf*)
      echo "[launch_waypoint_nav]   killed stale nav bridge tf pid=$pid" >&2
      kill -9 "$pid" 2>/dev/null || true ;;
    *"--child-frame-id base_link"*)
      echo "[launch_waypoint_nav]   killed stale base_link-parent tf pid=$pid" >&2
      kill -9 "$pid" 2>/dev/null || true ;;
  esac
done
sleep 1

ln -sfn "$LOG_FILE" "$LOG_LATEST"
echo "[launch_waypoint_nav] ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >&2
echo "[launch_waypoint_nav] log: $LOG_FILE" >&2
echo "[launch_waypoint_nav] symlink: $LOG_LATEST" >&2

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${HOME}/ros2_ws/install/setup.bash"

# 杀完进程后重置 ros2 daemon 的节点图缓存:死掉的 DDS 参与者会在缓存里残留
# (2026-06-09 现象:进程只剩 1 个 RSP,node list 里却有 2 个)。daemon 在下
# 一条 ros2 命令时自动重启,无副作用。
ros2 daemon stop >/dev/null 2>&1 || true

CMD=(ros2 launch david_nav waypoint_nav_bringup.launch.py "$@")
echo "[launch_waypoint_nav] cmd: ${CMD[*]}" >&2

if [[ "$BG" == "1" ]]; then
  nohup "${CMD[@]}" > "$LOG_FILE" 2>&1 &
  echo "[launch_waypoint_nav] PID=$!" >&2
else
  bash -o pipefail -c '"$@" 2>&1 | tee "$0"' "$LOG_FILE" "${CMD[@]}" || true
fi
