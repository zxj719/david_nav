#!/usr/bin/env bash
# nav_to_goal.sh —— 直接从当前位姿导航到目标(HEAD waypoint_follower 流程):
#   Nav2 planner 实时规划 (ComputePathToPose) -> 稀疏成 waypoints
#   -> position_controller 逐段盲走直线。
#
# 与 controller_bench 的区别:bench 走固定折线只测 controller;这个用 planner
# **实时算** waypoints,是真正的点到点导航。
#
# 前置(自己先起,全 ROS_DOMAIN_ID=0):
#   - lidar(alias 2)+ wheeltec 底盘(/odom + 吃 /cmd_vel)
#   - david_slam_gt localization:/map(占据图)+ map->odom TF + /groundtruth/pose
#     ros2 run david_slam_gt launch_slam_gt.sh localization <MAP>
#
# 本脚本会自动(若没起)拉起 waypoint_nav 栈(planner + global_costmap +
# waypoint_follower + position_controller + cmd_vel_mux),配置成吃 slam_gt 的 /map,
# 然后发目标并阻塞等完成。栈起来后会一直留着,可反复发目标。
#
# 用法:
#   nav_to_goal.sh <x> <y> [yaw度]        例: nav_to_goal.sh 3.62 1.8 90
#   RVIZ=1 nav_to_goal.sh 3.62 1.8 90     顺便开 RViz 看 plan/waypoints
#
# 日志: /tmp/waypoint_nav_latest.log(栈)。

set -eo pipefail

X="${1:?用法: nav_to_goal.sh <x> <y> [yaw度]}"
Y="${2:?用法: nav_to_goal.sh <x> <y> [yaw度]}"
YAW_DEG="${3:-0}"
RVIZ="${RVIZ:-0}"

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 域唯一真源(现在是 0)
set +eu
source "${DIR}/../config/load_ros_env.sh"
source /opt/ros/humble/setup.bash
source "${HOME}/ros2_ws/install/setup.bash"
set -eu
echo "[nav_to_goal] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

have_action() { ros2 action list 2>/dev/null | grep -q '^/waypoint_follower/navigate$'; }

# ---- 前置检查(缺就告警,map/odom 缺会规划失败) ----
ros2 topic list 2>/dev/null | grep -q '^/map$'  || echo "[nav_to_goal][WARN] 没 /map(slam_gt localization 起了吗?)"
ros2 topic list 2>/dev/null | grep -q '^/odom$' || echo "[nav_to_goal][WARN] 没 /odom(wheeltec 起了吗?)"

# ---- 栈没起就拉起(配 slam_gt:map_topic:=/map,不起 nav RSP) ----
if have_action; then
  echo "[nav_to_goal] waypoint_nav 栈已在跑,直接发目标。"
else
  echo "[nav_to_goal] 拉起 waypoint_nav 栈(slam_gt 配置,后台)..."
  BG=1 "${DIR}/launch_waypoint_nav.sh" \
      publish_robot_state:=false \
      map_topic:=/map \
      use_rviz:="$([ "$RVIZ" = 1 ] && echo true || echo false)"
  echo -n "[nav_to_goal] 等 planner + waypoint_follower 就绪"
  for i in $(seq 1 45); do
    have_action && break
    echo -n "."; sleep 1
  done
  echo
  if ! have_action; then
    echo "[nav_to_goal][ERROR] 45s 内 /waypoint_follower/navigate 没出现。看 /tmp/waypoint_nav_latest.log" >&2
    exit 1
  fi
  # 再给 global_costmap 一点时间接 /map 并 activate
  sleep 3
fi

# ---- yaw度 -> 四元数 ----
read -r Z W < <(python3 -c "import math; r=math.radians($YAW_DEG)/2; print(math.sin(r), math.cos(r))")
echo "[nav_to_goal] 目标: map ($X, $Y, ${YAW_DEG}°)  quat z=$Z w=$W"

# ---- 发目标(action,阻塞等完成,带反馈) ----
ros2 action send_goal -f /waypoint_follower/navigate nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {z: $Z, w: $W}}}}"

echo "[nav_to_goal] 完成。栈仍在后台,可再发:nav_to_goal.sh <x> <y> [yaw]"
