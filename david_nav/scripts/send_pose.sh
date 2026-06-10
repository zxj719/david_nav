#!/usr/bin/env bash
# 直接给 waypoint_follower 发 map 系导航目标(等价于 RViz 的 2D Nav Goal)。
# 用法: send_pose.sh <x> <y> [yaw度,默认0]
#   例: send_pose.sh 9.52 -8.82 0
# 需要 waypoint_nav 栈在跑(launch_waypoint_nav.sh)。
set -e
if [[ $# -lt 2 ]]; then
  echo "用法: $0 <x> <y> [yaw度]" >&2
  exit 1
fi
X=$1; Y=$2; YAW_DEG=${3:-0}
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${HOME}/ros2_ws/install/setup.bash"
read -r Z W <<< "$(python3 -c "import math; r=math.radians($YAW_DEG)/2; print(math.sin(r), math.cos(r))")"
echo "发布目标: map ($X, $Y, ${YAW_DEG}°)  [quat z=$Z w=$W]" >&2
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: $X, y: $Y}, orientation: {z: $Z, w: $W}}}"
