#!/usr/bin/env bash
# save_map.sh —— mapping 跑着时调用，序列化 slam_toolbox 地图。
#   用法: save_map.sh [MAP_FILE]   (默认 maps/david_map, 无扩展名)
#   产出: <MAP_FILE>.posegraph + <MAP_FILE>.data  (localization 模式加载用)
#         同时存一份 <MAP_FILE>.pgm/.yaml 占据栅格(给 RViz / nav2 静态层看)
set -eu

# 必须和 slam_toolbox 同域(默认 0),否则 service call 找不到 /slam_toolbox/*。
export ROS_DOMAIN_ID="${SLAM_GT_DOMAIN:-0}"

MAP_FILE="${1:-$HOME/ros2_ws/src/david_slam_gt/maps/david_map}"
mkdir -p "$(dirname "$MAP_FILE")"
# ROS 的 setup.bash 引用未定义的 AMENT_TRACE_SETUP_FILES，与 set -u 冲突，临时关掉。
set +u
source /opt/ros/humble/setup.bash
set -u

echo "[save_map] 序列化 pose-graph -> ${MAP_FILE}.posegraph/.data"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '${MAP_FILE}'}"

echo "[save_map] 另存占据栅格 -> ${MAP_FILE}.pgm/.yaml"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '${MAP_FILE}'}}" || \
  echo "[save_map] (save_map 可选, 失败不影响 localization 用 .posegraph)"

echo "[save_map] done. localization 用: launch_slam_gt.sh localization ${MAP_FILE}"
