#!/usr/bin/env bash
# launch_slam_gt.sh —— 一键起 slam_gt 真值/定位管路。
#
#   用法:
#     launch_slam_gt.sh mapping                      # 建图
#     launch_slam_gt.sh localization [MAP_FILE]      # 定位(默认 maps/david_map)
#   选项(环境变量):
#     USE_IMU=0   不起 IMU(alias 1)；默认起(用户要求, 但 slam_toolbox 不吃 IMU)
#     USE_LIDAR=0 不起 LSLIDAR(alias 2)；默认起
#
#   本脚本【不】起 wheeltec 底盘和遥控/导航：
#     - 建图: 你需另起 wheeltec(出 /odom) + 键盘遥控，手动把机器人开一圈。
#     - 定位调参: 另起 wheeltec + nav2(use_rsp:=false enable_scan_bridge:=false,
#       且【不要】起 lightning)。
#
#   日志: /tmp/slam_gt_latest.log (symlink, 直接 tail -F)
set -u

MODE="${1:-mapping}"
MAP_FILE="${2:-$HOME/ros2_ws/src/david_slam_gt/maps/david_map}"
LIGHTNING_WS="$HOME/lightning_ws"
ROS2_WS="$HOME/ros2_ws"
USE_IMU="${USE_IMU:-1}"
USE_LIDAR="${USE_LIDAR:-1}"
SUDO_PW="${SUDO_PW:-1234}"

TS="$(date +%Y%m%d_%H%M%S)"
LOG="/tmp/slam_gt_${MODE}_${TS}.log"
ln -sfn "$LOG" /tmp/slam_gt_latest.log
echo "[launch_slam_gt] mode=$MODE  log -> /tmp/slam_gt_latest.log ($LOG)" | tee "$LOG"

PIDS=()
cleanup() { echo "[launch_slam_gt] stopping..."; for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done; }
trap cleanup INT TERM EXIT

# —— alias 1: wit IMU 驱动 ——
if [ "$USE_IMU" = "1" ]; then
  echo "[launch_slam_gt] starting IMU (alias 1)..." | tee -a "$LOG"
  ( cd "$LIGHTNING_WS" && set +u && source ./install/setup.bash && set -u \
    && exec ros2 launch wit_imu_ros2 wit_imu.launch.py ) >>"$LOG" 2>&1 &
  PIDS+=($!)
fi

# —— alias 2: LSLIDAR C16 驱动 (含 eno1 网口配置) ——
if [ "$USE_LIDAR" = "1" ]; then
  echo "[launch_slam_gt] starting LSLIDAR C16 (alias 2)..." | tee -a "$LOG"
  ( cd "$LIGHTNING_WS" && set +u && source ./install/setup.bash && set -u \
    && echo "$SUDO_PW" | sudo -S ifconfig eno1 192.168.1.102 \
    && exec ros2 launch lslidar_driver lslidar_c16_launch.py ) >>"$LOG" 2>&1 &
  PIDS+=($!)
  sleep 3
  # lslidar 驱动 launch 写死会弹一个自带的 rviz(Fixed Frame 起步即报错 -> 黑屏，
  # 且只显示 /c16 原始点云，与 SLAM 无关)。精准杀掉它(只匹配 lslidar_c16.rviz，
  # 不会误伤 slam_gt 自己的 rviz),改由 slam_gt launch 起正确的可视化。
  pkill -f "rviz2.*lslidar_c16.rviz" 2>/dev/null || true
fi

# —— slam_gt 处理链 (ros2_ws) ——
# ROS 的 setup.bash 引用未定义的 AMENT_TRACE_SETUP_FILES，与 set -u 冲突，临时关掉。
set +u
source /opt/ros/humble/setup.bash
source "$ROS2_WS/install/setup.bash"
set -u

if [ "$MODE" = "mapping" ]; then
  echo "[launch_slam_gt] >>> MAPPING. 另起 wheeltec + 遥控，把机器人开一圈;" | tee -a "$LOG"
  echo "    建完用: ros2 run david_slam_gt save_map.sh $MAP_FILE" | tee -a "$LOG"
  ros2 launch david_slam_gt slam_gt_mapping.launch.py >>"$LOG" 2>&1 &
  PIDS+=($!)
elif [ "$MODE" = "localization" ]; then
  echo "[launch_slam_gt] >>> LOCALIZATION on map: $MAP_FILE" | tee -a "$LOG"
  echo "    nav2 请用 use_rsp:=false enable_scan_bridge:=false, 且不要起 lightning" | tee -a "$LOG"
  ros2 launch david_slam_gt slam_gt_localization.launch.py map_file:="$MAP_FILE" >>"$LOG" 2>&1 &
  PIDS+=($!)
else
  echo "[launch_slam_gt] 未知 mode '$MODE' (mapping|localization)"; exit 1
fi

echo "[launch_slam_gt] 全部已起。Ctrl-C 停止。tail -F /tmp/slam_gt_latest.log"
wait
