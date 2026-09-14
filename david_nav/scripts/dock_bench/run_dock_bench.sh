#!/usr/bin/env bash
# dock_bench 一键入口（alias 10）：起 recorder -> 跑 N 轮入位 -> 收尾出图。
#
# 前置（本脚本不启动，只检查）：
#   alias 3  lightning 定位 + 雷达 + IMU，**且已在 rviz 里给过初始位姿**
#   alias 0  nav2 精确参数 + dock_navigator + wheeltec 底盘（底盘 2026-09-08 已并入 0）
#
# 用法:
#   ./run_dock_bench.sh                       # 用 dock_case.yaml 里的 runs
#   ./run_dock_bench.sh --runs 5              # 冒烟
#   ./run_dock_bench.sh --runs 20 --battery-v 25.9 --note "地面A"

set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --help 要在起 recorder 之前拦下来，否则问个用法都会留下一个空的 track CSV
case "${1:-}" in
  -h|--help)
    sed -n '2,12p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    echo
    echo "透传给 run_dock_bench.py 的参数:"
    exec python3 "$HERE/run_dock_bench.py" --help
    ;;
esac
WS="$HOME/ros2_ws"
OUT_DIR="${DOCK_BENCH_OUT_DIR:-$HOME/david_nav_logs/dock_bench}"
BASE_FRAME="${BENCH_BASE_FRAME:-base_footprint}"
MAP_FRAME="${BENCH_MAP_FRAME:-map}"
SESSION="${DOCK_BENCH_SESSION:-dock_$(date +%Y%m%d_%H%M%S)}"
DOCK_NODE="${DOCK_BENCH_NODE:-dock_navigator}"

# ---- source ROS env（与 controller_bench/run_bench.sh 同款处理）----
# set -e/-u 都要临时关：ROS 的 setup.bash 在 set -u 下引用未绑定变量，
# 个别 ament hook 又会返回非零被 set -e 打死。
set +eu
[[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
[[ -f "$WS/install/setup.bash" ]] && source "$WS/install/setup.bash"
if [[ -f "$HERE/../../config/ros_env.conf" ]]; then
  set -o allexport; source "$HERE/../../config/ros_env.conf"; set +o allexport
fi
set -eu

# 陈旧 pyc 坑：controller_bench 踩过，改完 py 不清会测到旧逻辑
rm -rf "$HERE/__pycache__" "$HERE/../controller_bench/__pycache__" 2>/dev/null || true

mkdir -p "$OUT_DIR"
echo "[dock] SESSION=$SESSION  OUT=$OUT_DIR  TF ${MAP_FRAME}->${BASE_FRAME}  DOMAIN=${ROS_DOMAIN_ID:-?}"

# ---- 前置检查（缺就告警，不强退）----
NODES="$(ros2 node list 2>/dev/null || true)"
TOPICS="$(ros2 topic list 2>/dev/null || true)"
echo "$NODES" | grep -q "$DOCK_NODE" || \
  echo "[dock][WARN] 没看到 $DOCK_NODE 节点 —— alias 0 起了吗？（没有它 /dock_goal 没人消费）"
echo "$NODES" | grep -q 'controller_server' || \
  echo "[dock][WARN] 没看到 controller_server —— nav2 起了吗？"
echo "$NODES" | grep -q 'wheeltec_robot' || \
  echo "[dock][WARN] 没看到 wheeltec_robot —— 底盘没起，车不会动"
echo "$TOPICS" | grep -q '^/lightning/loc/odometry$' || \
  echo "[dock][WARN] 没看到 /lightning/loc/odometry —— alias 3 起了吗？"
# /scan 没发布者 = 阶段2 没有动态避障，只剩静态地图。空场地跑没问题，但要知情。
if ! ros2 topic info /scan 2>/dev/null | grep -q 'Publisher count: [1-9]'; then
  echo "[dock][WARN] /scan 没有发布者 —— 本轮测试**没有动态避障**（scan_bridge 未开）。"
  echo "[dock][WARN] 确保入位路径上没有人和临时障碍物。"
fi

# ---- 起 recorder（后台）----
# --rosout-node 让它顺带把 dock_navigator 的判词流也落一份（编排器自己也会存一份，
# 两份互为备份）；--extra-action-status 收 Spin 动作，用于阶段边界的交叉校验。
python3 "$HERE/../controller_bench/record_track.py" \
  --run-id "$SESSION" --output-dir "$OUT_DIR" \
  --map-frame "$MAP_FRAME" --base-frame "$BASE_FRAME" \
  --extra-action-status /spin/_action/status \
  --rosout-node "$DOCK_NODE" &
REC_PID=$!
echo "[dock] recorder pid=$REC_PID"
sleep 1.5

cleanup() {
  if kill -0 "$REC_PID" 2>/dev/null; then
    kill -TERM "$REC_PID" 2>/dev/null || true
    wait "$REC_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

# ---- 跑 N 轮（前台）----
python3 "$HERE/run_dock_bench.py" \
  --case "$HERE/dock_case.yaml" \
  --session "$SESSION" --output-dir "$OUT_DIR" \
  --map-frame "$MAP_FRAME" --base-frame "$BASE_FRAME" \
  --dock-node "$DOCK_NODE" \
  "$@" || echo "[dock][WARN] run_dock_bench.py 非零退出"

# 先停 recorder 让 track CSV 落定，再出图
if kill -0 "$REC_PID" 2>/dev/null; then
  kill -TERM "$REC_PID" 2>/dev/null || true
  wait "$REC_PID" 2>/dev/null || true
fi

python3 "$HERE/plot_dock_run.py" --session "$SESSION" --track-dir "$OUT_DIR" \
  --case "$HERE/dock_case.yaml" \
  || echo "[dock][WARN] 出图失败（/map 在吗？）"

echo "[dock] 完成。"
echo "  轨迹CSV : $OUT_DIR/track_${SESSION}.csv"
echo "  判词流  : $OUT_DIR/rosout_${SESSION}.csv"
echo "  汇总行  : $OUT_DIR/runs_dock.csv"
echo "  图      : $OUT_DIR/plot_${SESSION}_i*.png"
echo "[dock] 出报告: python3 $HERE/dock_metrics.py --session $SESSION --out /tmp/dock_report.md"
