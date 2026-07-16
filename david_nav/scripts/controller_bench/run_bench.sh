#!/usr/bin/env bash
# controller_bench 单次 run 编排:起 recorder -> 驱动被测 controller -> 收尾归档。
#
# 一次调用 = 一次 run(每次跑前操作员要物理复位机身到起点 + 重给 slam 初始位姿)。
# 重复 N 次拿分布,再用 bench_metrics.py 汇总。
#
# 前置(本脚本不启动,只检查):
#   - david_slam_gt localization 在跑,发 /groundtruth/pose(真值来源):
#       ros2 run david_slam_gt launch_slam_gt.sh localization <MAP_FILE>
#   - wheeltec 底盘驱动在跑(出 /odom,给 slam_gt 的 odom->base)
#   - position_controller 节点在跑(提供 /position_controller/move + set_parameters)
#
# 用法:
#   ./run_bench.sh <RUN_ID> [--battery-v 25.9] [--note "地面A/冷启动"]
#   ./run_bench.sh S1S4_run03 --battery-v 25.9

set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$HOME/ros2_ws"
# 2026-07-08:默认迁出 /tmp(重启即清,丢过一整天 raw CSV)。BENCH_OUT_DIR 可覆盖。
OUT_DIR="${BENCH_OUT_DIR:-$HOME/david_nav_logs/bench}"
BASE_FRAME="${BENCH_BASE_FRAME:-base_footprint}"
MAP_FRAME="${BENCH_MAP_FRAME:-map}"

RUN_ID="${1:-}"
if [[ -z "$RUN_ID" || "$RUN_ID" == --* ]]; then
  # 不给 run_id 时按 <controller>_<时间戳> 自动起名(可整条塞进 .bashrc alias)。
  ctrl="pos"; prev=""
  for a in "$@"; do
    [[ "$prev" == "--controller" ]] && ctrl="$a"
    prev="$a"
  done
  case "$ctrl" in position_controller) ctrl="pos" ;; esac
  RUN_ID="${ctrl}_$(date +%Y%m%d_%H%M%S)"
else
  shift
fi
PASSTHRU=("$@")   # --controller / --battery-v / --note 等透传给 run_bench.py

# ---- source ROS env ----
# set -e 和 -u 都要临时关:ROS 的 setup.bash 在 set -u 下引用未绑定的
# AMENT_TRACE_SETUP_FILES;个别 ament hook 返回非零又会被 set -e 打死。
# ros_env.conf 用 allexport 真正导出 ROS_DOMAIN_ID 给子进程(否则纯 KEY=value 不 export)。
set +eu
[[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
[[ -f "$WS/install/setup.bash" ]] && source "$WS/install/setup.bash"
if [[ -f "$HERE/../../config/ros_env.conf" ]]; then
  set -o allexport; source "$HERE/../../config/ros_env.conf"; set +o allexport
fi
set -eu

mkdir -p "$OUT_DIR"
echo "[bench] RUN_ID=$RUN_ID  OUT=$OUT_DIR  TF ${MAP_FRAME}->${BASE_FRAME}  DOMAIN=${ROS_DOMAIN_ID:-?}"

# ---- 前置检查(缺就告警,不强退)----
NODES="$(ros2 node list 2>/dev/null || true)"
echo "$NODES" | grep -q position_controller || \
  echo "[bench][WARN] 没看到 position_controller 节点,驱动会 REJECTED"
ros2 topic list 2>/dev/null | grep -q '^/groundtruth/pose$' || \
  echo "[bench][WARN] 没看到 /groundtruth/pose,david_slam_gt localization 起了吗?(会退回 map->base TF)"

# ---- 起 recorder(后台)----
python3 "$HERE/record_track.py" \
  --run-id "$RUN_ID" --output-dir "$OUT_DIR" \
  --map-frame "$MAP_FRAME" --base-frame "$BASE_FRAME" &
REC_PID=$!
echo "[bench] recorder pid=$REC_PID"
sleep 1.5   # 让 recorder 订阅就绪 + 收 latched reference

cleanup() {
  if kill -0 "$REC_PID" 2>/dev/null; then
    kill -TERM "$REC_PID" 2>/dev/null || true
    wait "$REC_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

# ---- 驱动(前台,跑完返回)----
python3 "$HERE/run_bench.py" \
  --reference "$HERE/reference_path.yaml" \
  --run-id "$RUN_ID" --output-dir "$OUT_DIR" \
  --map-frame "$MAP_FRAME" --base-frame "$BASE_FRAME" \
  "${PASSTHRU[@]}" || echo "[bench][WARN] run_bench.py 非零退出"

# 先停 recorder,让 track CSV 落定,再出图
if kill -0 "$REC_PID" 2>/dev/null; then
  kill -TERM "$REC_PID" 2>/dev/null || true
  wait "$REC_PID" 2>/dev/null || true
fi

# 自动出图:地图底(活 /map)+ 参考折线 + waypoints + 真值轨迹 -> plot_<run>.png
python3 "$HERE/plot_run.py" --run-id "$RUN_ID" --output-dir "$OUT_DIR" \
  || echo "[bench][WARN] plot_run 出图失败(localization /map 在吗?)"

echo "[bench] 完成。"
echo "  轨迹CSV : $OUT_DIR/track_${RUN_ID}.csv"
echo "  图      : $OUT_DIR/plot_${RUN_ID}.png"
echo "  汇总    : $OUT_DIR/runs.csv"
echo "[bench] 汇总 N 次: python3 $HERE/bench_metrics.py --out /tmp/bench_report.md"
