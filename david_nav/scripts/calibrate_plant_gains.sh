#!/usr/bin/env bash
# 滚动态 wz/v 执行增益标定 —— 一键入口（alias 9）。
#
# 前置：alias 2 (lidar) → 5 (wheeltec) → 3 (lightning) → 手动给初始位姿。
# 真值 = lightning（2026-07-17 定稿）：/lightning/odometry (激光里程计,主源)
#   > TF odom→base > /lightning/loc/odometry (兜底,偶发重定位跳变已有缝合)。
#   slam_gt 仅在显式传 --truth-topic /groundtruth/pose 时参与。
# 禁止：nav2 / cmd_vel_separator / david_move / 键盘遥控 —— 都会抢 /cmd_vel，
#       preflight 会检测并要求确认。
#
# 用法：
#   9                     # 全流程（v → wzstat → wzroll，组间确认）
#   9 --dry-run           # 只验证话题/打印计划，不动车
#   9 --sign-check        # 低速脉冲验证 vx/wz 符号
#   9 --tests wzroll --repeats 5
#   9 --analyze /tmp/david_nav_logs/calibrate_plant_gains_XXX.csv
#
# 所有参数透传给 calibrate_plant_gains.py。

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
NAV_DIR="$(cd "${SCRIPT_DIR}/.." >/dev/null 2>&1 && pwd)"

# 域真源：config/ros_env.conf（外部已 export 则尊重外部值）
# shellcheck disable=SC1091
source "${NAV_DIR}/config/load_ros_env.sh"

# ROS 环境（slam_gt 的 set -u/source 坑：这里不开 -u，且容错 source）
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f "${HOME}/ros2_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${HOME}/ros2_ws/install/setup.bash"
fi

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"

# --analyze / --help / --dry-run 不需要在线 preflight
skip_preflight=false
for a in "$@"; do
  case "$a" in
    --analyze|--analyze=*|-h|--help) skip_preflight=true ;;
  esac
done

if [[ "$skip_preflight" != true ]]; then
  echo "── preflight ──────────────────────────────────────────────"
  topics="$(timeout 10 ros2 topic list 2>/dev/null || true)"

  if ! grep -qx "/odom" <<<"$topics"; then
    echo "✗ /odom 不存在 —— 先启动 alias 5 (wheeltec)"; exit 1
  fi
  echo "✓ /odom 在"

  truth_found=""
  for t in /lightning/odometry /lightning/loc/odometry; do
    if grep -qx "$t" <<<"$topics"; then
      truth_found="${truth_found} $t"
    fi
  done
  if [[ -z "$truth_found" ]]; then
    echo "✗ 无 lightning 真值话题 —— 先启动 alias 3 (lightning) 并手动给初始位姿"
    exit 1
  fi
  echo "✓ 真值候选:${truth_found}（主源 /lightning/odometry，脚本按新鲜度裁决）"
  if ! grep -qx "/lightning/odometry" <<<"$topics"; then
    echo "⚠ 主源 /lightning/odometry 不在，只有 loc/odometry（有重定位跳变风险，已有缝合兜底）"
  fi

  # /cmd_vel 必须有订阅者（wheeltec 在收）
  subs="$(timeout 5 ros2 topic info /cmd_vel 2>/dev/null | awk '/Subscription count/{print $3}')"
  if [[ -z "$subs" || "$subs" == "0" ]]; then
    echo "⚠ /cmd_vel 当前无订阅者 —— wheeltec 可能没起好（车不会动）"
  else
    echo "✓ /cmd_vel 订阅者: ${subs}"
  fi

  # 抢 /cmd_vel 的进程检测（本标定要求独占 /cmd_vel）
  fighters="$(pgrep -af 'controller_server|cmd_vel_separator|david_move|teleop_twist|velocity_smoother' 2>/dev/null || true)"
  if [[ -n "$fighters" ]]; then
    echo "⚠ 检测到会抢 /cmd_vel 的进程："
    echo "$fighters" | sed 's/^/    /'
    read -r -p "  继续会导致指令混叠、标定数据作废。仍要继续? [y/N] " ans
    [[ "$ans" == "y" || "$ans" == "Y" ]] || exit 1
  else
    echo "✓ 无 /cmd_vel 竞争进程 (nav2/separator/david_move/teleop)"
  fi
  echo "───────────────────────────────────────────────────────────"
fi

exec python3 "${SCRIPT_DIR}/calibrate_plant_gains.py" "$@"
