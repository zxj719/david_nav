#!/usr/bin/env bash
# ──────────────────────────────────────────────────────────────────────────
# David 全栈 ROS 环境加载器 —— 被各启动脚本 source。
#
# 真源是同目录的 ros_env.conf（纯 KEY=value，systemd 也用 EnvironmentFile 读它）。
# 改 ROS_DOMAIN_ID 只动 ros_env.conf 一行 —— 这里和各脚本都不要再写死数字。
#
# 优先级：外部已 export 的 ROS_DOMAIN_ID  >  ros_env.conf  >  兜底 1
#         （ad-hoc 调试时 `export ROS_DOMAIN_ID=N` 仍可临时覆盖单次运行。）
#
# 用法：  source "<…/david_nav/config>/load_ros_env.sh"
# nounset(set -u) 安全。
# ──────────────────────────────────────────────────────────────────────────
__david_ros_env_conf="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)/ros_env.conf"
if [[ -z "${ROS_DOMAIN_ID:-}" && -f "${__david_ros_env_conf}" ]]; then
  set -o allexport
  # shellcheck disable=SC1090
  source "${__david_ros_env_conf}"
  set +o allexport
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
unset __david_ros_env_conf
