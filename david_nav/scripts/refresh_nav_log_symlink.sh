#!/usr/bin/env bash
# Find the most recently-modified bringup.log under ~/rosbags/*/logs/ (created
# by navigate_semantic_pose_with_debug_bag.sh) and (re)point /tmp/nav2_latest.log
# at it. Idempotent. Prints the resolved path.
#
# Run this right after starting navigate_semantic_pose_with_debug_bag.sh so
# the assistant can read /tmp/nav2_latest.log without remembering bag names.

set -euo pipefail

BAG_ROOT="${BAG_ROOT:-${HOME}/rosbags}"
LOG_LATEST="${LOG_LATEST:-/tmp/nav2_latest.log}"

latest_log="$(find "$BAG_ROOT" -maxdepth 3 -name bringup.log -printf '%T@ %p\n' 2>/dev/null \
              | sort -nr | head -1 | awk '{print $2}')"

if [[ -z "$latest_log" ]]; then
  echo "[refresh_nav_log_symlink] no bringup.log found under $BAG_ROOT" >&2
  exit 1
fi

ln -sfn "$latest_log" "$LOG_LATEST"
echo "[refresh_nav_log_symlink] linked: $LOG_LATEST -> $latest_log" >&2
echo "$latest_log"
