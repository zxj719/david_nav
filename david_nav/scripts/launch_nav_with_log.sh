#!/usr/bin/env bash
# Wrapper around `ros2 launch david_nav 2d_david_nav2_bringup.launch.py …`
# that pipes all output to a timestamped log AND keeps /tmp/nav2_latest.log
# as a symlink to the latest run — so the assistant (or anyone) can always
# `tail -f /tmp/nav2_latest.log` regardless of which run is current.
#
# Usage:
#   launch_nav_with_log.sh [arg1 arg2 …]   # extra args forwarded to ros2 launch
#
# Examples:
#   launch_nav_with_log.sh use_rviz:=true enable_scan_bridge:=false
#
# Background mode (returns control to shell):
#   BG=1 launch_nav_with_log.sh use_rviz:=true
#
# Auto-start the brake-trace recorder (default ON). Set RECORD_BRAKE=0
# to skip. CSV lands next to the launch log:
#   RECORD_BRAKE=0 launch_nav_with_log.sh use_rviz:=true
#
# The log file path is printed on stdout before launch starts.

# Note: NOT using `set -u` because ROS's setup.bash references unset vars
# (e.g. COLCON_TRACE) and would crash under nounset. Keep -e and pipefail.
set -eo pipefail

LOG_DIR="${LOG_DIR:-/tmp/david_nav_logs}"
LOG_LATEST="${LOG_LATEST:-/tmp/nav2_latest.log}"
BG="${BG:-0}"
RECORD_BRAKE="${RECORD_BRAKE:-1}"
RECORD_SCRIPT="${HOME}/ros2_ws/src/david_nav/scripts/record_brake_trace.py"

mkdir -p "$LOG_DIR"
stamp="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="${LOG_DIR}/nav2_${stamp}.log"

# Pre-flight: kill any MoveIt / second-robot_state_publisher processes that
# would publish a *conflicting* URDF (e.g. one with `world -> base_link` from
# david_moveit_config/config/david.urdf.xacro). When two RSPs publish different
# parents for `base_link`, tf2 splits the tree → nav2 can't find
# base_footprint↔odom → costmaps fail / no goals work.
#
# We intentionally only kill the *MoveIt-side* RSP (the one started by david
# arm / move_group launches, whose URDF includes `world` as the URDF root).
# The nav2 launch's own RSP loading david_nav/urdf/david.urdf is fine and
# will be started by ros2 launch below.
PRE_KILL_PATTERNS=(
  "moveit_ros_move_group/move_group"
  "moveit_simple_controller_manager"
  "david_sdk_bridge/lib/david_sdk_bridge/arm_node"
)
echo "[launch_nav_with_log] pre-flight: killing MoveIt residues …" >&2
# IMPORTANT: `pkill -f` matches ANY process whose cmdline contains the
# pattern — including THIS bash script (whose source has the patterns
# baked in as literal strings). That would kill our own shell with
# signal 15, exit 143/144. So we use pgrep + explicit self/parent
# exclusion + kill instead.
SELF_PID=$$
SCRIPT_PARENT=$PPID
for pat in "${PRE_KILL_PATTERNS[@]}"; do
  for pid in $(pgrep -f -- "$pat" 2>/dev/null); do
    [[ "$pid" == "$SELF_PID" ]] && continue
    [[ "$pid" == "$SCRIPT_PARENT" ]] && continue
    if kill "$pid" 2>/dev/null; then
      echo "[launch_nav_with_log]   killed pid=$pid (pattern=$pat)" >&2
    fi
  done
done
# Also kill any robot_state_publisher whose URDF param file mentions
# david_moveit_config (= MoveIt-side RSP, conflicts with chassis RSP).
for pid in $(pgrep -f robot_state_publisher 2>/dev/null); do
  [[ "$pid" == "$SELF_PID" ]] && continue
  pfile=""
  # /proc/<pid>/cmdline is null-separated — tr to newlines and grep
  if [[ -r "/proc/$pid/cmdline" ]]; then
    pfile=$(tr '\0' '\n' < "/proc/$pid/cmdline" | grep -E '^/tmp/launch_params_' | head -1)
  fi
  if [[ -n "$pfile" ]] && grep -q "david_moveit_config" "$pfile" 2>/dev/null; then
    echo "[launch_nav_with_log]   killed MoveIt-side RSP pid=$pid (params=$pfile)" >&2
    kill -9 "$pid" 2>/dev/null || true
  fi
done
sleep 1
echo "[launch_nav_with_log] pre-flight done." >&2

# Refresh "latest" symlink atomically.
ln -sfn "$LOG_FILE" "$LOG_LATEST"

echo "[launch_nav_with_log] log: $LOG_FILE" >&2
echo "[launch_nav_with_log] symlink: $LOG_LATEST" >&2

# Source ROS env with nounset relaxed (setup.bash uses unset vars).
safe_source() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}
# We're already not using -u so plain source is fine, but keep helper for clarity.
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${HOME}/ros2_ws/install/setup.bash"

CMD=(ros2 launch david_nav 2d_david_nav2_bringup.launch.py "$@")
echo "[launch_nav_with_log] cmd: ${CMD[*]}" >&2

# ---- start brake-trace recorder in background (optional) ----
RECORDER_PID=""
TRACE_CSV=""
if [[ "$RECORD_BRAKE" == "1" ]]; then
  if [[ -x "$RECORD_SCRIPT" ]]; then
    TRACE_CSV="${LOG_DIR}/brake_trace_${stamp}.csv"
    RECORDER_LOG="${LOG_DIR}/brake_trace_${stamp}.log"
    nohup python3 "$RECORD_SCRIPT" --output "$TRACE_CSV" \
        > "$RECORDER_LOG" 2>&1 &
    RECORDER_PID=$!
    echo "[launch_nav_with_log] brake_trace recorder PID=$RECORDER_PID" >&2
    echo "[launch_nav_with_log] brake_trace csv: $TRACE_CSV" >&2
  else
    echo "[launch_nav_with_log] WARNING: $RECORD_SCRIPT not found / not exec — skipping recorder" >&2
  fi
fi

# Clean up recorder when this script exits (Ctrl-C in foreground mode,
# or when ros2 launch terminates).
cleanup_recorder() {
  if [[ -n "$RECORDER_PID" ]] && kill -0 "$RECORDER_PID" 2>/dev/null; then
    echo "[launch_nav_with_log] stopping brake_trace recorder PID=$RECORDER_PID" >&2
    kill -INT "$RECORDER_PID" 2>/dev/null || true
    sleep 0.5
    kill -0 "$RECORDER_PID" 2>/dev/null && kill -9 "$RECORDER_PID" 2>/dev/null || true
  fi
  if [[ -n "$TRACE_CSV" ]] && [[ -f "$TRACE_CSV" ]]; then
    echo "[launch_nav_with_log] brake_trace csv saved: $TRACE_CSV" >&2
  fi
}
trap cleanup_recorder EXIT INT TERM

if [[ "$BG" == "1" ]]; then
  nohup "${CMD[@]}" > "$LOG_FILE" 2>&1 &
  echo "[launch_nav_with_log] PID=$!" >&2
  # Don't run cleanup_recorder on exit — let it run alongside nav2 in BG mode.
  trap - EXIT INT TERM
else
  # Foreground: don't exec (we need our trap to fire on exit).
  bash -o pipefail -c '"$@" 2>&1 | tee "$0"' "$LOG_FILE" "${CMD[@]}" || true
fi
