#!/usr/bin/env bash

set -euo pipefail

SCRIPT_NAME="$(basename "$0")"
ROS_DOMAIN_ID_DEFAULT="${ROS_DOMAIN_ID_DEFAULT:-30}"
BAG_ROOT="${BAG_ROOT:-/home/unitree/rosbags}"
DRY_RUN=false
OUTPUT_NAME=""

EXPLICIT_TOPICS=(
  /tf
  /tf_static
  /parameter_events
  /rosout
  /cmd_vel
  /cmd_vel_safe
  /cmd_control_command
  /scan
  /map
  /lightning/odometry
  /lightning/grid_map
  /odommodestate
  /sportmodestate
  /lf/odommodestate
  /lf/sportmodestate
  /planner_map
  /gridmap
  /collision_clouds
  /pre_collision_clouds
  /safe_clouds
  /warning_clouds
  /unitree/slam_mapping/odom
  /unitree/slam_mapping/points
  /unitree/slam_relocation/odom
  /unitree/slam_relocation/global_map
  /unitree/slam_relocation/points
  /utlidar/cloud_livox_mid360
  /utlidar/imu_livox_mid360
)

REGEX='^/((local_costmap|global_costmap|frontier|explore|collision_monitor|navigate_to_pose|navigate_through_poses|plan|goal_pose|clicked_point|behavior_tree_log|map_live_ready)(/.*)?)$'

usage() {
  cat <<'EOF'
Usage:
  record_g1_nav_repro_bag.sh [-d] [-o NAME]

Behavior:
  Record the key topics needed to replay and debug G1 mapping/navigation issues.
  Runs in the foreground; press Ctrl+C to stop recording cleanly.

Options:
  -o, --output NAME   Bag folder name. Default: g1_nav_repro_YYYYmmdd_HHMMSS
  -d, --dry-run       Print the ros2 bag command without starting it.
  -h, --help          Show this help.

Shortcuts:
  g1bag               Start recording with default name
  g1bag -o turn_test  Record into /home/unitree/rosbags/turn_test
  g1bag -d            Show the exact command
EOF
}

log() {
  printf '[%s] %s\n' "$SCRIPT_NAME" "$*"
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -o|--output)
        if [[ $# -lt 2 ]]; then
          printf 'Missing value for %s\n' "$1" >&2
          exit 1
        fi
        OUTPUT_NAME="$2"
        shift
        ;;
      -d|--dry-run)
        DRY_RUN=true
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        printf 'Unknown argument: %s\n' "$1" >&2
        usage >&2
        exit 1
        ;;
    esac
    shift
  done
}

require_command() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    printf 'Missing required command: %s\n' "$cmd" >&2
    exit 1
  fi
}

main() {
  parse_args "$@"
  export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_DEFAULT"

  require_command ros2
  mkdir -p "$BAG_ROOT"

  if [[ -z "$OUTPUT_NAME" ]]; then
    OUTPUT_NAME="g1_nav_repro_$(date +%Y%m%d_%H%M%S)"
  fi

  local output_path="${BAG_ROOT}/${OUTPUT_NAME}"
  local -a cmd=(
    ros2 bag record
    --include-unpublished-topics
    --include-hidden-topics
    --compression-mode file
    --compression-format zstd
    --max-cache-size 104857600
    --output "$output_path"
  )

  cmd+=("${EXPLICIT_TOPICS[@]}")
  cmd+=(-e "$REGEX")

  log "Using ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  log "Recording repro bag to ${output_path}"

  if [[ "$DRY_RUN" == true ]]; then
    printf '%q ' "${cmd[@]}"
    printf '\n'
    exit 0
  fi

  exec "${cmd[@]}"
}

main "$@"
