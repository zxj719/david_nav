#!/usr/bin/env bash
# deploy.sh — Deploy operator_console to David robot
#
# Usage:
#   ./deploy.sh                    # deploy to default David
#   ./deploy.sh 192.168.1.100      # deploy to specific IP
#   ./deploy.sh --check            # dry-run: show what would be done
#
# Prerequisites on David:
#   - Ubuntu 22.04 + ROS 2 Humble
#   - Python 3 with PyYAML (sudo apt install python3-yaml)
#   - SSH access as unitree user

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
David_USER="${David_USER:-unitree}"
David_IP="${1:-172.16.22.130}"
David_DEST="/home/${David_USER}/ros2_ws/src/david_nav/operator_console"
CHECK_ONLY=false

if [[ "${1:-}" == "--check" ]]; then
    CHECK_ONLY=true
    David_IP="<David_IP>"
fi

log() { printf '[deploy] %s\n' "$*"; }

if [[ "$CHECK_ONLY" == true ]]; then
    log "=== Dry-run: showing deployment plan ==="
    log ""
    log "1. Copy operator_console/ to ${David_USER}@${David_IP}:${David_DEST}/"
    log "   Files:"
    find "$SCRIPT_DIR" -type f | sort | while read -r f; do
        printf '     %s\n' "${f#$SCRIPT_DIR/}"
    done
    log ""
    log "2. Install foxglove_bridge on David:"
    log "   sudo apt update && sudo apt install -y ros-humble-foxglove-bridge"
    log ""
    log "3. Install Python dependencies on David:"
    log "   sudo apt install -y python3-yaml"
    log ""
    log "4. Install systemd services:"
    log "   sudo cp ${David_DEST}/systemd/foxglove-bridge.service /etc/systemd/system/"
    log "   sudo cp ${David_DEST}/systemd/launch-manager.service /etc/systemd/system/"
    log "   sudo systemctl daemon-reload"
    log "   sudo systemctl enable --now foxglove-bridge launch-manager"
    log ""
    log "5. Verify:"
    log "   curl http://${David_IP}:8080/api/health"
    log "   Open http://${David_IP}:8080 on phone"
    log "   Open Foxglove Web, connect to ws://${David_IP}:8765"
    exit 0
fi

log "Deploying to ${David_USER}@${David_IP}:${David_DEST}/"

# Step 1: Copy files
log "Step 1/4: Copying files..."
ssh "${David_USER}@${David_IP}" "mkdir -p ${David_DEST}"
rsync -avz --delete \
    "${SCRIPT_DIR}/" \
    "${David_USER}@${David_IP}:${David_DEST}/"

# Step 2: Install dependencies
log "Step 2/4: Installing dependencies..."
ssh "${David_USER}@${David_IP}" "dpkg -l ros-humble-foxglove-bridge >/dev/null 2>&1 || \
    (echo 'Installing foxglove_bridge...' && sudo apt update -qq && sudo apt install -y -qq ros-humble-foxglove-bridge)"
ssh "${David_USER}@${David_IP}" "python3 -c 'import yaml' 2>/dev/null || \
    (echo 'Installing python3-yaml...' && sudo apt install -y -qq python3-yaml)"

# Step 3: Install systemd services
log "Step 3/4: Installing systemd services..."
ssh "${David_USER}@${David_IP}" "sudo cp ${David_DEST}/systemd/foxglove-bridge.service /etc/systemd/system/ && \
    sudo cp ${David_DEST}/systemd/launch-manager.service /etc/systemd/system/ && \
    sudo systemctl daemon-reload && \
    sudo systemctl enable foxglove-bridge launch-manager"

# Step 4: Start / restart services
log "Step 4/4: Starting services..."
ssh "${David_USER}@${David_IP}" "sudo systemctl restart foxglove-bridge launch-manager"

log ""
log "=== Deployment complete ==="
log ""
log "Control panel: http://${David_IP}:8080"
log "Health check:  curl http://${David_IP}:8080/api/health"
log "Foxglove:      Open https://app.foxglove.dev, connect ws://${David_IP}:8765"
log ""
log "Service status:"
ssh "${David_USER}@${David_IP}" "systemctl --no-pager status foxglove-bridge launch-manager" || true
