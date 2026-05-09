# G1 Auto Mapping Nav Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a reusable skill package that can migrate this machine's verified G1 automatic mapping and obstacle-avoidance stack onto another G1 with working Lightning SLAM by dynamically detecting and repairing only the missing ROS 2, DDS, SDK, and workspace components.

**Architecture:** The implementation has two halves. On the source machine, a packaging script creates a clean source snapshot containing `g1_cmd`, `g1_nav`, `g1_sim`, `realsense-ros`, `unitree_ros2`, `unitree_sdk2_python`, `unitree_sdk2-main`, and the required skills while excluding transient and out-of-scope content. On the target machine, a preflight script detects what already exists, an install-or-repair script fills only the missing pieces using online `apt`/`pip`/`git clone` repair when needed, and a generated runtime environment script normalizes CycloneDDS and workspace sourcing before layered verification.

**Tech Stack:** Markdown, Bash, ROS 2 Humble, `colcon`, `rosdep`, `pip`, CycloneDDS, Unitree ROS 2, Unitree Python SDK

---

## File Structure

- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/SKILL.md`
- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/agents/openai.yaml`
- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/install-and-verify.md`
- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/package-manifest.md`
- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/runtime-topology.md`
- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/troubleshooting.md`
- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/package_sources.sh`
- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/preflight_check.sh`
- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/install_or_repair.sh`
- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/target_env.sh`
- Modify: `/home/unitree/ros2_ws/src/g1_nav/docs/superpowers/plans/2026-04-14-g1-auto-mapping-nav-migration-plan.md`

### Task 1: Scaffold The Skill Package And Stable Reference Docs

**Files:**
- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/agents/openai.yaml`
- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/package-manifest.md`
- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/runtime-topology.md`

- [ ] **Step 1: Confirm the new skill package does not exist yet**

Run:

```bash
test -e /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration && echo "unexpectedly present"
```

Expected: no output and exit code `1`.

- [ ] **Step 2: Create the directory skeleton and base metadata**

Run:

```bash
mkdir -p /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/agents
mkdir -p /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets
mkdir -p /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references
cat <<'EOF' > /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/agents/openai.yaml
interface:
  display_name: G1 Auto Mapping Nav Migration
  short_description: Package this G1's verified ros2, unitree_ros2, and sdk stack,
    then repair only the missing pieces on another G1 that already has Lightning SLAM.
policy:
  products:
  - chatgpt
  - codex
  - api
  - atlas
  allow_implicit_invocation: true
EOF
```

- [ ] **Step 3: Write the source bundle manifest**

Create `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/package-manifest.md` with:

```markdown
# Package Manifest

## Include

- `~/ros2_ws/src/g1_cmd`
- `~/ros2_ws/src/g1_nav`
- `~/ros2_ws/src/g1_sim`
- `~/ros2_ws/src/realsense-ros`
- `~/unitree_ros2`
- `~/unitree_sdk2_python`
- `~/unitree_sdk2-main`
- `~/Documents/skill_archive/g1-auto-mapping-nav-migration`
- `~/Documents/skill_archive/unitree-g1-python-sdk-setup`

## Optional Include

- `~/cyclonedds`
  Use only as a fallback when the target machine cannot satisfy the Python SDK's
  CycloneDDS dependency through `/opt/ros/humble` or the user-space overlay in
  `~/unitree_ros2/cyclonedds_user_overlay.sh`.

## Exclude

- `~/ros2_ws/src/zenoh_bridge_tools`
- `~/hoslam_lightning_lm`
- `~/lightning-lm-livox`
- `~/livox_ws`
- `build/`
- `install/`
- `log/`
- `.git/`
- `.pytest_cache/`
- `.worktrees/`
- `__pycache__/`
- `*.pyc`
- `rosbag2_*`

## Target Layout

- `~/ros2_ws/src/g1_cmd`
- `~/ros2_ws/src/g1_nav`
- `~/ros2_ws/src/g1_sim`
- `~/ros2_ws/src/realsense-ros`
- `~/unitree_ros2`
- `~/unitree_sdk2_python`
- `~/unitree_sdk2-main`
- `~/Documents/skill_archive/g1-auto-mapping-nav-migration`
- `~/Documents/skill_archive/unitree-g1-python-sdk-setup`
```

- [ ] **Step 4: Write the runtime topology reference**

Create `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/runtime-topology.md` with:

```markdown
# Runtime Topology

## Ownership Boundaries

- Lightning SLAM owns `/lightning/odometry` and `/lightning/grid_map`.
- `g1_nav` consumes those Lightning topics plus `/scan` and drives Nav2 plus
  exploration.
- `g1_cmd` converts navigation motion output into `unitree_sdk2py` locomotion
  commands.
- `unitree_sdk2py` communicates with the robot through CycloneDDS channels.
- `unitree_ros2` provides the ROS 2 DDS environment and topic visibility such as
  `/sportmodestate`.

## What This Migration Skill Does

- packages the verified source trees from the source machine
- repairs ROS 2 Humble, CycloneDDS runtime, and Unitree SDK dependencies on the
  target machine
- rebuilds `~/unitree_ros2` and `~/ros2_ws` only when needed
- generates a target-side runtime environment script with the detected network
  interface

## What This Migration Skill Does Not Do

- install or migrate Lightning SLAM itself
- migrate `zenoh_bridge_tools`
- assume `~/cyclonedds` is always required

## Verification Layers

1. DDS / ROS 2: `ros2 topic list`, `ros2 topic echo /sportmodestate`
2. SDK: `python3 ~/unitree_sdk2_python/example/g1/high_level/g1_loco_client_example.py <iface>`
3. Nav stack: confirm `/lightning/odometry`, `/lightning/grid_map`, and `/scan`,
   then launch `g1_auto_explore.launch.py`
```

- [ ] **Step 5: Verify the scaffold exists**

Run:

```bash
test -f /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/agents/openai.yaml
test -f /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/package-manifest.md
test -f /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/runtime-topology.md
rg -n 'Include|Optional Include|Exclude|Target Layout' /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/package-manifest.md
rg -n 'Ownership Boundaries|Verification Layers' /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/runtime-topology.md
```

Expected: all `test` commands exit `0`, and `rg` prints the requested headings.

- [ ] **Step 6: Commit**

Run:

```bash
git -C /home/unitree/ros2_ws/src/g1_nav add /home/unitree/ros2_ws/src/g1_nav/docs/superpowers/plans/2026-04-14-g1-auto-mapping-nav-migration-plan.md
git -C /home/unitree/ros2_ws/src/g1_nav commit -m "docs: plan G1 migration skill scaffold"
```

Expected: either a clean commit containing the updated plan or a deliberate choice
to defer this commit because only external skill-archive files changed in the task.

### Task 2: Author The Skill Entry Point And Operator Docs

**Files:**
- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/SKILL.md`
- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/install-and-verify.md`
- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/troubleshooting.md`

- [ ] **Step 1: Confirm the documentation files are still missing**

Run:

```bash
test -f /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/SKILL.md
```

Expected: exit code `1`.

- [ ] **Step 2: Write `SKILL.md` with the full trigger and workflow**

Create `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/SKILL.md` with:

```markdown
---
name: g1-auto-mapping-nav-migration
description: Use when another G1 already has Lightning SLAM working, but needs this machine's verified ros2 workspace, unitree_ros2, and sdk stack migrated with dynamic install-or-repair instead of a full reinstall.
---

# g1 auto mapping nav migration

Follow this order exactly. The target machine is assumed to have network access
for `apt`, `pip`, and `git clone`.

## Scope

This skill migrates and repairs:

- `~/ros2_ws/src/g1_cmd`
- `~/ros2_ws/src/g1_nav`
- `~/ros2_ws/src/g1_sim`
- `~/ros2_ws/src/realsense-ros`
- `~/unitree_ros2`
- `~/unitree_sdk2_python`
- `~/unitree_sdk2-main`

This skill does not migrate:

- Lightning SLAM itself
- `zenoh_bridge_tools`

## Rules

1. Use the source machine's verified business-package snapshot rather than
   cloning fresh upstream copies for `g1_cmd`, `g1_nav`, `g1_sim`, and
   `realsense-ros`.
2. Detect the target machine's current state first, then repair only what is
   missing.
3. Treat `~/cyclonedds` as an optional fallback, not a required payload.
4. Validate DDS / ROS 2 first, then the Python SDK, then `g1_nav`.

## Workflow

1. Read `references/package-manifest.md`.
2. On the source machine, run `assets/package_sources.sh` to create the transfer
   bundle.
3. Move the bundle to the target machine.
4. On the target machine, run `assets/preflight_check.sh`.
5. On the target machine, run `assets/install_or_repair.sh`.
6. Source `assets/target_env.sh`.
7. Follow `references/install-and-verify.md`.
8. If anything fails, use `references/troubleshooting.md`.

## DDS Policy

Prefer this order:

1. `/opt/ros/humble` CycloneDDS runtime
2. `~/unitree_ros2/cyclonedds_user_overlay.sh`
3. `~/cyclonedds` or a fresh `releases/0.10.x` clone only when the Python SDK
   still reports `Could not locate cyclonedds`
```

- [ ] **Step 3: Write the install guide and troubleshooting reference**

Create `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/install-and-verify.md` with:

````markdown
# Install And Verify

## 1. Transfer The Bundle

On the source machine:

```bash
cd ~/Documents/skill_archive/g1-auto-mapping-nav-migration/assets
./package_sources.sh --output-dir ~/Documents/g1-migration-dist
```

On the target machine:

```bash
mkdir -p ~/migration_bundle
tar -xzf ~/Downloads/g1-auto-mapping-nav-migration-*.tar.gz -C ~/migration_bundle
```

## 2. Run Preflight

```bash
cd ~/migration_bundle/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets
./preflight_check.sh --bundle-root ~/migration_bundle/home/unitree
```

Review:

- OS release
- ROS 2 Humble presence
- Unitree ROS 2 presence and build status
- `unitree_sdk2py` import status
- `/sportmodestate`
- `/lightning/odometry`
- `/lightning/grid_map`
- `/scan`

## 3. Install Or Repair

```bash
./install_or_repair.sh --bundle-root ~/migration_bundle/home/unitree
```

If the robot Ethernet NIC is known, pass it explicitly:

```bash
./install_or_repair.sh --bundle-root ~/migration_bundle/home/unitree --iface enp3s0
```

## 4. Source The Unified Runtime

```bash
source ~/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/target_env.sh
```

If the interface was not baked into the generated environment, pass it:

```bash
source ~/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/target_env.sh enp3s0
```

## 5. Verify DDS / ROS 2

```bash
ros2 topic list
ros2 topic echo /sportmodestate
```

## 6. Verify The Python SDK

```bash
cd ~/unitree_sdk2_python
python3 ./example/g1/high_level/g1_loco_client_example.py enp3s0
```

If locomotion is rejected because the robot is in the wrong mode:

```bash
python3 ./example/motionSwitcher/motion_switcher_example.py enp3s0
```

## 7. Verify Nav Inputs

```bash
ros2 topic echo /lightning/odometry --once
ros2 topic echo /lightning/grid_map --once
ros2 topic echo /scan --once
```

## 8. Launch Automatic Exploration

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch g1_nav g1_auto_explore.launch.py
```
````

Create `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/troubleshooting.md` with:

```markdown
# Troubleshooting

## `/sportmodestate` Is Missing

- confirm the correct NIC is used in `CYCLONEDDS_URI`
- confirm `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- source `~/unitree_ros2/setup.sh` or `assets/target_env.sh`
- rerun `ros2 topic list`

## `unitree_sdk2py` Import Fails

- rerun `pip3 install -e ~/unitree_sdk2_python`
- confirm `python3 -c "import unitree_sdk2py"` works
- if the install error mentions CycloneDDS, use the fallback path in the next
  section

## `Could Not Locate cyclonedds`

- try the Humble package path first
- then source `~/unitree_ros2/cyclonedds_user_overlay.sh`
- only then use `~/cyclonedds/install` or clone `releases/0.10.x`

## `g1_nav` Starts But `/scan` Is Missing

- check whether RealSense is expected to publish `/scan`
- if not, launch the RealSense depth-to-scan bridge or enable the intended scan
  publisher

## Lightning Topics Are Missing

- this skill does not install Lightning SLAM
- repair Lightning separately before blaming DDS or SDK setup
```

- [ ] **Step 4: Verify the written docs for key coverage and placeholder-free content**

Run:

```bash
test -f /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/SKILL.md
test -f /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/install-and-verify.md
test -f /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/troubleshooting.md
rg -n 'dynamic install-or-repair|~/cyclonedds|/sportmodestate|g1_auto_explore.launch.py' /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/SKILL.md /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/install-and-verify.md /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/troubleshooting.md
rg -n 'TBD|TODO|placeholder|later' /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/SKILL.md /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/install-and-verify.md /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/troubleshooting.md
```

Expected: the first `rg` prints the required matches; the second `rg` exits `1`.

- [ ] **Step 5: Commit**

Run:

```bash
git -C /home/unitree/ros2_ws/src/g1_nav commit --allow-empty -m "docs: plan G1 migration skill content"
```

Expected: either an intentionally empty checkpoint commit in the planning repo or
an explicit decision to defer commits for external skill-archive content.

### Task 3: Implement Source Packaging And Target Preflight Scripts

**Files:**
- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/package_sources.sh`
- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/preflight_check.sh`

- [ ] **Step 1: Confirm the scripts are absent before writing them**

Run:

```bash
test -f /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/package_sources.sh
```

Expected: exit code `1`.

- [ ] **Step 2: Write the source packaging script**

Create `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/package_sources.sh` with:

```bash
#!/usr/bin/env bash
set -euo pipefail

output_dir="$HOME/Documents/g1-migration-dist"
bundle_name=""
include_cyclonedds=0
dry_run=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir)
      output_dir="$2"
      shift 2
      ;;
    --bundle-name)
      bundle_name="$2"
      shift 2
      ;;
    --include-cyclonedds)
      include_cyclonedds=1
      shift
      ;;
    --dry-run)
      dry_run=1
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if [[ -z "$bundle_name" ]]; then
  bundle_name="g1-auto-mapping-nav-migration-$(date +%F).tar.gz"
fi

required_paths=(
  "$HOME/ros2_ws/src/g1_cmd"
  "$HOME/ros2_ws/src/g1_nav"
  "$HOME/ros2_ws/src/g1_sim"
  "$HOME/ros2_ws/src/realsense-ros"
  "$HOME/unitree_ros2"
  "$HOME/unitree_sdk2_python"
  "$HOME/unitree_sdk2-main"
  "$HOME/Documents/skill_archive/g1-auto-mapping-nav-migration"
  "$HOME/Documents/skill_archive/unitree-g1-python-sdk-setup"
)

for path in "${required_paths[@]}"; do
  [[ -e "$path" ]] || { echo "missing required path: $path" >&2; exit 1; }
done

stage_root="$(mktemp -d)"
trap 'rm -rf "$stage_root"' EXIT
mkdir -p "$output_dir"

copy_tree() {
  local src="$1"
  rsync -a --relative \
    --exclude '.git/' \
    --exclude 'build/' \
    --exclude 'install/' \
    --exclude 'log/' \
    --exclude '.pytest_cache/' \
    --exclude '.worktrees/' \
    --exclude '__pycache__/' \
    --exclude '*.pyc' \
    --exclude 'rosbag2_*' \
    "$src" "$stage_root/"
}

for path in "${required_paths[@]}"; do
  copy_tree "$path"
done

if [[ "$include_cyclonedds" -eq 1 && -d "$HOME/cyclonedds" ]]; then
  copy_tree "$HOME/cyclonedds"
fi

archive_path="$output_dir/$bundle_name"
if [[ "$dry_run" -eq 1 ]]; then
  find "$stage_root" -maxdepth 4 | sort
  echo "dry-run archive path: $archive_path"
  exit 0
fi

tar -czf "$archive_path" -C "$stage_root" .
echo "created bundle: $archive_path"
```

- [ ] **Step 3: Write the target preflight script**

Create `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/preflight_check.sh` with:

```bash
#!/usr/bin/env bash
set -euo pipefail

bundle_root="$HOME"
iface_override=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bundle-root)
      bundle_root="$2"
      shift 2
      ;;
    --iface)
      iface_override="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

status_line() {
  printf '%-26s %s\n' "$1" "$2"
}

topic_visible() {
  local topic_name="$1"
  if ! command -v ros2 >/dev/null 2>&1; then
    return 1
  fi
  timeout 5 ros2 topic list 2>/dev/null | grep -qx "$topic_name"
}

echo "== g1 migration preflight =="
source /etc/os-release
status_line "os" "${PRETTY_NAME:-unknown}"
status_line "ros_humble" "$([[ -f /opt/ros/humble/setup.bash ]] && echo present || echo missing)"
status_line "unitree_ros2" "$([[ -d $HOME/unitree_ros2 ]] && echo present || echo missing)"
status_line "unitree_ros2_build" "$([[ -f $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash ]] && echo built || echo missing)"
status_line "cyclonedds_rmw" "$([[ -f /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ]] && echo present || echo missing)"
status_line "bundle_g1_nav" "$([[ -d $bundle_root/ros2_ws/src/g1_nav ]] && echo present || echo missing)"
status_line "bundle_g1_cmd" "$([[ -d $bundle_root/ros2_ws/src/g1_cmd ]] && echo present || echo missing)"
status_line "bundle_g1_sim" "$([[ -d $bundle_root/ros2_ws/src/g1_sim ]] && echo present || echo missing)"
status_line "bundle_realsense" "$([[ -d $bundle_root/ros2_ws/src/realsense-ros ]] && echo present || echo missing)"
status_line "bundle_sdk_py" "$([[ -d $bundle_root/unitree_sdk2_python ]] && echo present || echo missing)"
status_line "bundle_unitree_ros2" "$([[ -d $bundle_root/unitree_ros2 ]] && echo present || echo missing)"

if python3 -c "import unitree_sdk2py" >/dev/null 2>&1; then
  status_line "unitree_sdk2py" "importable"
else
  status_line "unitree_sdk2py" "missing"
fi

echo "interfaces:"
ip -o link show | awk -F': ' '{print "  - " $2}'

if [[ -n "$iface_override" ]]; then
  status_line "selected_iface" "$iface_override"
fi

status_line "/sportmodestate" "$(topic_visible /sportmodestate && echo visible || echo unavailable)"
status_line "/lightning/odometry" "$(topic_visible /lightning/odometry && echo visible || echo unavailable)"
status_line "/lightning/grid_map" "$(topic_visible /lightning/grid_map && echo visible || echo unavailable)"
status_line "/scan" "$(topic_visible /scan && echo visible || echo unavailable)"
```

- [ ] **Step 4: Run syntax checks and safe dry-runs**

Run:

```bash
bash -n /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/package_sources.sh
bash -n /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/preflight_check.sh
/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/package_sources.sh --dry-run --output-dir /tmp/g1-migration-dist
/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/preflight_check.sh --bundle-root "$HOME"
```

Expected:

- both `bash -n` commands exit `0`
- the packaging dry-run prints the staged tree and a `dry-run archive path`
- the preflight command prints the status report without crashing

- [ ] **Step 5: Commit**

Run:

```bash
git -C /home/unitree/ros2_ws/src/g1_nav commit --allow-empty -m "docs: plan G1 migration packaging scripts"
```

Expected: an explicit checkpoint decision for the planning session.

### Task 4: Implement Dynamic Install / Repair And Unified Runtime Sourcing

**Files:**
- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/install_or_repair.sh`
- Create: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/target_env.sh`

- [ ] **Step 1: Confirm the install-side scripts are absent**

Run:

```bash
test -f /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/install_or_repair.sh
```

Expected: exit code `1`.

- [ ] **Step 2: Write the install-or-repair script**

Create `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/install_or_repair.sh` with:

```bash
#!/usr/bin/env bash
set -euo pipefail

bundle_root=""
iface=""
dry_run=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bundle-root)
      bundle_root="$2"
      shift 2
      ;;
    --iface)
      iface="$2"
      shift 2
      ;;
    --dry-run)
      dry_run=1
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

[[ -n "$bundle_root" ]] || { echo "--bundle-root is required" >&2; exit 2; }
[[ -d "$bundle_root" ]] || { echo "bundle root missing: $bundle_root" >&2; exit 1; }

run() {
  echo "+ $*"
  if [[ "$dry_run" -eq 0 ]]; then
    eval "$@"
  fi
}

ensure_ros_apt_repo() {
  if [[ -f /etc/apt/sources.list.d/ros2.list ]]; then
    return 0
  fi
  run "sudo apt update"
  run "sudo apt install -y curl gnupg lsb-release software-properties-common"
  run "sudo mkdir -p /etc/apt/keyrings"
  run "curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg"
  run "echo 'deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main' | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null"
}

ensure_apt_pkg() {
  local pkg="$1"
  if dpkg -s "$pkg" >/dev/null 2>&1; then
    echo "already installed: $pkg"
    return 0
  fi
  run "sudo apt install -y $pkg"
}

copy_if_missing() {
  local src="$1"
  local dst="$2"
  if [[ -e "$dst" ]]; then
    echo "keeping existing path: $dst"
    return 0
  fi
  run "mkdir -p $(dirname "$dst")"
  run "rsync -a '$src/' '$dst/'"
}

ensure_ros_apt_repo
run "sudo apt update"
ensure_apt_pkg ros-humble-ros-base
ensure_apt_pkg ros-dev-tools
ensure_apt_pkg ros-humble-rmw-cyclonedds-cpp
ensure_apt_pkg ros-humble-rosidl-generator-dds-idl
ensure_apt_pkg libyaml-cpp-dev
ensure_apt_pkg python3-pip
ensure_apt_pkg python3-colcon-common-extensions
ensure_apt_pkg python3-rosdep

copy_if_missing "$bundle_root/unitree_ros2" "$HOME/unitree_ros2"
copy_if_missing "$bundle_root/unitree_sdk2_python" "$HOME/unitree_sdk2_python"
copy_if_missing "$bundle_root/unitree_sdk2-main" "$HOME/unitree_sdk2-main"
copy_if_missing "$bundle_root/ros2_ws/src/g1_cmd" "$HOME/ros2_ws/src/g1_cmd"
copy_if_missing "$bundle_root/ros2_ws/src/g1_nav" "$HOME/ros2_ws/src/g1_nav"
copy_if_missing "$bundle_root/ros2_ws/src/g1_sim" "$HOME/ros2_ws/src/g1_sim"
copy_if_missing "$bundle_root/ros2_ws/src/realsense-ros" "$HOME/ros2_ws/src/realsense-ros"
copy_if_missing "$bundle_root/Documents/skill_archive/g1-auto-mapping-nav-migration" "$HOME/Documents/skill_archive/g1-auto-mapping-nav-migration"
copy_if_missing "$bundle_root/Documents/skill_archive/unitree-g1-python-sdk-setup" "$HOME/Documents/skill_archive/unitree-g1-python-sdk-setup"

if [[ ! -f "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash" ]]; then
  run "bash -lc 'source /opt/ros/humble/setup.bash && cd $HOME/unitree_ros2/cyclonedds_ws && colcon build'"
fi

run "bash -lc 'source /opt/ros/humble/setup.bash && sudo rosdep init || true'"
run "rosdep update"
run "bash -lc 'source /opt/ros/humble/setup.bash && cd $HOME/ros2_ws && rosdep install --from-paths src --ignore-src -r -y'"

if ! python3 -c "import unitree_sdk2py" >/dev/null 2>&1; then
  if ! run "bash -lc 'cd $HOME/unitree_sdk2_python && pip3 install -e .'"; then
    if [[ -d "$HOME/cyclonedds/install" ]]; then
      run "bash -lc 'export CYCLONEDDS_HOME=$HOME/cyclonedds/install && cd $HOME/unitree_sdk2_python && pip3 install -e .'"
    else
      run "bash -lc 'cd $HOME && git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x cyclonedds || true'"
      run "bash -lc 'cd $HOME/cyclonedds && mkdir -p build install && cd build && cmake .. -DCMAKE_INSTALL_PREFIX=../install && cmake --build . --target install'"
      run "bash -lc 'export CYCLONEDDS_HOME=$HOME/cyclonedds/install && cd $HOME/unitree_sdk2_python && pip3 install -e .'"
    fi
  fi
fi

run "bash -lc 'source /opt/ros/humble/setup.bash && cd $HOME/ros2_ws && colcon build --symlink-install'"

target_env="$HOME/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/target_env.sh"
if [[ -n "$iface" ]]; then
  run "sed -i \"s/__ROBOT_IFACE__/$iface/g\" '$target_env'"
fi

echo "install_or_repair complete"
```

- [ ] **Step 3: Write the unified runtime environment helper**

Create `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/target_env.sh` with:

```bash
#!/usr/bin/env bash
set -euo pipefail

robot_iface="${1:-__ROBOT_IFACE__}"

source /opt/ros/humble/setup.bash

if [[ -f "$HOME/unitree_ros2/cyclonedds_user_overlay.sh" ]]; then
  # shellcheck disable=SC1091
  source "$HOME/unitree_ros2/cyclonedds_user_overlay.sh"
  if declare -f _unitree_source_cyclone_overlay >/dev/null 2>&1; then
    _unitree_source_cyclone_overlay
  fi
fi

if [[ -f "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$HOME/unitree_ros2/cyclonedds_ws/install/setup.bash"
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${robot_iface}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"

if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$HOME/ros2_ws/install/setup.bash"
fi

echo "Configured ROS 2 + Unitree DDS environment for interface: ${robot_iface}"
```

- [ ] **Step 4: Run syntax checks and dry-run the installer**

Run:

```bash
bash -n /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/install_or_repair.sh
bash -n /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/target_env.sh
mkdir -p /tmp/g1-migration-bundle/home/unitree
/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/install_or_repair.sh --bundle-root /tmp/g1-migration-bundle/home/unitree --iface test0 --dry-run || true
source /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/target_env.sh test0 || true
```

Expected:

- both `bash -n` commands exit `0`
- the installer dry-run prints the intended commands without performing them
- the environment helper prints the selected interface after sourcing the
  available files

- [ ] **Step 5: Commit**

Run:

```bash
git -C /home/unitree/ros2_ws/src/g1_nav commit --allow-empty -m "docs: plan G1 migration repair scripts"
```

Expected: an explicit planning checkpoint.

### Task 5: Final Verification Of The Skill Package Structure

**Files:**
- Verify: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/SKILL.md`
- Verify: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/agents/openai.yaml`
- Verify: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/install-and-verify.md`
- Verify: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/package-manifest.md`
- Verify: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/runtime-topology.md`
- Verify: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/troubleshooting.md`
- Verify: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/package_sources.sh`
- Verify: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/preflight_check.sh`
- Verify: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/install_or_repair.sh`
- Verify: `/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/target_env.sh`

- [ ] **Step 1: List the finished skill tree**

Run:

```bash
find /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration -maxdepth 3 -type f | sort
```

Expected: the tree includes `SKILL.md`, `agents/openai.yaml`, four reference
docs, and four asset scripts.

- [ ] **Step 2: Run a full script syntax pass**

Run:

```bash
bash -n /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/package_sources.sh
bash -n /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/preflight_check.sh
bash -n /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/install_or_repair.sh
bash -n /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/target_env.sh
```

Expected: no output and exit code `0`.

- [ ] **Step 3: Run document placeholder and keyword checks**

Run:

```bash
rg -n 'Lightning|zenoh_bridge_tools|/sportmodestate|g1_auto_explore.launch.py|~/cyclonedds' /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/SKILL.md /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/*.md
! rg -n 'TBD|TODO|placeholder|later' /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/SKILL.md /home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/references/*.md
```

Expected: the first command prints matches; the second exits `0` because the
negation operator confirms there are no placeholders.

- [ ] **Step 4: Produce a sample migration bundle**

Run:

```bash
/home/unitree/Documents/skill_archive/g1-auto-mapping-nav-migration/assets/package_sources.sh --output-dir /tmp/g1-migration-dist
ls -lh /tmp/g1-migration-dist
```

Expected: a `g1-auto-mapping-nav-migration-YYYY-MM-DD.tar.gz` archive is created
in `/tmp/g1-migration-dist`.

- [ ] **Step 5: Commit**

Run:

```bash
git -C /home/unitree/ros2_ws/src/g1_nav add /home/unitree/ros2_ws/src/g1_nav/docs/superpowers/plans/2026-04-14-g1-auto-mapping-nav-migration-plan.md
git -C /home/unitree/ros2_ws/src/g1_nav commit -m "docs: add G1 migration implementation plan"
```

Expected: the plan document is committed in the `g1_nav` repo.
