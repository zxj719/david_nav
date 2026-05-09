# 2026-04-14 G1 Auto Mapping Nav Migration Design

## Summary

This design defines a reusable skill package for migrating the verified G1
automatic mapping and obstacle-avoidance stack from the current machine to
another G1 that already has Lightning SLAM working.

The migration package must:

- carry the current machine's verified business-source snapshot
- dynamically detect what already exists on the target machine
- install or repair only the missing ROS 2, DDS, SDK, and workspace pieces
- leave Lightning SLAM itself untouched
- exclude the Zenoh bridge toolkit from the migration

The target machine is assumed to have working network access for `apt`,
`pip`, and `git clone`, so the migration flow can use online dependency repair
instead of relying on a fully offline bundle.

## Problem

The current G1 workspace is not a single self-contained package. The automatic
mapping and obstacle-avoidance workflow spans:

- `~/ros2_ws/src/g1_nav` for Nav2 bringup and autonomous exploration
- `~/ros2_ws/src/g1_cmd` for command bridging into the Unitree Python SDK
- `~/ros2_ws/src/g1_sim` for robot description and TF publishing
- `~/ros2_ws/src/realsense-ros` for the RealSense ROS 2 packages
- `~/unitree_ros2` for Unitree DDS / ROS 2 integration
- `~/unitree_sdk2_python` for runtime robot motion control
- `~/unitree_sdk2-main` for the C++ SDK sources

The destination G1 is only partially prepared: Lightning SLAM is already
integrated, but the ROS 2, Unitree DDS, and SDK state is unknown. A static
"reinstall everything" guide would be too destructive, while a pure
"copy these folders" guide would be too brittle when DDS or SDK dependencies
are missing.

The migration therefore needs an explicit preflight phase, dynamic install and
repair logic, and layered verification so failures can be attributed to the
correct subsystem.

## Goals

- Reproduce the current machine's automatic mapping and obstacle-avoidance
  behavior on another G1.
- Reuse the current machine's verified application sources instead of pulling
  upstream HEAD for business packages.
- Dynamically skip components already present on the target machine.
- Treat CycloneDDS as a conditionally repaired dependency instead of a blanket
  mandatory source build.
- Package the knowledge as a reusable skill that another G1 agent can invoke.

## Non-Goals

- Migrating or reinstalling Lightning SLAM itself.
- Migrating `zenoh_bridge_tools` or any DDS-over-Zenoh bridge workflow.
- Producing a fully offline installer.
- Refactoring the current navigation or SDK code as part of the migration.

## Current Runtime Topology

The runtime stack to be reproduced on the target machine is:

1. Lightning SLAM publishes `/lightning/odometry` and `/lightning/grid_map`.
2. `g1_nav` consumes those topics plus `/scan`.
3. `g1_nav` launches Nav2 and exploration, then publishes motion commands.
4. `g1_cmd` converts navigation commands into `unitree_sdk2py` locomotion
   calls.
5. `unitree_sdk2py` talks to the robot through CycloneDDS-based Unitree SDK
   channels.
6. `unitree_ros2` provides the ROS 2 DDS environment and topic visibility such
   as `/sportmodestate`.

This separation matters operationally:

- Lightning ownership stays outside the migration package.
- Nav2 and obstacle avoidance depend on Lightning topics and `/scan`.
- Robot motion depends on Unitree DDS and the Python SDK even if Lightning is
  already healthy.

## Chosen Approach

Use an online, dynamic-repair migration skill with two clear halves:

### Source-Machine Half

The source machine produces a clean source snapshot containing the verified
workspace packages, Unitree ROS 2 tree, SDK sources, and required skills.

This snapshot intentionally excludes:

- `zenoh_bridge_tools`
- Lightning / Livox SLAM trees
- `build/`, `install/`, `log/`
- `.git`, `.pytest_cache`, `.worktrees`, rosbags, and other transient data

### Target-Machine Half

The target machine runs a preflight script that detects:

- whether Ubuntu 22.04 is present
- whether ROS 2 Humble is already installed
- whether `~/unitree_ros2` exists and is already built
- whether CycloneDDS ROS 2 runtime support is available
- whether `unitree_sdk2py` imports successfully
- whether the business package sources are already present
- whether Lightning topics are visible
- whether `/scan` already exists or must be bridged from RealSense

The installer then repairs only what is missing:

- missing ROS 2 system packages via `apt`
- missing Python dependencies via `pip`
- missing source trees from the transferred snapshot
- missing workspace builds via `colcon build`
- missing DDS runtime via system packages or the existing overlay mechanism

This preserves target-machine state when healthy while still converging an
unknown machine to a working state.

## Packaging Boundaries

The migration package should include these source trees:

- `~/ros2_ws/src/g1_cmd`
- `~/ros2_ws/src/g1_nav`
- `~/ros2_ws/src/g1_sim`
- `~/ros2_ws/src/realsense-ros`
- `~/unitree_ros2`
- `~/unitree_sdk2_python`
- `~/unitree_sdk2-main`

The package should also include the skill artifacts needed to operate the
migration on the target machine:

- the new `g1-auto-mapping-nav-migration` skill
- the existing `unitree-g1-python-sdk-setup` skill

Optional navigation-debug skills may be bundled for later diagnosis, but they
are not required for the initial migration path.

The package should exclude:

- `~/ros2_ws/src/zenoh_bridge_tools`
- `~/hoslam_lightning_lm`
- `~/lightning-lm-livox`
- `~/livox_ws`
- `~/ros2_ws/build`
- `~/ros2_ws/install`
- `~/ros2_ws/log`
- `.git` histories by default
- caches, rosbags, and temporary artifacts

## CycloneDDS Strategy

`~/cyclonedds` should not be treated as a default required migration input.

The preferred DDS recovery order is:

1. use the ROS 2 Humble system packages already installed in `/opt/ros/humble`
2. if CycloneDDS ROS 2 runtime is missing there, use
   `~/unitree_ros2/cyclonedds_user_overlay.sh` to bootstrap a user-space
   overlay from downloaded Debian packages
3. only if the Python SDK installation still fails with an error such as
   `Could not locate cyclonedds`, fall back to a source-built CycloneDDS tree
   such as `~/cyclonedds/install` or a freshly cloned `releases/0.10.x`

This policy matches the current environment:

- `unitree_ros2` already contains overlay logic for missing
  `rmw_cyclonedds_cpp`
- `unitree_sdk2_python` pins `cyclonedds==0.10.2`
- the main need for a standalone CycloneDDS source tree is Python SDK build
  compatibility when system or overlay paths are not enough

The migration skill should therefore describe `~/cyclonedds` as an optional
compatibility fallback, not as a mandatory payload.

## Dynamic Install And Repair Flow

The target-machine workflow should be:

### Step 1: Preflight

Collect and print:

- OS release
- ROS 2 Humble presence
- active network interfaces
- `~/unitree_ros2` presence and build state
- CycloneDDS ROS 2 runtime presence
- `unitree_sdk2py` import status
- transferred source tree presence
- visibility of `/sportmodestate`
- visibility of `/lightning/odometry`, `/lightning/grid_map`, and `/scan`

### Step 2: Repair Missing System Dependencies

Install only the missing base packages, including:

- `ros-humble-ros-base`
- `ros-dev-tools`
- `ros-humble-rmw-cyclonedds-cpp`
- `ros-humble-rosidl-generator-dds-idl`
- `libyaml-cpp-dev`
- additional ROS dependencies resolved by `rosdep`

### Step 3: Repair Unitree ROS 2 Environment

- copy `~/unitree_ros2` from the bundle if it does not exist
- reuse it in place if it already exists
- build `~/unitree_ros2/cyclonedds_ws` only if install artifacts are missing
- generate or patch the runtime setup script using the detected target-machine
  interface instead of hard-coding the source-machine NIC names

### Step 4: Repair Python SDK Environment

- copy `~/unitree_sdk2_python` if missing
- run `pip3 install -e .` when `unitree_sdk2py` is not importable
- use the CycloneDDS source fallback only when the known Python SDK build error
  appears

### Step 5: Repair ROS 2 Workspace Sources

- copy `g1_cmd`, `g1_nav`, `g1_sim`, and `realsense-ros` from the migration
  snapshot if missing
- prefer these transferred sources over fresh upstream clones so the target
  machine matches the verified source machine
- run `rosdep install` for the workspace
- build the workspace with `colcon build`

### Step 6: Prepare Unified Runtime Environment

Generate a single target-side environment helper that sources:

- `/opt/ros/humble/setup.bash`
- the repaired `~/unitree_ros2` environment
- `~/ros2_ws/install/setup.bash`

The helper must use the target machine's detected interface name when
constructing `CYCLONEDDS_URI`.

## Verification Strategy

Verification should be layered so failures are isolated quickly.

### Layer 1: DDS / ROS 2 Robot Communication

Validate that:

- `source ~/unitree_ros2/setup.sh` or an equivalent generated env script works
- `ros2 topic list` shows robot topics
- `ros2 topic echo /sportmodestate` returns data

### Layer 2: Unitree Python SDK

Validate that:

- `python3 ./example/g1/high_level/g1_loco_client_example.py <iface>` runs
- if needed, `motion_switcher_example.py <iface>` can place the robot in the
  required mode before locomotion tests

### Layer 3: Automatic Mapping And Obstacle Avoidance

Validate that:

- `/lightning/odometry` exists
- `/lightning/grid_map` exists
- `/scan` exists, either natively or via the RealSense bridge
- `g1_nav/launch/g1_auto_explore.launch.py` starts successfully

This layered structure prevents misdiagnosing a Lightning issue as a DDS issue
or a DDS issue as a Nav2 issue.

## Skill Deliverables

The skill package should contain:

- `SKILL.md`
- `agents/openai.yaml`
- `references/install-and-verify.md`
- `references/package-manifest.md`
- `references/runtime-topology.md`
- `references/troubleshooting.md`
- `assets/package_sources.sh`
- `assets/preflight_check.sh`
- `assets/install_or_repair.sh`
- `assets/target_env.sh`

The responsibilities should be:

- `SKILL.md`: trigger conditions, scope, workflow order, and key policy rules
- `install-and-verify.md`: exact operator-facing commands for the target
  machine
- `package-manifest.md`: authoritative include and exclude list
- `runtime-topology.md`: subsystem ownership and data-flow explanation
- `troubleshooting.md`: known failure signatures and decision tree
- `package_sources.sh`: clean source-side bundle creation
- `preflight_check.sh`: target-side environment detection
- `install_or_repair.sh`: conditional install and repair actions
- `target_env.sh`: consistent target runtime setup

## Risks And Tradeoffs

- A dynamic repair flow is safer for unknown targets, but it is more complex
  than a single full-reinstall script.
- Copying the source machine's business packages improves reproducibility, but
  it also means the migration bundle must be curated carefully so transient
  artifacts are excluded.
- Not bundling `~/cyclonedds` by default keeps the package smaller and cleaner,
  but the troubleshooting path must be explicit so operators know when to use
  the fallback.
- Because Lightning is treated as an external prerequisite, the migration skill
  must clearly explain that a successful DDS / SDK repair does not imply the
  SLAM topics are healthy.
