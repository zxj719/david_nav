# david_config

Configuration data of the David robot. This package holds no code: robot
specific values live here so that navigation packages stay robot independent
(refactor plan §5.3, §12). Porting to another robot means copying this package
to `<robot>_config` and changing values, not code.

## Status

| Component | State | Plan reference |
|---|---|---|
| `robot/david.yaml`: robot profile (frames, footprint, limits, goal tolerances, chassis adapter and driver, teleop, localization, Nav2 topic wiring, sensors, platform, preflight, recording) with the values production runs today | available (PROGRESS P2.2a, P2.3) | plan §12.2 |
| `sites/daolan/site.yaml` + `maps/daolan_map_nav.yaml`: navigation map (trinary; image stays in the lightning map directory), point files, agent server | available (P2.2b) | plan §14 |
| `modes/{tour,precise,bench_slamgt,teleop}.yaml`: mode overlays holding only the differences (profile values and Nav2 parameter overrides) | available (P2.2c) | plan §12.2 |
| `urdf/david.urdf`: robot description | available (P2.2c) | plan §12.6 |

The site map yaml and the URDF are byte-identical copies of
`david_nav/config/maps/daolan_map_nav.yaml` and `david_nav/urdf/david.urdf`
until the launch files read this package (Phase 4); tests fail if the copies
drift apart. The URDF still loads its meshes from `david_nav`.

## Frames

`base_link` and `base_footprint` are different frames and are never merged:
Nav2, docking and costmaps use `frames.nav_base` (`base_footprint`); the body
frame published by localization is `frames.body` (`base_link`).

## Rendering Nav2 parameters

`navcfg render` (package `nav_bringup`) combines the Nav2 base parameter tree
and binding table installed by `nav_bringup` with this profile and one mode:

```bash
share=$(ros2 pkg prefix david_config)/share/david_config
navcfg render --profile $share/robot/david.yaml --mode $share/modes/precise.yaml \
  --output /tmp/nav2_precise.yaml
```

Values that several Nav2 parameters share (frames, footprint, speed limits,
goal tolerances, odom, scan and map topics) are written once in the profile. A
mode changes them under `profile:` and changes other Nav2 parameters under
`nav2_overrides:` (for example the controller critics of `precise`). The refactor
launch consumes prepared run-local files. Production launch files still load
`david_nav/config`; switching production aliases remains an operator step.

## Startup and recording data

### Optional L2/L3 experiments (P5.2)

The five default mode files are unchanged. Complete alternative mode overlays
are installed under `modes/experiments/{tour,precise}_{l2,l3,l2_l3}.yaml`.
They still declare `mode: tour` or `mode: precise`; precise retains its goal
tolerances and RotateToGoal critic. Select one explicitly, initially read-only:

```bash
navrun up --mode precise --robot david --site daolan --no-executor --dry-run \
  --mode-file /absolute/share/david_config/modes/experiments/precise_l2_l3.yaml
```

`--mode-file` replaces the default mode overlay, not the robot profile. Its
identity must match `--mode`; its exact bytes and source hash enter the run.
`navcfg render/validate --mode` also accepts these files. Do not enable an
experiment in production before the P5.U1 motion/stop/resource A/B review.

| Option | Explicit delta |
|---|---|
| L2 | Omit velocity_smoother process and lifecycle entry; route Nav2 directly to the unchanged adapter input; behavior max_rotational_vel 0.8 → 0.5; adapter timeout 2.0 → 1.0 s; angular hard limit remains 0.60, stationary floor remains 0.55 |
| L3 | Adapter publish_hz 50 → 20; no other command-processing change |
| L2+L3 | Both sets of changes |

The old smoother parameter section is retained as inactive provenance. Defaults,
odom wiring, dock/U2 behavior and localization are unchanged. In-memory tests
compare steady differential commands (y=0, within declared upstream limits) and
Spin after the new cap, plus exact parameter deltas, timer periods and one-shot
stale zeros. They do **not** prove identical inter-message timing or behavior for
out-of-contract mixed/lateral commands: removing a resampling/clamping hop and
reducing timer rate can change both. Bag comparison of Spin/BackUp/navigation,
physical braking/startup, and CPU savings remain P5.U1; no nonzero command is
published by the automated tests.

The refactor startup planner now binds the WS server and POI paths from site.yaml.
`pois.dock_poi_ids: [POI_017]` applies only to precise mode; it does not change tour
routing. Both modes retain a 120-second Nav2 readiness deadline and a 3-second WS
reconnect interval. The legacy executor implementation is not otherwise changed.

The robot profile now holds the exact old light/full bag topic lists, full regex,
100 MiB cache and full-only hidden/unpublished/zstd options. Large global maps stay
out of light recording. The golden test reads these list definitions without ever
executing the old shell script. These changes prepare the refactor workflow; they
do not deploy new aliases or start recordings in the production environment.

## How the values are guarded

`modes/experiments/{tour,precise}_cpp.yaml` selects the C++ adapter explicitly
through the same `--mode-file` interface. Numerical settings and the original
smoother remain unchanged; default modes still use Python. The component and
equivalence/zero-only validation are documented in `chassis_adapter/README.md`.

`test/test_robot_profile.py` compares the profile with the effective parameters
dumped from the pre-refactor bringup (`david_nav/test/golden/params`, PROGRESS
P0.7), the `david_move` and keyboard defaults, the point-cloud-to-scan settings
of the bringup launch and the wheeltec driver launch defaults. It also checks
the profile against every package schema found in the workspace (`navcfg`).

`test/test_modes.py` renders tour, precise and bench_slamgt and compares the
result key by key with `david_nav/config/nav2_params_2d.yaml`,
`nav2_params_2d_precise.yaml` and `nav2_params_2d_bench_slamgt.yaml`
(footprints compared as polygons), and checks that teleop gives the chassis
adapter the `david_move` values of `david_cmd/config/cmd.yaml`. A launch file
can still rewrite parameters after loading a file (the bringup rewrites the
static layer map topic, for example); comparing rendered parameters with the
dumped effective ones is PROGRESS P2.4.

## Testing

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws_refactor
colcon build --symlink-install --metas src/colcon.meta --packages-up-to david_config
ROS_DOMAIN_ID=99 colcon test --packages-select david_config --event-handlers console_direct+
colcon test-result --verbose
```
