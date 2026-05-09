# 2026-04-16 Headless Navigation CPU Profiling Report

## Summary

This report captures a live CPU profiling snapshot while running:

```bash
./start_navigation_headless.sh -rse
```

Observed symptom:

- The system load was high.
- `rviz2` looked hung from the UI side.
- Multiple navigation processes were active at the same time.

Primary conclusion:

- The dominant CPU consumer is `rviz2`, not the navigation executor and not SLAM.
- `rviz2` is spending most of its CPU time in software rasterization threads named `llvmpipe-*`.
- The likely trigger is the RViz map rendering pipeline hitting a GLSL shader/link issue while rendering occupancy-grid style map displays.

## Profiling Snapshot

Snapshot time:

- 2026-04-16 11:34 Asia/Shanghai

System-level `top` snapshot:

```text
load average: 10.79, 7.00, 4.71
%Cpu(s): 70.1 us, 7.5 sy, 19.7 id
Mem: 15655.6 MiB total, 8825.2 MiB free
```

Interpretation:

- The machine was CPU-bound rather than memory-bound.
- About 80% of CPU time was busy (`us + sy`), with around 20% idle left.
- Memory pressure was low and swap was unused.

## Top Processes By CPU

From `ps -eo ... --sort=-pcpu` and `top -b -n 1`:

| PID | Process | Approx CPU | Notes |
|---|---|---:|---|
| 15117 | `rviz2` | 174% to 194% | Largest consumer by a wide margin |
| 9648 | `gnome-shell` | 48% to 217% | Desktop compositor also elevated, likely affected by RViz rendering load |
| 14905 | `run_slam_online` | 12% to 25% | Moderate, expected for live SLAM |
| 14986 | `navigation_executor.py` | 12% to 18% | Noticeable but much smaller than RViz |
| 15128 | `g1_move` | 8% to 18% | Moderate |
| Nav2 servers | mostly 4% to 12% each | Normal active navigation stack background cost |

Important observation:

- Even though `run_slam_online`, `navigation_executor.py`, and Nav2 nodes all contribute CPU, the single biggest abnormal consumer is still RViz.

## RViz Thread-Level Profiling

Thread snapshot for PID `15117`:

```text
15196 llvmpipe-0  ~21%
15197 llvmpipe-1  ~20%
15198 llvmpipe-2  ~19%
15199 llvmpipe-3  ~18%
15200 llvmpipe-4  ~18%
15201 llvmpipe-5  ~17%
15202 llvmpipe-6  ~17%
15203 llvmpipe-7  ~18%
```

Interpretation:

- RViz is not primarily burning CPU in the Qt event loop.
- The load is concentrated in Mesa software rendering workers (`llvmpipe-*`).
- This strongly indicates software OpenGL fallback or a rendering path that is effectively executing on CPU instead of GPU.

## RViz Rendering Errors

From the live RViz log:

File:

- `/home/unitree/.ros/log/rviz2_15117_1776310221399.log`

Key lines:

```text
[INFO] OpenGl version: 4.5 (GLSL 4.5)
[ERROR] GLSL link result :
active samplers with a different type refer to the same texture image unit
```

Interpretation:

- RViz starts and initializes OpenGL, so this is not a simple "window failed to launch" problem.
- The rendering pipeline later hits a GLSL program link error.
- After that, RViz continues trying to create map textures:

```text
Trying to create a map of size 75 x 70 using 1 swatches
Trying to create a map of size 30 x 30 using 1 swatches
Trying to create a map of size 85 x 70 using 1 swatches
```

- This aligns with the visible symptom of RViz appearing stuck while still consuming CPU.

## Why RViz Is the Main Suspect

The loaded RViz config enables three map-like displays at the same time:

- `Map` using `/lightning/grid_map`
- `Global Costmap` using `/global_costmap/costmap`
- `Local Costmap` using `/local_costmap/costmap`

These are defined in:

- `rviz/nav2_go2_view.rviz`

The shader error is consistent with occupancy-grid / indexed texture rendering, so the current evidence points to those map displays as the immediate trigger rather than TF, robot model, or path displays.

## Other Process Notes

### Lightning SLAM

`run_slam_online` was active but not the dominant bottleneck.

Thread view showed a few working threads in the high single-digit range, but not the broad multi-core saturation seen in RViz.

Conclusion:

- SLAM contributes meaningful CPU load, but it is not the primary reason total CPU is high in this session.

### Navigation Executor

`navigation_executor.py` showed roughly low-to-mid teens CPU.

Thread view showed almost all work concentrated in the main Python thread.

Conclusion:

- The executor is active and not free, but it is not the root cause of the current CPU spike.

### Nav2 Stack

`controller_server`, `planner_server`, `behavior_server`, `bt_navigator`, `velocity_smoother`, and `waypoint_follower` were all active in the single-digit to low-teens CPU range.

Conclusion:

- Nav2 is doing real work, but the pattern looks like normal background compute for a live stack, not a runaway failure.

## Root Cause Assessment

Most likely root cause:

1. RViz loads the default `nav2_go2_view.rviz` profile.
2. That profile enables multiple map/costmap displays simultaneously.
3. RViz hits a GLSL texture/shader compatibility issue.
4. Rendering falls into or behaves like a software-rendered path using `llvmpipe`.
5. CPU usage rises sharply and the RViz UI appears hung or unresponsive.

This is an inference from the live evidence above, especially:

- process-level CPU ranking
- thread-level `llvmpipe-*` dominance
- RViz GLSL link error
- repeated map texture creation log entries

## Practical Impact

Current impact on the system:

- RViz is the single largest avoidable CPU consumer in this run mode.
- Desktop responsiveness may degrade because `gnome-shell` is also elevated.
- The navigation stack can still run, but operator visibility through RViz becomes unreliable.

## Recommended Next Steps

Recommended first:

1. Disable RViz map-like displays by default in `rviz/nav2_go2_view.rviz`:
   - `Map`
   - `Global Costmap`
   - `Local Costmap`
2. Re-run the same startup and compare `rviz2` CPU again.

Recommended second:

1. If RViz is still needed, try a reduced RViz config that keeps only:
   - TF
   - Path
   - Pose
   - optional LaserScan
2. Use that config as the default for remote/headless operation.

Optional follow-up:

1. Test forcing a different GL backend or a simpler rendering path for RViz.
2. Compare behavior with and without `Map` displays individually to identify the exact offending display.

## Bottom Line

At the time of profiling, the high CPU condition was primarily caused by RViz software rendering under a map-display shader/rendering failure, not by SLAM, not by the navigation executor, and not by Nav2 alone.
