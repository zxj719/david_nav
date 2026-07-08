# CLAUDE.md — David ROS2 Workspace Onboarding

> 任何 agent 接手这个 workspace 时先读这个文件。包含：项目要点、组件清单、典型
> debug 流程、最容易踩的坑。具体细节都指向 `david_nav/docs/reports/` 里的报告。

## 1. 项目一句话

David 是一台 **70 kg 量级差速底盘 + 双 7 自由度臂 + 双灵巧手** 的人形操作机器人。
当前 workspace 主要做：
- `david_nav`：2D 导航（nav2 + 自研 cmd_vel_separator 适配高负载低力矩底盘 +
  万向轮）
- `david_cmd`：底盘 velocity bridge（输入 `/cmd_vel_safe` → 输出 `/cmd_vel`，
  做 deadzone lift 和 sanitize）
- `david_moveit_config` / `david_description`：MoveIt 配置 + URDF（含 world 节
  点，与 nav2 chassis-only URDF 有冲突，需要 pre-flight 杀掉）
- `david_sdk_bridge`：arm SDK ↔ ROS bridge
- `david_grasp`：抓取任务

## 2. cmd_vel 全链路

这是 David 调试时最重要的一张图：

```
DWB (controller_server) → /cmd_vel_nav
  → velocity_smoother    → /cmd_vel_smoothed
  → collision_monitor    → /cmd_vel_safe       (可选，默认关)
  → david_move           → /cmd_vel_raw
  → cmd_vel_separator    → /cmd_vel            ← 自研，所有适配逻辑在此
  → wheeltec_robot_node  → STM32 (PI 电机环)
```

`cmd_vel_separator` 是 nav2 之外**专门适配 David 底盘**的一层：
- 线/角互斥硬切（差速底盘不允许同时给 v 和 ω）
- binary_output（cmd 强制端点，踢穿万向轮静摩擦）
- 主动反向制动（消耗机身动量对抗 caster 摩擦）
- 1m goal_brake（绕过 upstream 缓冲直接刹）
- angle_brake（旋转预测性提前停 30° 让动量补完）
- angular sign-latch（180° 对称点抗 ω 翻向）
- input_dead 时间滞回（抗 DWB 单 tick 抽风）
- silent_on_idle（让位给键盘遥控）

## 3. 三个必读文档（按顺序）

1. **[Nav 调参方法 - 高负载低力矩底盘](david_nav/docs/reports/2026-05-20-nav-tuning-for-heavy-load-low-torque-chassis.md)**
   nav2 stack 在 David 上特定问题（旋转打滑、起步死锁、终点过冲、180° 翻向）的
   配方 + 一整套 separator 参数（生产配置）。
2. **[一键启动集成 debug 经验](david_nav/docs/reports/2026-05-20-integrated-debug-launch-experience.md)**
   `launch_nav_with_log.sh` / `navigate_semantic_pose_with_debug_bag.sh` 的设计：
   自动 log 落盘、pre-flight 清理、brake_trace CSV 录制、RViz 延迟启动。
3. **[Nav2 调试常见坑 - RViz / TF 视角](david_nav/docs/reports/2026-05-20-rviz-and-tf-pitfalls.md)**
   每个曾经卡住人的"屏幕上没东西"+对应根因 + 修法。

## 4. 标准 debug 工作流

```bash
# 1. 起 chassis 周边（lightning loc / wheeltec / lidar / IMU）
#    要点：所有进程必须在同一 ROS_DOMAIN_ID（统一 1，机械臂组 2026-06-29 改）。
#    david_nav 侧改 domain 只动唯一真源 david_nav/config/ros_env.conf（脚本+systemd 都读它）。
ros2 run lightning run_loc_online --config ~/lightning_ws/.../default_c16.yaml &
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py &

# 2. 起 nav2（任选一）
~/ros2_ws/src/david_nav/scripts/launch_nav_with_log.sh use_rviz:=true enable_scan_bridge:=false
# 或带 bag 录制 + action goal：
~/ros2_ws/src/david_nav/navigate_semantic_pose_with_debug_bag.sh --target <pose_id>

# 3. RViz 起来后：
#    a) 点 "2D Pose Estimate" → lightning 收到 /initialpose → 开始发 map → odom
#    b) 点 "2D Nav Goal" 发目标位置

# 4. 跑完看：
#    /tmp/nav2_latest.log                    — 完整 launch + 节点输出
#    /tmp/david_nav_logs/brake_trace_*.csv   — cmd_vel 全链 + odom 数据
```

`/tmp/nav2_latest.log` 是 **symlink**，永远指向最新一次。任何 agent 直接
`tail -F` / `cat` 它就行，**不需要问用户日志路径**。

## 5. 最容易踩的 10 个坑（速查表）

| 症状 | 根因 | 修法 |
|---|---|---|
| RViz 不显示 map / `frame map does not exist` | ① 没在 RViz 点 "2D Pose Estimate" 给 lightning loc 初始位姿 ② ROS_DOMAIN_ID 不匹配（脚本默认强制 30 vs lightning 在 0） | ① 在 map 上点机器人位置 ② 检查 `cat /proc/<pid>/environ \| grep ROS_DOMAIN` |
| RViz 不显示 costmap | Map display 的 Durability 是 Volatile，但 nav2 用 Transient Local 发 | rviz config 改 `Durability Policy: Transient Local` |
| `Tf has two or more unconnected trees` | 多个 robot_state_publisher 同时跑（MoveIt URDF 残留 + nav2 URDF）| pre-flight 杀所有 RSP；nav2 launch 会起自己的 |
| nav2 起来 ~15 秒被全部 SIGINT | `.sh` 里 `verify_nav2_runtime_config` 硬编码 `local_costmap.global_frame == map`（错的，应该 `odom`）+ `set -e` | verify 改成 warning-only return 0 |
| DWB 完全不前进 | `max_vel_x` 与 `max_speed_xy` 不一致，端点被 isValidSpeed 过滤 | 两个值必须相等（同时还要 `min_vel_theta=0`） |
| 旋转 180° 但 ω 反复翻向、净旋转 ≈ 0 | DWB 在 180° 对称点上 +ω / 0 / -ω 等分 | separator 加 angular sign-latch（已实现） |
| `STATE LINEAR ↔ BRAKE_LINEAR` 高频切换 | DWB 单 tick `w≈0` 抽风 | `input_dead_persist_sec: 0.30`（时间滞回） |
| 终点冲过 1m 不停 | upstream 还在 buffer 老命令，input_dead 检测来得晚 | 距离-触发 1m brake：`dist_to_goal < 1.0m` 强制 BRAKE_LINEAR |
| `d2g` 在临门一脚突然变 n/a → brake 不触发 | `tf_buffer.transform()` 用 goal stamp 做 lookup，buffer 已经丢弃那个 stamp | transform 前把 stamp 改 0（让 TF 用 latest-available） |
| 键盘遥控不响应 | separator 在 IDLE 也 20Hz 发零速抢 `/cmd_vel` | `silent_on_idle: True`（IDLE 后 0.5s 不发）|

## 6. 关键参数速查

### nav2_params_2d.yaml 关键
```yaml
controller_server.ros__parameters.FollowPath:
  max_vel_x: 0.50
  max_speed_xy: 0.50           # 必须 == max_vel_x
  acc_lim_x: 2.5
  decel_lim_x: -2.5
  min_vel_theta: 0.0           # 关键：否则 binary 采样把 ω=0 那档过滤掉
  vx_samples: 2                # binary
  vtheta_samples: 3            # [-max, 0, +max]
  critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "GoalDist"]

general_goal_checker:
  xy_goal_tolerance: 0.50      # 必须 > FollowPath 内部 tolerance + 旋转后的横向偏移

velocity_smoother.ros__parameters:
  max_velocity: [0.50, 0.0, 0.5]   # [0] 同步 DWB max_vel_x
  max_accel:    [2.5,  0.0, 5.0]
  max_decel:    [-2.5, 0.0, -5.0]

local_costmap / global_costmap.ros__parameters:
  inflation_radius: 0.40       # 必须 ≥ inscribed (0.375)
```

### launch arg 默认
- `enable_scan_bridge: false`（按需开）
- `enable_collision_monitor: false`（debug 阶段关）
- `rviz_delay_sec: 12.0`（等 lifecycle Activated）

### cmd_vel_separator 生产参数
见 [nav 调参方法 §4](david_nav/docs/reports/2026-05-20-nav-tuning-for-heavy-load-low-torque-chassis.md#4-完整的-separator-参数表生产配置)。

## 7. 物理特性认知（决定上述配方的根本原因）

| 项 | 描述 | 影响 |
|---|---|---|
| 高负载差速底盘 | 70 kg 量级 | 启动需要克服大静摩擦，cmd 必须够大 |
| 小电机 | 力矩有限 | 加速度受限，cmd 高到一定程度后 actual 饱和 |
| 纯 P 电机控制 (STM32) | 没有 I 项消除稳态 | cmd 0.7 实测 0.4-0.5，跟踪误差大 |
| 被动万向轮 (caster) | 拖尾 + 横向静摩擦 | 旋转后给小线速度时 caster 卡住一侧驱动轮；旋转停止时机身按动量惯性过冲 ~20-45° |

这套硬件是 **separator 所有适配逻辑的存在理由**。STM32 改 PID 之后很多 brake
设定可以放松甚至关掉，但目前以 P 控为准。

## 8. 项目内常见路径

```
~/ros2_ws/src/david_nav/
  ├── david_nav/                          # python nodes
  │   ├── cmd_vel_separator.py            # ★ 适配层核心，反复改的就是这个
  │   ├── semantic_pose_navigator.py      # action goal sender
  │   └── ...
  ├── config/
  │   ├── nav2_params_2d.yaml             # ★ DWB / costmap / smoother 参数
  │   ├── collision_monitor_params.yaml   # 当前 polygons 禁用
  │   └── ...
  ├── launch/
  │   └── 2d_david_nav2_bringup.launch.py # ★ 主 bringup
  ├── scripts/
  │   ├── launch_nav_with_log.sh          # ★ 包装 launch + 自动 log + 自动 record
  │   ├── refresh_nav_log_symlink.sh
  │   └── record_brake_trace.py           # CSV trace recorder
  ├── navigate_semantic_pose_with_debug_bag.sh   # ★ 带 bag 录制的完整启动
  ├── rviz/
  │   └── nav2_go2_view.rviz              # Map display 都是 Transient Local
  ├── urdf/
  │   └── david.urdf                      # chassis-only URDF
  └── docs/reports/                       # 所有调试报告

~/ros2_ws/src/david_description/urdf/david.urdf  # 全机 URDF（含 world 节点，
                                                   注意冲突 ↓）

~/ros2_ws/src/david_moveit_config/config/david.urdf.xacro
                                          # MoveIt URDF（include david_description
                                          # 那个，所以也带 world 节点）
```

## 9. 重要外部依赖

不在 workspace 里、但 nav 跑起来依赖：

```
~/lightning_ws/                           # SLAM / 定位
  src/hoslam_lightning_lm/config/default_c16.yaml
  install/lightning/lib/lightning/{run_slam_online, run_loc_online}

~/wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2/   # 底盘驱动
  src/turn_on_wheeltec_robot/launch/turn_on_wheeltec_robot.launch.py
  src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp
```

## 10. Git commit 习惯

- 一次 commit 一个独立改动；commit message 用中文 / 英文均可但要说明：**改了什
  么 + 为什么这样改 + 影响范围**
- nav 相关 commit 默认 push 到 `feature/david_main`（参考 `git log`）
- Co-Author 行：`Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>`

## 11. 推荐的迭代节奏

1. **写改动 → 在 david_nav 重 `colcon build` → 干跑 `ast/yaml` 验证**
2. **跑一次** `.sh` 或 `launch_nav_with_log.sh`，读 `/tmp/nav2_latest.log`
3. 看 separator 的 `STATE` 转换 + `OUT mode=` 速度数据
4. 看 `brake_trace_*.csv` 在 goal 附近的几行确认 brake 链路
5. 一组改动稳了再 commit + push
