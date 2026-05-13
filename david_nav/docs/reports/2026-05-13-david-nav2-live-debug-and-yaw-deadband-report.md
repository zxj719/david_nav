# 2026-05-13 David Nav2 现场调试与 yaw 死区修复报告

## 概要

本 session 处理的问题是：

- `ros2 launch david_nav 2d_david_nav2_bringup.launch.py use_rviz:=true enable_scan_bridge:=true` 后，机器人无法稳定导航到前方约 1 m 目标点。
- 现场日志出现 `FollowPath` abort、`Failed to make progress`。
- 机器人能收到上游 `cmd_vel`，但底盘没有响应或掉头阶段卡住。
- 后续明确要求暂不处理 Realsense，当前方案优先基于 LiDAR/Lightning 链路。

本轮结论：

- Nav2 上游速度链路可以产生非零速度。
- 之前 `david_move` 被 launch remap 成“订阅 `/cmd_vel_safe`，又发布回 `/cmd_vel_safe`”，导致最终 `/cmd_vel` 没有发布者。
- 修正 remap 后，`david_move` 正确发布 `/cmd_vel`，但现场当时没有底盘驱动节点订阅 `/cmd_vel`。
- 用户提供的底盘入口是 `turn_on_wheeltec_robot.launch.py`，不是 `scout_base`。后续必须同时启动 wheeltec 底盘驱动。
- 掉头问题参考 `origin/feature/g1-embodied-navigation-import`，核心修复不是改 DWB，而是在底盘桥接层把原地小角速度抬升到硬件可执行角速度。

## 现场排查链路

### Nav2 action abort

现场日志里关键失败模式：

- `controller_server`: `[follow_path] [ActionServer] Aborting handle`
- `controller_server`: `Failed to make progress`

这类日志说明 Nav2 控制器已经开始执行路径，但 odom/TF 反馈没有体现足够位移，最终触发 progress checker 超时。

### 速度 topic 链路

本轮期望链路调整为：

```text
controller_server / behavior_server
  -> /cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel_smoothed
  -> collision_monitor
  -> /cmd_vel_safe
  -> david_move
  -> /cmd_vel
  -> wheeltec_robot
```

现场验证时，上游 topic 有非零速度：

- `/cmd_vel_nav`: 有 controller 输出。
- `/cmd_vel_smoothed`: 有 velocity smoother 输出。
- `/cmd_vel_safe`: 有 collision monitor 输出。

但旧 launch 状态下：

```text
/david_move
  Subscribers:
    /cmd_vel_safe
  Publishers:
    /cmd_vel_safe
```

问题是 `2d_david_nav2_bringup.launch.py` 中旧的：

```python
remappings=[('cmd_vel', 'cmd_vel_safe')]
```

会把 `david_move` 的输出 `/cmd_vel` 也 remap 掉，最终没有节点发布 `/cmd_vel`。

修复后验证：

```text
/david_move
  Subscribers:
    /cmd_vel_safe
  Publishers:
    /cmd_vel
```

### 底盘驱动缺失

修复 `david_move` 输出后，现场继续检查：

```text
/cmd_vel
  Publisher count: 1
  Publisher: david_move
  Subscription count: 0
```

这说明 Nav2 和 `david_move` 已经把最终速度发出来，但当时 ROS 图里没有底盘驱动节点订阅 `/cmd_vel`。

一开始系统中看到 `scout_base`，但用户随后明确给出实际底盘启动入口：

```bash
cd ~/wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2/
source ./install/setup.bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

该 launch 启动：

- `wheeltec_robot_node`
- `twist_transform`

其中 `wheeltec_robot_node` 订阅 `cmd_vel`，发布 `odom`，并通过 `/dev/wheeltec_controller` 串口写底盘控制帧。

## 已实施改动

### 1. Nav2 显式速度链路

文件：

- `david_nav/launch/2d_david_nav2_bringup.launch.py`

本轮将 Nav2 bringup 从直接 include `nav2_bringup/navigation_launch.py` 调整为显式启动主要 Nav2 lifecycle nodes，以避免默认 velocity smoother 把输出直接 remap 到 `/cmd_vel`。

关键效果：

- `controller_server` 输出 remap 到 `cmd_vel_nav`
- `behavior_server` 输出 remap 到 `cmd_vel_nav`
- `bt_navigator` 输出 remap 到 `cmd_vel_nav`
- `velocity_smoother` 输入 remap 到 `cmd_vel_nav`
- `velocity_smoother` 默认输出保留为 `/cmd_vel_smoothed`
- `collision_monitor` 处理 `/cmd_vel_smoothed` 后输出 `/cmd_vel_safe`
- `david_move` 订阅 `/cmd_vel_safe`，发布 `/cmd_vel`

### 2. Collision monitor 输入改为 smoothed cmd_vel

文件：

- `david_nav/config/collision_monitor_params.yaml`

关键参数：

```yaml
cmd_vel_in_topic: "cmd_vel_smoothed"
cmd_vel_out_topic: "cmd_vel_safe"
```

目的：

- 保持 `velocity_smoother -> collision_monitor -> david_move -> wheeltec_robot` 的顺序。
- 避免未平滑速度或多个节点竞争写 `/cmd_vel`。

### 3. RViz LaserScan QoS

文件：

- `david_nav/rviz/nav2_go2_view.rviz`

将 `/scan` 显示的可靠性从 Reliable 调整为 Best Effort，匹配现场 `pointcloud_to_laserscan` / costmap / collision monitor 的 LaserScan QoS。

### 4. `david_move` timeout 与速度死区

文件：

- `david_cmd/david_cmd/david_move.py`
- `david_cmd/config/cmd.yaml`
- `david_cmd/test/test_david_move_node.py`

当前关键参数：

```yaml
cmd_vel_timeout: 2.0
min_linear_x: 0.20
max_linear_x: 0.20
min_angular_z: 0.20
min_stationary_angular_z: 0.35
stationary_angular_threshold: 0.025
max_angular_z: 0.50
publish_hz: 50.0
```

行为：

- 非零 `linear.x` 小于 `0.20` 时抬升到 `±0.20`。
- 非零 `angular.z` 小于 `0.20` 时抬升到 `±0.20`。
- 原地掉头时，如果 `abs(angular.z) >= 0.025`，抬升到 `±0.35`。
- `angular.z == 0` 保持为 0，不误触发。
- 命令超过 `2.0 s` 未更新后清零。

## G1 分支参考结论

参考分支：

```text
remotes/origin/feature/g1-embodied-navigation-import
```

重点提交：

```text
ee2f9a5 Tune G1 yaw deadband response
```

该提交处理的现场现象：

- Nav2 在掉头阶段会输出很小的原地角速度，例如 `angular.z ~= 0.029 rad/s`。
- 如果底盘桥接层把这类小角速度压住或延迟补偿，机器人会出现掉头前卡顿。
- 修复策略是在底盘桥接层加入原地 yaw 的单独阈值，把 `~0.029 rad/s` 这类命令立即抬升到可执行角速度。

本轮 David 采用的对应策略：

- 普通边走边转仍用 `min_angular_z = 0.20`。
- 原地掉头使用更高的 `min_stationary_angular_z = 0.35`。
- 触发阈值使用 `stationary_angular_threshold = 0.025`。

这比单纯提高 Nav2 `min_vel_theta` 更直接，因为问题发生在硬件执行死区和桥接层，而不是只发生在 DWB 采样层。

## 验证结果

已执行：

```bash
colcon build --packages-select david_cmd david_nav --symlink-install
```

结果：

```text
Summary: 2 packages finished
```

已执行：

```bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/david_cmd/test/test_david_move_node.py
```

结果：

```text
9 passed
```

手动验证 `david_move` 抬升逻辑：

```text
angular.z = 0.029, linear.x = 0.0  -> angular.z = 0.35
angular.z = 0.029, linear.x = 0.1  -> linear.x = 0.20, angular.z = 0.20
```

## 当前启动前提

完整现场验证时需要同时启动底盘驱动和 Nav2。

底盘：

```bash
cd ~/wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2/
source ./install/setup.bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

导航：

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch david_nav 2d_david_nav2_bringup.launch.py use_rviz:=true enable_scan_bridge:=true
```

现场验证时必须确认：

```bash
ros2 topic info -v /cmd_vel
```

应至少满足：

- Publisher: `david_move`
- Subscriber: `wheeltec_robot`

同时确认速度链路：

```bash
ros2 topic echo /cmd_vel_nav --once
ros2 topic echo /cmd_vel_smoothed --once
ros2 topic echo /cmd_vel_safe --once
ros2 topic echo /cmd_vel --once
```

## 剩余风险

### 1. 当前未完成完整实机导航闭环验证

本轮完成了链路定位、代码修复、单测和 build，但 session 中没有在 wheeltec 底盘驱动已启动的条件下完成一次完整的 RViz 目标点导航闭环。

### 2. Collision monitor 仍可能受 TF 时间影响

现场曾出现：

```text
Failed to get "base"->"base_footprint" frame transform:
Lookup would require extrapolation into the future
```

如果后续 `/cmd_vel_safe` 有时缺失或变 0，需要继续抓：

- `/cmd_vel_smoothed`
- `/cmd_vel_safe`
- `/scan.header.frame_id`
- `odom -> base_footprint` / `base -> base_footprint` TF 时间

### 3. `/cmd_vel` 订阅者是硬前提

如果 `/cmd_vel` 仍显示 `Subscription count: 0`，不要继续调 Nav2 参数。先启动或修复 `wheeltec_robot_node`。

### 4. Realsense 暂不纳入本轮方案

用户已明确说明暂不处理 Realsense，后续方案可能删除 Realsense。因此本报告不把 Realsense 作为当前导航链路的前提。

