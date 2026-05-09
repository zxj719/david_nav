# g1_nav

`g1_nav` 是面向 Unitree G1 的 ROS 2 导航与探索包，当前主要围绕以下几件事展开：

- 基于 Nav2 的 2D 导航
- 基于 frontier 的自主探索
- 与外部 Lightning SLAM 地图/里程计话题对接
- 使用 Realsense depth-to-scan 或激光点云桥接做局部避障输入
- 面向现场调参与问题定位的 RViz 可视化与调试工具

这个仓库目前更适合作为研发与现场调试用导航仓库，而不是“已经完全闭环、无人干预即可稳定完成建图”的成品系统。

## 系统组成

### 1. Nav2 导航底座

`g1_nav` 使用 Nav2 作为导航执行核心，提供：

- `NavigateToPose`
- 局部/全局 costmap
- 行为树恢复逻辑
- RViz 调试入口

主要入口：

- [2d_g1_nav2_bringup.launch.py](/home/unitree/ros2_ws/src/g1_nav/launch/2d_g1_nav2_bringup.launch.py)
- [nav2_params_2d.yaml](/home/unitree/ros2_ws/src/g1_nav/config/nav2_params_2d.yaml)

### 2. Frontier Explorer

自主探索逻辑由 `frontier_explorer_cpp` 提供，核心能力包括：

- frontier 搜索
- frontier 排序
- frontier 非极大值抑制
- snapped nav goal 选择
- 黑名单与完成状态管理
- 运行时 preempt / abort 结果解释

相关文件：

- [frontier_explorer.cpp](/home/unitree/ros2_ws/src/g1_nav/src/frontier_explorer.cpp)
- [frontier_search.cpp](/home/unitree/ros2_ws/src/g1_nav/src/frontier_search.cpp)
- [frontier_goal_selector.cpp](/home/unitree/ros2_ws/src/g1_nav/src/frontier_goal_selector.cpp)
- [frontier_explorer_params.yaml](/home/unitree/ros2_ws/src/g1_nav/config/frontier_explorer_params.yaml)

### 3. 外部地图与传感器对接

当前导航链路依赖外部系统提供的地图和位姿话题，典型输入包括：

- `/lightning/grid_map`
- `/lightning/odometry`
- `/tf`
- `/tf_static`

局部避障输入可以来自两种桥接方式：

- Livox 点云转 `/scan`
- Realsense 深度图转 `/scan`

相关 launch：

- [2d_g1_nav2_bringup.launch.py](/home/unitree/ros2_ws/src/g1_nav/launch/2d_g1_nav2_bringup.launch.py)
- [realsense_depth_to_scan.launch.py](/home/unitree/ros2_ws/src/g1_nav/launch/realsense_depth_to_scan.launch.py)

### 4. G1 控制链路

当前 bringup 会同时拉起 `g1_cmd` 包内的 `g1_move` 节点，让导航输出最终能够进入 G1 控制链路。

## 依赖与环境

当前仓库按 ROS 2 Humble / `ament_cmake` 工程组织，核心依赖见 [package.xml](/home/unitree/ros2_ws/src/g1_nav/package.xml)，包括：

- `nav2_msgs`
- `nav2_costmap_2d`
- `nav2_behavior_tree`
- `rclcpp`
- `rclcpp_action`
- `tf2_ros`
- `visualization_msgs`
- `geometry_msgs`
- `nav_msgs`
- `unitree_go`
- `unitree_api`
- `unitree_interfaces`
- `custom_interface`
- `go2_sport`

运行时通常还需要这些外部包或系统：

- `nav2_bringup`
- `pointcloud_to_laserscan`
- `depthimage_to_laserscan`
- `g1_cmd`
- `g1_sim`
- 外部 Lightning SLAM 系统

## 快速开始

### 1. 构建

在工作空间根目录执行：

```bash
cd /home/unitree/ros2_ws
colcon build --packages-select g1_nav
source install/setup.bash
```

### 2. 纯导航 bringup

这条命令更适合做“给定目标点能不能稳定导航”的验证：

```bash
ros2 launch g1_nav 2d_g1_nav2_bringup.launch.py use_rviz:=true
```

常用开关：

- `enable_scan_bridge:=true`
  使用 Livox 点云转 `/scan`
- `enable_realsense_scan_bridge:=true`
  使用 Realsense 深度图转 `/scan`
- `enable_exploration:=true`
  启动 frontier explorer

### 3. 自动探索

这条命令会启动 Nav2、frontier explorer，以及自动探索所需的探索行为树配置：

```bash
ros2 launch g1_nav g1_auto_explore.launch.py
```

默认行为：

- `use_rviz:=true`
- `enable_realsense_scan_bridge:=true`

### 4. 单独启动 Realsense depth-to-scan

适合单独验证 Realsense 深度桥接是否正常：

```bash
ros2 launch g1_nav realsense_depth_to_scan.launch.py
```

默认输入话题：

- `/camera/camera/depth/image_rect_raw`
- `/camera/camera/depth/camera_info`

默认输出话题：

- `/scan`

## 常用运行方式

### 场景 A：只验证 Nav2

目标：确认纯导航链路、RViz、局部/全局 costmap 是否正常。

建议：

```bash
ros2 launch g1_nav 2d_g1_nav2_bringup.launch.py \
  use_rviz:=true \
  enable_exploration:=false
```

### 场景 B：验证自主探索

目标：观察 frontier 搜索、goal snap、blacklist、preempt 行为。

建议：

```bash
ros2 launch g1_nav g1_auto_explore.launch.py \
  use_rviz:=true
```

### 场景 C：验证 Realsense 局部避障输入

目标：只看 depth-to-scan 是否把近场障碍稳定转成 `/scan`。

建议：

```bash
ros2 launch g1_nav realsense_depth_to_scan.launch.py
```

## 关键配置文件

### Nav2 参数

- [nav2_params_2d.yaml](/home/unitree/ros2_ws/src/g1_nav/config/nav2_params_2d.yaml)

主要关注：

- `local_costmap`
- `global_costmap`
- `inflation_layer`
- 行为树默认 XML

### Frontier Explorer 参数

- [frontier_explorer_params.yaml](/home/unitree/ros2_ws/src/g1_nav/config/frontier_explorer_params.yaml)

主要关注：

- `search_free_threshold`
- `min_frontier_size`
- `frontier_snap_radius`
- `goal_clearance_radius`
- `planner_frequency`
- `blacklist_radius`

### 行为树

- [navigate_to_pose_w_g1_embedded_recovery.xml](/home/unitree/ros2_ws/src/g1_nav/behavior_trees/navigate_to_pose_w_g1_embedded_recovery.xml)
- [navigate_through_poses_w_g1_embedded_recovery.xml](/home/unitree/ros2_ws/src/g1_nav/behavior_trees/navigate_through_poses_w_g1_embedded_recovery.xml)
- [navigate_to_pose_w_g1_explore_fail_fast.xml](/home/unitree/ros2_ws/src/g1_nav/behavior_trees/navigate_to_pose_w_g1_explore_fail_fast.xml)

## RViz 调试

### 1. 已有导航目标可视化

`frontier_explorer` 会发布 frontier 相关 marker 到：

- `explore/frontiers`

### 2. Frontier 调参可视化

当前 `explore/frontiers` 中包含 3 层 marker：

- `frontier_cells`
  原始 frontier 搜索结果中的每个 frontier cell
- `frontier_centroids`
  每个 frontier cluster 的质心
- `frontier_goals`
  最终经过筛选与 snap 后的导航目标

这三层的意义不同：

- 调 `search_free_threshold` 时，优先看 `frontier_cells` 和 `frontier_centroids`
- 看系统最终准备导航到哪里时，优先看 `frontier_goals`

其中 `frontier_cells` 和 `frontier_centroids` 基于原始 frontier 搜索结果，不经过 blacklist、NMS 和 goal snap 的后处理，适合观察 frontier 检测本身是否偏多、偏少或落在墙内。

## 当前状态与已知限制

截至当前版本，这个仓库已经形成了较清晰的 frontier 与 Nav2 分层策略，但仍存在以下限制：

1. 机器人尚不能在完全无人干预下稳定完成一次自主建图
2. 3D LiDAR 前端地面点和地图燥点问题仍会影响建图质量
3. 动态障碍物处理仍不充分，主要依赖 Realsense 与较快重规划来缓解
4. 机器人被误判进入障碍物后的脱困机制仍不完整
5. 电梯门等特殊材质环境下仍可能出现 frontier 误引导
6. 建图完成后自动返回初始点能力尚未完全闭环

因此，当前仓库更适合：

- 研发调试
- 现场实验
- 参数迭代
- 方法论验证

而不应被视为已经完全产品化的自主探索方案。

## 仓库结构

```text
g1_nav/
├── behavior_trees/              # Nav2 行为树 XML
├── config/                      # Nav2 与 frontier explorer 参数
├── docs/                        # 设计稿、计划稿、阶段报告
├── g1_nav/                      # Python 节点与脚本
├── include/g1_nav/              # C++ 头文件
├── launch/                      # 启动文件
├── rviz/                        # RViz 配置
├── src/                         # C++ 实现
└── test/                        # gtest / pytest
```

## 文档入口

### 阶段报告

- [2026-04-03-g1-navigation-methodology-report.md](/home/unitree/ros2_ws/src/g1_nav/docs/reports/2026-04-03-g1-navigation-methodology-report.md)

### 设计稿

- [2026-03-27-probability-map-frontier-design.md](/home/unitree/ros2_ws/src/g1_nav/docs/superpowers/specs/2026-03-27-probability-map-frontier-design.md)
- [2026-03-30-frontier-goal-snap-clearance-design.md](/home/unitree/ros2_ws/src/g1_nav/docs/superpowers/specs/2026-03-30-frontier-goal-snap-clearance-design.md)
- [2026-03-30-nav2-inflation-radius-tuning-design.md](/home/unitree/ros2_ws/src/g1_nav/docs/superpowers/specs/2026-03-30-nav2-inflation-radius-tuning-design.md)

### 计划稿

- [2026-03-27-probability-map-frontier-plan.md](/home/unitree/ros2_ws/src/g1_nav/docs/superpowers/plans/2026-03-27-probability-map-frontier-plan.md)
- [2026-03-30-frontier-goal-snap-clearance-plan.md](/home/unitree/ros2_ws/src/g1_nav/docs/superpowers/plans/2026-03-30-frontier-goal-snap-clearance-plan.md)
- [2026-03-30-nav2-inflation-radius-tuning-plan.md](/home/unitree/ros2_ws/src/g1_nav/docs/superpowers/plans/2026-03-30-nav2-inflation-radius-tuning-plan.md)

## 测试

常用验证命令：

```bash
cd /home/unitree/ros2_ws
colcon build --packages-select g1_nav
colcon test --packages-select g1_nav --event-handlers console_direct+
colcon test-result --verbose
```

最近一次本地验证结果：

- `colcon build --packages-select g1_nav` 通过
- `colcon test --packages-select g1_nav --event-handlers console_direct+` 通过
- `colcon test-result --verbose` 结果为 `35 tests, 0 errors, 0 failures, 0 skipped`
