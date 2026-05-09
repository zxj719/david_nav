# Realsense 局部避障接入 Nav2 计划

## 目标

在不修改 `hoslam_lightning_lm` 的前提下，把机器人上的 Realsense 深度相机接入 `g1_nav`，让 Nav2 能使用实时的局部障碍观测来实现近距离避障。

优先采用的链路为：

```text
Realsense 深度图 -> depthimage_to_laserscan -> /scan -> Nav2 local costmap obstacle layer
```

这个方案第一阶段明确不使用 `voxel_layer`，也不依赖 Realsense 点云输出。

## 为什么优先用这个方案

相比直接走 `PointCloud2 -> voxel_layer`，当前阶段更推荐这个方案，原因是：

- Realsense 的深度图已经在机器人上正常发布
- `depthimage_to_laserscan` 已经安装在系统 ROS 环境里
- Nav2 通过 `/scan` 接入比 `voxel_layer` 更简单、更轻量
- 参数更少，更容易调通，也更容易排查问题
- `g1_nav` 里本身已经有 `/scan` 接入的结构基础

## 当前已知情况

### 当前可用的 Realsense 话题

目前已经看到这些 Realsense 相关话题：

- `/camera/camera/depth/image_rect_raw` (`sensor_msgs/msg/Image`)
- `/camera/camera/depth/camera_info` (`sensor_msgs/msg/CameraInfo`)
- `/camera/camera/color/image_raw`
- `/camera/camera/color/camera_info`

深度图是活的，深度图 header 里的坐标系为：

- `camera_depth_optical_frame`

### 当前 Realsense 状态

- Realsense 节点为 `/camera/camera`
- 深度流已开启
- 彩色流已开启
- Realsense 点云输出当前未开启：
  - `pointcloud__neon_.enable = false`

### 当前 `g1_nav` 里的 Nav2 状态

当前本地代价地图配置文件是：

- `config/nav2_params_2d.yaml`

其中 local costmap 目前使用的是：

- `static_layer`
- `inflation_layer`

也就是说，当前还没有接入实时局部障碍传感器。

另外，在：

- `launch/2d_g1_nav2_bringup.launch.py`

里已经有一个现成的 `/scan` 桥接位置，目前是把：

- `/utlidar/cloud_livox_mid360`

通过：

- `pointcloud_to_laserscan`

转成 `/scan`。

## 工作范围

### 本次计划包含

- 在 `g1_nav` 中加入 Realsense 深度图转 `/scan`
- 让 Nav2 local costmap 使用 `/scan` 做局部障碍观测
- 保留当前的 SLAM 静态图层
- 让这套接入方式可以通过 launch 参数切换
- 验证 TF、话题和实际避障行为

### 本次计划不包含

- 修改 `hoslam_lightning_lm`
- 重写 SLAM 或地图链路
- 改动 global costmap 的主要逻辑
- 第一阶段接入 `voxel_layer`
- 大范围重调 controller 参数

## 预计会改动的文件

如果开始实现，预计主要会改这几个文件：

- `launch/2d_g1_nav2_bringup.launch.py`
- `launch/g1_auto_explore.launch.py`
- `config/nav2_params_2d.yaml`

如果后面确实有必要，可能还会新增：

- 一个独立的深度图转 `/scan` launch 文件
- RViz 配置中对 `/scan` 或 local costmap 的显示项

## 集成设计

### 1. 新增 Realsense 深度图到 `/scan` 的桥接

在 `g1_nav` 的 launch 中加入：

- `depthimage_to_laserscan/depthimage_to_laserscan_node`

输入：

- `/camera/camera/depth/image_rect_raw`
- `/camera/camera/depth/camera_info`

输出：

- `/scan`

建议的初始参数：

- `scan_time: 0.2`
- `range_min: 0.2`
- `range_max: 3.0`
- `scan_height: 3`
- `output_frame: camera_depth_optical_frame`

这样设置的原因：

- `range_min` 既要支持近距离避障，又不能太小导致机器人一直看到自身噪声
- `range_max` 只需要覆盖局部避障的短距离范围
- `scan_height` 适当大于 `1` 可以降低单行深度图噪声

### 2. 把 scan 来源做成可配置

当前 bringup 里已经有：

- `enable_scan_bridge`

但它只控制 lidar 点云转 `/scan`。

后续应把 scan 来源做成明确可切换的模式，例如：

- 关闭
- lidar 点云桥接
- realsense 深度桥接

这样就不需要以后每次手改 launch 文件，也方便同一套系统在不同传感器来源之间切换。

### 3. 在 Nav2 local costmap 中启用障碍层

在 `config/nav2_params_2d.yaml` 里，让 local costmap 保留：

- `static_layer`
- `inflation_layer`

同时增加：

- `obstacle_layer`

本地代价地图的插件逻辑应变成：

```yaml
plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

障碍观测源使用：

- `/scan`
- `data_type: "LaserScan"`

这样既保留 SLAM 实时地图，又能加入近距离动态障碍标记与清除。

### 4. 第一阶段不改 global costmap

global costmap 第一阶段保持现状，只基于 SLAM 地图工作。

原因：

- 回归风险更小
- 当前真正要解决的问题是局部避障
- 没必要在第一阶段把临时局部障碍混进全局层

### 5. 在期望 Nav2 正常工作前，先确认 TF

这套接入依赖于以下 TF 链路存在并且正确：

- `camera_depth_optical_frame`
- `base`
- `odom`

如果 TF 不对，常见后果有：

- `/scan` 虽然发布了，但 Nav2 用不了
- 障碍物在 local costmap 里位置漂移
- 机器人转动时障碍跟着错位

所以 TF 检查不是收尾工作，而是接入过程中的关键步骤。

## 详细工作计划

### 阶段 1：Launch 接入

1. 在 `launch/2d_g1_nav2_bringup.launch.py` 里增加 Realsense 深度桥接节点
2. 把相关 launch 参数接到顶层 bringup
3. 在 `launch/g1_auto_explore.launch.py` 中透传这些参数
4. 保留现有 lidar 的 scan bridge，不直接删掉

阶段目标：

- 能以 Realsense 深度桥模式启动 `g1_nav`
- 启动后能看到 `/scan`

### 阶段 2：本地代价地图接入

1. 在 local costmap 启用 `obstacle_layer`
2. 让 `obstacle_layer` 订阅 `/scan`
3. 保留 `static_layer`，保证 local planner 仍然尊重实时 SLAM 栅格图
4. 保留 `inflation_layer`，让障碍物形成安全缓冲带

阶段目标：

- local costmap 同时包含地图障碍和实时近距离障碍

### 阶段 3：TF 与运行时验证

1. 确认 `/scan` 正在持续发布
2. 确认 scan frame 能变换到 `base` 和 `odom`
3. 在 RViz 中确认 local costmap 会随着近处障碍变化
4. 确认 controller 会对近障产生规避行为，而不是继续沿旧地图直冲

阶段目标：

- 新的观测链路在真实运行图里可用

### 阶段 4：最小必要调参

1. 调 `depthimage_to_laserscan`：
   - `range_min`
   - `range_max`
   - `scan_height`
2. 调 local obstacle layer：
   - `obstacle_max_range`
   - `obstacle_min_range`
   - `raytrace_max_range`
   - `raytrace_min_range`
3. 如果确实还有贴障问题，再微调 local inflation

阶段目标：

- 近距离避障稳定，误报可控

## 建议的初始参数策略

### depthimage_to_laserscan

先从保守值开始：

- `range_min: 0.2`
- `range_max: 3.0`
- `scan_height: 3`

然后根据深度噪声和刹停距离再逐步调整。

### local obstacle layer

建议起始值：

- `obstacle_min_range: 0.2`
- `obstacle_max_range: 2.5`
- `raytrace_min_range: 0.2`
- `raytrace_max_range: 3.0`

这些值面向的是局部避障，不是远距离路径规划。

## 验证清单

### 话题层验证

- `/camera/camera/depth/image_rect_raw` 持续有数据
- `/camera/camera/depth/camera_info` 持续有数据
- 启动桥接后出现 `/scan`
- `/scan` 发布频率足以支撑局部避障

### TF 验证

- `camera_depth_optical_frame -> base` 存在
- `base -> odom` 存在
- Nav2 不再报 `/scan` 相关 TF timeout

### Nav2 验证

- 把障碍物放到相机前方时，local costmap 能实时变化
- 控制器会减速、绕避或重新调整局部路径
- 启用局部障碍感知后，探索主流程仍然正常

### 回归验证

- global planner 仍然主要基于 `/lightning/grid_map`
- frontier exploration 仍然可运行
- 启用 Realsense 局部避障后，现有 SLAM 导航链路不被破坏

## 风险与应对

### 风险 1：相机到 base 的 TF 缺失

影响：

- 障碍物无法正确投影到 local costmap

应对：

- 先确认 TF
- 只有在确实缺失时，才在 `g1_nav` 的 launch 范围内补一个静态 TF

### 风险 2：机器人近处深度噪声较大

影响：

- local costmap 误报
- 障碍不稳定闪烁

应对：

- 初始 `range_min` 不设得过小
- `scan_height` 不用 `1`
- 先调观测范围，再碰规划或控制参数

### 风险 3：深度相机视场比激光更窄

影响：

- 视场外障碍不会被看到

应对：

- 把它定位成前向近距离避障补充，不是 360 度替代方案
- 保留现有静态 SLAM 地图

### 风险 4：障碍层让探索行为变得过于保守

影响：

- 局部规划可能更容易拒绝窄通道

应对：

- 第一阶段只改 local costmap
- 先调障碍层距离参数，不先改 BT 或 controller 逻辑

## 验收标准

满足以下条件即可认为集成成功：

1. `g1_nav` 能以 Realsense 为输入产生 `/scan`
2. Nav2 local costmap 能显示来自深度相机的实时局部障碍
3. 机器人会对前方近距离障碍作出避让
4. 全局规划和 frontier exploration 仍然可用
5. 整个过程不需要修改 `hoslam_lightning_lm`

## 推荐实施顺序

实际实现时建议严格按这个顺序来：

1. 先加 Realsense 深度图转 `/scan`
2. 验证 `/scan`
3. 再启用 local `obstacle_layer`
4. 验证 local costmap
5. 验证真实避障行为
6. 最后只调最少必要参数

这样做能最大限度减少排查时的混乱，把问题分层隔离开。

## 补充说明

- 后续如果确实需要，也可以再开启 Realsense 点云输出，但这不是第一阶段成功接入所必需的
- 如果未来需要更丰富的 3D 障碍表达，再考虑第二阶段迁移到 `voxel_layer`
- 第一阶段最重要的是简单、稳定、好观测，而不是一步到位堆功能
