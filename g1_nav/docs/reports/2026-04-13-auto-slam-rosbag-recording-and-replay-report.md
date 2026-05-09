# 2026-04-13 自动探索项目 Rosbag 录制与回放报告

## 目标

这份报告回答两个问题：

1. 为 `g1_nav` 当前自动探索链路，最少应该录哪些 topic，才能在另一台机器上稳定回放
2. 如果希望后续既能复现实验，又能做问题定位，应该把哪些派生 topic 一并录下来

本报告面向当前仓库的自动探索链路：

- 外部 Lightning SLAM 提供地图与里程计
- Nav2 负责规划、控制、costmap 与 recovery
- `frontier_explorer_cpp` 负责 frontier 搜索与目标选择
- 局部避障输入来自 `/scan`

## 当前工程的数据链路

按照仓库当前配置，自动探索的核心输入链路是：

- `/tf`
- `/tf_static`
- `/lightning/odometry`
- `/lightning/grid_map`
- `/scan`

其中：

- `Nav2` 明确消费 `/tf(map/odom/base)`、`/lightning/odometry`、`/lightning/grid_map`、`/scan`
- `frontier_explorer` 消费 `/lightning/odometry`、`/lightning/grid_map`、`/global_costmap/costmap`
- `global_costmap/costmap` 本身由 Nav2 在运行时根据 `/lightning/grid_map` 与 `/scan` 生成

因此，从“可回放”角度看，真正必须录的是上游输入，不是所有中间产物。

## 结论

### 1. 最小可回放 bag

如果目标是“重新跑起自动探索并让系统自己重算 frontier、costmap、规划与控制”，推荐录下面这组：

```bash
ros2 bag record -o g1_auto_explore_minimal \
  /tf \
  /tf_static \
  /lightning/odometry \
  /lightning/grid_map \
  /scan
```

这组 bag 适合：

- 复现自动探索行为
- 重新生成 local/global costmap
- 重新计算 frontier
- 回放 Nav2 与 frontier 逻辑

这组 bag 不适合：

- 调试 `/scan` 是如何从原始传感器桥接出来的
- 追溯 “为什么这一次 `/scan` 本身就错了”

### 2. 原始传感器可回放 bag

如果你希望在回放时连传感器桥接也一起重现，那么不要直接录 `/scan`，而是录 `/scan` 的上游原始输入。

#### Realsense 路线

当前 Realsense bridge 的输入是：

- `/camera/camera/depth/image_rect_raw`
- `/camera/camera/depth/camera_info`

对应录包建议：

```bash
ros2 bag record -o g1_auto_explore_realsense_raw \
  /tf \
  /tf_static \
  /lightning/odometry \
  /lightning/grid_map \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info
```

适用场景：

- 想回放 `depthimage_to_laserscan`
- 想检查深度图到 `/scan` 的转换是否稳定
- 想在不同机器上重跑相同桥接参数

#### Livox 路线

当前 Livox bridge 的输入是：

- `/utlidar/cloud_livox_mid360`

对应录包建议：

```bash
ros2 bag record -o g1_auto_explore_livox_raw \
  /tf \
  /tf_static \
  /lightning/odometry \
  /lightning/grid_map \
  /utlidar/cloud_livox_mid360
```

适用场景：

- 想回放 `pointcloud_to_laserscan`
- 想检查点云桥接参数是否合理
- 想比较不同 `/scan` 生成策略

### 3. 全量调试 bag

如果目标不只是“能回放”，而是“回放后保留足够多的诊断信息”，建议在最小可回放 bag 基础上补充以下 topic：

- `/global_costmap/costmap`
- `/local_costmap/costmap`
- `/local_costmap/costmap_raw`
- `/local_costmap/published_footprint`
- `/explore/frontiers`
- `/cmd_vel`
- `/cmd_vel_safe`
- `/cmd_vel_nav`
- `/map_live_ready`
- `/explore/resume`

推荐命令：

```bash
ros2 bag record -o g1_auto_explore_debug \
  /tf \
  /tf_static \
  /lightning/odometry \
  /lightning/grid_map \
  /scan \
  /global_costmap/costmap \
  /local_costmap/costmap \
  /local_costmap/costmap_raw \
  /local_costmap/published_footprint \
  /explore/frontiers \
  /cmd_vel \
  /cmd_vel_safe \
  /cmd_vel_nav \
  /map_live_ready \
  /explore/resume
```

这组 bag 的优点：

- 能看 frontier 候选与最终目标
- 能看 local/global costmap 当时长什么样
- 能看控制链路从 `cmd_vel` 到 `cmd_vel_safe` 是否被 collision monitor 限速或截停
- 能看 map warmup 是否暂停过探索

代价：

- 体积更大
- 回放时更容易与当前系统已有 publisher 冲突

## 回放策略

### A. 如果 bag 里录的是 `/scan`

这是最简单的回放方式。回放时不要再启动任何 scan bridge。

启动：

```bash
ros2 launch g1_nav g1_auto_explore.launch.py \
  use_sim_time:=true \
  enable_scan_bridge:=false \
  enable_realsense_scan_bridge:=false
```

播放：

```bash
ros2 bag play g1_auto_explore_minimal --clock
```

### B. 如果 bag 里录的是 Realsense 原始 depth

这时需要在回放时重新启动 `realsense_depth_to_scan.launch.py`，也就是让系统现场再生成 `/scan`。

启动：

```bash
ros2 launch g1_nav g1_auto_explore.launch.py \
  use_sim_time:=true \
  enable_scan_bridge:=false \
  enable_realsense_scan_bridge:=true
```

播放：

```bash
ros2 bag play g1_auto_explore_realsense_raw --clock
```

### C. 如果 bag 里录的是 Livox 原始点云

这时需要在回放时重新启动 `pointcloud_to_laserscan`。

启动：

```bash
ros2 launch g1_nav g1_auto_explore.launch.py \
  use_sim_time:=true \
  enable_scan_bridge:=true \
  enable_realsense_scan_bridge:=false
```

播放：

```bash
ros2 bag play g1_auto_explore_livox_raw --clock
```

## 录包与回放时的注意事项

### 1. 不要同时回放 `/scan` 又启动 scan bridge

否则会出现两个 publisher 同时向 `/scan` 发数据，导致：

- costmap 叠加异常
- 时间戳混乱
- 回放结果不稳定

规则很简单：

- bag 里已经有 `/scan`：两个 bridge 都关掉
- bag 里没有 `/scan`，只有 depth：开 Realsense bridge
- bag 里没有 `/scan`，只有点云：开 Livox bridge

### 2. `use_sim_time` 必须打开

回放 rosbag 时，`g1_auto_explore.launch.py` 必须带：

```bash
use_sim_time:=true
```

同时 `ros2 bag play` 要带：

```bash
--clock
```

否则 Nav2、TF、frontier planner 的时间基准会与 bag 不一致。

### 3. `tf_static` 一定要录

没有 `/tf_static`，常见结果包括：

- `base` 与 `base_link` 对不齐
- 相机 frame 链不完整
- `/scan` 或 depth 数据在 costmap 中投影错误

### 4. `global_costmap/costmap` 通常不是最小录包必需项

虽然 `frontier_explorer` 会订阅 `/global_costmap/costmap`，但这个 topic 由 Nav2 基于 `/lightning/grid_map` 与 `/scan` 动态生成。

所以：

- 想要“系统自己重算”：不必录
- 想做“事后诊断”：建议录

### 5. `explore/frontiers` 建议作为调试 topic，而不是回放依赖

`/explore/frontiers` 只是 frontier 可视化 marker，不是算法输入。

所以：

- 不录它，不影响回放
- 录它，有利于对照 bag 中当时的 frontier 选择结果

## 推荐实践

### 日常现场采集

优先使用最小可回放 bag：

```bash
ros2 bag record -o g1_auto_explore_minimal \
  /tf /tf_static /lightning/odometry /lightning/grid_map /scan
```

原因：

- 体积适中
- 最容易重放
- 足够复现大多数 frontier 与 Nav2 问题

### 需要分析局部避障输入质量

如果怀疑 `/scan` 本身有问题，不要只录 `/scan`，要录原始传感器输入：

- Realsense：录 depth image + camera info
- Livox：录原始点云

### 需要做深入调参与问题归因

再额外录全量调试 topic：

- costmap
- footprint
- frontier markers
- command chain
- map warmup control signals

## 最终建议

对于当前 `g1_nav` 自动探索项目，推荐分三档：

1. 默认现场录包：`/tf /tf_static /lightning/odometry /lightning/grid_map /scan`
2. 怀疑桥接链路时：改录原始 Realsense depth 或 Livox 点云，不直接录 `/scan`
3. 做系统级诊断时：在 1 或 2 的基础上，再补 costmap、frontier markers、cmd_vel 链路与 warmup 控制 topic

如果只允许保留一份标准录包方案，推荐优先选“最小可回放 bag”，因为它在体积、可移植性、复现能力之间最平衡。
