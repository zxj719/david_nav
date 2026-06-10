# waypoint_follower — Nav2 规划 + 位置环驱动的航点跟随

> Nav2 **只负责规划**，把全局路径稀疏化成若干「无障碍直线 ≤3m」的 waypoint，
> 由 `position_controller` 的 `MoveTo` action 逐段精确 turn-drive-turn 驱动。
> 盲走直线之所以安全，正是因为每段都被在全局 costmap 上验证过无障碍。
>
> 设计规格：[docs/superpowers/specs/2026-06-09-waypoint-follower-position-controller-design.md](superpowers/specs/2026-06-09-waypoint-follower-position-controller-design.md)
> 实现计划：[docs/superpowers/plans/2026-06-09-waypoint-follower-position-controller.md](superpowers/plans/2026-06-09-waypoint-follower-position-controller.md)

---

## 1. 这是什么 / 为什么

`position_controller`（上位机位置环）做精确点到点（直线 + 原地转 + turn-drive-turn），
**不做规划/避障**，盲走直线。Nav2 做全局规划 + 局部避障，但 DWB 在 David（70kg 重载
差速 + 被动万向轮）上连续跟踪问题多（打滑/起步死锁/终点过冲，见 CLAUDE.md §5/§7）。

本节点把两者拼起来：
- **Nav2 当纯规划器**（`ComputePathToPose`，只规划不执行）出一条绕开障碍的折线。
- 把折线**稀疏化**成 waypoint，每相邻两点满足两个条件 → 见 §2。
- 每段用位置环 `MoveTo` 精确驱动。**DWB 这条 controller 链（含 `cmd_vel_separator`）
  被旁路**；位置环走 `/cmd_vel_position → cmd_vel_mux → /cmd_vel`。

## 2. 两个 waypoint 条件（关键）

生成路径后，相邻两点之间：
1. **无障碍**：在膨胀后的 global costmap 上做 Bresenham 直线通视，任一 cell
   `value ≥ los_cost_threshold`（或未知 cell，当 `block_unknown`）即判阻挡。用膨胀层
   做通视 → 清空的直线自动保留机器人半径间隙。
2. **距离 ≤ `max_segment`**（默认 3.0m）。

稀疏化用**贪心 farthest-reachable**：从锚点尽量取最远的、仍满足两条件的路径点作为下一
waypoint，段数最少（少 turn-drive-turn → 少蹭脚轮）。目标点恒为最后一个 waypoint。
> 注意：global_costmap 以 `nav_msgs/OccupancyGrid` 发布，值域 **0–100**（lethal=100，
> inscribed≈99，free=0），与原始 0–255 costmap cost 不同尺度；阈值用 0–100 尺度。

## 3. 坐标系与朝向

- **逐段相对目标**：驱动每个 waypoint 前读实时 `map` 位姿，算 map 系 `(dx,dy)` 旋进机器人系，
  发 `MoveTo{mode:POSE, target:(dx,dy), relative:true}`。逐段重算 → 免疫 map↔odom 漂移，
  位置环无需 map→odom TF。
- **中间点 position-only**（`yaw=NaN`，跳过末端原地转）；**末段**套用目标朝向。
  `intermediate_heading:true` 时给中间点也按段方向定向。

## 4. 接口

| 类型 | 名称 | 消息 | 说明 |
|---|---|---|---|
| Action server | `~/navigate` | `nav2_msgs/action/NavigateToPose` | 主入口（复用类型，**非** nav2 bt_navigator） |
| Topic sub | `~/goal_pose` | `geometry_msgs/PoseStamped` | 便捷入口（map frame） |
| Action client | `compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | 向 planner_server 要路径 |
| Action client | `/position_controller/move` | `david_nav_msgs/action/MoveTo` | 逐段驱动 |
| Topic sub | `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | 通视校验 |

支持 **cancel**：传播到在飞的 `MoveTo`（位置环 ≤1 周期零速），follow 结束。

参数见 [config/waypoint_follower.yaml](../config/waypoint_follower.yaml)。

## 5. 静态地图假设（边界）

段在**规划时一次性校验**。运行中**不监控、不重规划**（位置环盲走直线）。若环境可能有
动态障碍进入已规划的某段，本节点不处理——需要时另上 collision_monitor 或改造为监控+重规划。

## 6. 运行

```bash
# 起 follower + position_controller + cmd_vel_mux
ros2 launch david_nav waypoint_follower.launch.py

# 需要在标准 nav bringup 里跑（要 planner_server + /global_costmap/costmap + map->base TF）：
#   见 CLAUDE.md §4：起 Nav2、在 RViz 点 2D Pose Estimate 给初始位姿
# 发目标（map frame 绝对位姿）：
ros2 action send_goal /waypoint_follower/navigate nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"
```
看 follower 日志的 `path N pts -> n waypoints` 和逐段 `segment k/n -> body(...)`，确认每段
≤3m；位置环驱动 `/odom` 逐段收敛到目标。

## 7. 测试

```bash
cd ~/ros2_ws/src/david_nav
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  test/test_grid_map_view.py test/test_waypoint_sparsifier.py -q
```
（纯逻辑单测，无需 colcon build。`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 绕开环境里损坏的
anyio pytest 插件。）覆盖：GridMapView 坐标换算/`is_blocked`、Bresenham 通视、≤3m 强制、
障碍分段、退化路径、终点必含、相对目标几何。
