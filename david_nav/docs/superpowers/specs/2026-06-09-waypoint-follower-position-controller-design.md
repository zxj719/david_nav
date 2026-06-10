# Waypoint Follower — Nav2-planned, position-controller-driven

> 设计规格。Nav2 负责规划，把全局路径稀疏化成若干 waypoint，每段都是
> **校验过无障碍的直线且 ≤3m**，再由 `position_controller` 的 `MoveTo` action
> 逐段精确 turn-drive-turn 驱动。盲走直线之所以安全，正是因为每段都被验证过。

相关组件：[position_controller_README](../../position_controller_README.md)、
`navigation_goal_resolver.py`（复用其 `_GridMapView`）、`pose_provider.py`。
用户文档：[waypoint_follower_README](../../waypoint_follower_README.md)。

> **状态（2026-06-09，分支 `feature/disable-cmd-vel-separator`）：已实现并 headless 验证通过。**
> §1–§9 是实现前设计；§10 起为**实现记录**（as-built），记录实际交付、与设计的偏差、
> 本次对话新增的可视化 + 一键启动 + RViz 集成、构建/测试坑、以及实车前必查项。
> 实现要点回溯到 "Implement David position controller node" session（位置环 `MoveTo`
> 契约、caster-safe stop-and-turn、`/cmd_vel_position → mux` 链、不做 map→odom TF）。

---

## 1. 背景与动机

- `position_controller`（上位机位置环）做**车体位姿闭环**：精确直线 + 原地转 +
  turn-drive-turn 定点，**不做规划/避障**，盲走直线。接口为 `david_nav_msgs/action/MoveTo`。
- Nav2 做全局规划 + 局部避障，但在 David（70kg 重载差速 + 被动万向轮）上 DWB
  连续跟踪存在打滑/起步死锁/终点过冲等一系列适配难题（见 CLAUDE.md §5/§7）。
- 思路：**让 Nav2 只规划**，把路径切成若干「无障碍直线 ≤3m」的 waypoint，
  每段交给位置环精确驱动。每段都是验证过的无障碍直线 → 位置环盲走直线安全。
  这正是两个 waypoint 条件存在的意义。

## 2. 决策记录（brainstorming 已确认）

| 决策 | 选择 | 理由 |
|---|---|---|
| 谁驱动每段 | **位置环 `MoveTo` 逐段** | Nav2 只规划；位置环在 David 上点到点更可控 |
| 动态障碍 | **静态地图假设** | 规划时一次性校验全部段；运行中不监控/不重规划（符合「无避障」哲学） |
| 路径来源 | **`ComputePathToPose`**（planner-only action） | 只规划不执行，比抓 `/plan` topic 干净 |
| 触发接口 | 复用 **`nav2_msgs/NavigateToPose`** action 类型 | 不新增 msg 包；RViz "2D Nav Goal"/现有工具可直接指向 |
| 坐标系 | **逐段相对目标**（从实时 map 位姿算 relative） | 免疫 map↔odom 漂移；位置环本就不做 map→odom TF |
| 中间点朝向 | 默认 position-only（`yaw=NaN`） | 跳过末端原地转，少蹭脚轮；下段 ALIGN 自然纠向 |

## 3. 数据流

```
goal pose (map frame)
   │
   ▼
ComputePathToPose  ──►  dense nav_msgs/Path (map frame, planner 保证无障碍)
   │
   ▼
sparsify (greedy farthest-reachable, 纯逻辑)  ──►  sparse waypoints
   │   每相邻对: global costmap 上直线通视  AND  ≤ 3 m
   ▼
for each waypoint:
   读实时 map 位姿 → 算 robot-relative (dx,dy[,dyaw])
   send MoveTo POSE (relative=true) → /position_controller/move
   等到位 → 下一段
   │
   ▼
/cmd_vel_position → cmd_vel_mux → /cmd_vel   (位置环已拥有这条链)
```

## 4. 组件清单（全部落在 `david_nav`，纯加法，不改位置环/mux/separator）

| 文件 | 职责 | 可测 |
|---|---|---|
| `david_nav/waypoint_sparsifier.py` | **纯逻辑，无 ROS**。贪心 farthest-reachable 稀疏化 + Bresenham 直线通视。输入：dense polyline + grid view + `max_segment`(3.0) + `los_cost_threshold`；输出：满足两条件的 waypoint 列表。 | ✅ 单测 |
| `david_nav/waypoint_follower.py` | rclpy 节点：action server `~/navigate` (`NavigateToPose`) + `~/goal_pose` topic。调 `ComputePathToPose`，用最新 `/global_costmap/costmap` 跑稀疏化，逐段经 `MoveTo` action client 驱动，发 feedback，支持 cancel。 | 集成 |
| `david_nav/grid_map_view.py` | 把 `navigation_goal_resolver.py` 里的 `_GridMapView` 提取为共享模块（`world_to_cell`/`in_bounds`/`value`/`cell_center_world`/`resolution`），两处共用。 | ✅ 单测 |
| `config/waypoint_follower.yaml` | `max_segment: 3.0`、`los_cost_threshold`、planner action 名、costmap topic、`intermediate_heading: false`、各超时。 | — |
| `launch/waypoint_follower.launch.py` | 起 `waypoint_follower` + `position_controller` + `cmd_vel_mux`。 | — |
| `test/test_waypoint_sparsifier.py` | Bresenham 正确性、≤3m 强制、被障碍打断的分段、退化路径（0/1 点）、终点必含。 | ✅ |

> 复用：`grid_map_view` 由 `navigation_goal_resolver.py` 改为 import（行为不变，
> 仅搬家）。`pose_provider.current_map_pose()` 提供实时 map 位姿。

## 5. 稀疏化算法（两个条件）

dense 路径 `P[0..N]`（P[0]=机器人当前，P[N]=目标）上贪心选最远可达点：

```
anchor = P[0]; waypoints = []
i = 1
while i <= N:
    j 尽量前进, 同时满足:
        line_clear(anchor, P[j])  AND  dist(anchor, P[j]) <= max_segment
    取最后一个有效 P[j] 作为下一 waypoint; anchor = P[j]; i = j+1
保证把目标 P[N] 作为最后一个 waypoint(带目标朝向)
```

- **`line_clear`**：从 anchor cell 到 target cell 在**膨胀后的 global costmap** 上
  跑 Bresenham；任一 cell `value ≥ los_cost_threshold` 即判阻挡。用**膨胀**层做
  通视意味着清空的直线自动保留机器人半径间隙（`inflation_radius 0.40 ≥ inscribed 0.375`）。
  > 注意：nav2 把 global_costmap 以 `nav_msgs/OccupancyGrid` 发布，值域 **0–100**
  > （lethal=100，inscribed≈99，free=0），与原始 0–255 costmap cost **不同尺度**。
  > 阈值用 0–100 尺度。`value == -1`（未知）按可配 `block_unknown`（默认 true）当阻挡。
- **`max_segment = 3.0 m`**：硬上限（spec 要求）。
- **`los_cost_threshold`**：可配，**0–100 尺度**。默认 99（inscribed），直线可贴膨胀
  梯度但绝不进 inscribed/lethal 带；调高（→100）更宽松只避致命格、调低（如 1）
  更保守完全避开膨胀梯度。
- 保证：每段都是无障碍直线 AND ≤3m；段数最少（最少 turn-drive-turn → 少蹭脚轮）。
- 退化与兜底：N≤1（机器人已在目标附近）→ 直接单段到目标；若连 `P[i]`（相邻 dense
  点，planner 已保证无障碍且间距远小于 3m）都判不可达，取 `P[i]` 自身为 waypoint 前进，
  避免死循环（理论上不发生，作为防御）。

## 6. 坐标系与朝向

- **逐段相对目标**：驱动第 k 个 waypoint 前，读实时 map 位姿，算 map 系下 `(dx,dy)`，
  旋转进机器人系，发 `MoveTo{mode:POSE, target:(dx,dy), relative:true}`。逐段重算 →
  免疫 map↔odom 漂移，位置环无需 map→odom TF。
- **中间点朝向**：`yaw=NaN`（position-only），跳过末端原地转；末段套用目标朝向。
  `intermediate_heading:true` 时给中间点也定向（一般不需要）。

## 7. 错误处理 / 生命周期

- planner 不可用 / 空路径 → abort action，带原因消息。
- 尚无 costmap → abort（无法校验通视）。
- 某段 `MoveTo` abort（odom 丢失 / 超时 / 位置环 busy）→ 整个 follow abort，停车。
- **cancel**：传播到在飞的 `MoveTo`（位置环 ≤1 周期内零速），follow 结束。
- action feedback：当前 waypoint index / 总数、当前段剩余距离。

## 8. 明确不做（YAGNI / 边界）

- 不做运行中障碍监控或重规划（静态地图假设，已确认）。
- 不新增 message 包（复用 `NavigateToPose`）。
- 不改 `position_controller` / `cmd_vel_mux` / `cmd_vel_separator` 内部——纯加法。
- 不接管 RViz 默认 "2D Nav Goal"（其默认指向 nav2 `bt_navigator`）；本节点 action
  名独立（`~/navigate`），按需把工具指向它。
  > 更新（见 §13）：实现的 planner-only 整栈里**不起 bt_navigator**，并把节点的
  > `~/goal_pose` remap 成 `/goal_pose`，于是 RViz 默认 "2D Nav Goal" 直接驱动本节点。

## 9. 验收

- **单测**（纯逻辑，秒级）：`test_waypoint_sparsifier.py`——Bresenham、≤3m、
  障碍分段、退化、终点必含；`grid_map_view` 坐标换算。
- **仿真先行**：复用 `scripts/sim_fake_chassis.py`（积分 `/cmd_vel_position`→`/odom`）+
  发布一张合成 `/global_costmap/costmap` + 一条已知折线路径，验证：稀疏化段数正确、
  逐段 `MoveTo` 顺序驱动到目标、cancel 即停。
- **实车**：在已建图环境给一个目标，确认生成的 waypoint 每段直线穿过的都是空地、
  ≤3m，位置环逐段到位，总落点在目标容差内。

---

# 实现记录（as-built，2026-06-09）

## 10. 实际交付物

按 TDD 逐步实现，每步红/绿/commit。纯逻辑单测共 **17 passed**（`grid_map_view` 4 +
`waypoint_sparsifier` 13），现有套件回归 **无新增失败**（基线 9 failed / 49 passed →
9 failed / 62 passed，9 个失败是先前就有、与本特性无关）。

| 文件 | 状态 | 说明 |
|---|---|---|
| `david_nav/grid_map_view.py` | 新增 | `GridMapView`（含 `is_blocked(threshold, block_unknown)` + `from_occupancy_grid` 工厂） |
| `david_nav/navigation_goal_resolver.py` | 改 | 删除内嵌 `_GridMapView`，改 import 共享模块（行为不变，回归通过） |
| `david_nav/waypoint_sparsifier.py` | 新增 | `bresenham_cells` / `line_clear` / `sparsify` / `relative_target` / `relative_yaw`（纯逻辑） |
| `david_nav/waypoint_follower.py` | 新增 | 节点（见 §11–§13），含可视化发布 |
| `config/waypoint_follower.yaml` | 新增 | 参数默认值 |
| `launch/waypoint_follower.launch.py` | 新增 | 最小：follower + position_controller + cmd_vel_mux |
| `launch/waypoint_nav_bringup.launch.py` | 新增 | **一键整栈**（planner-only nav2 + 上述 + RViz），见 §13 |
| `rviz/waypoint_follower.rviz` | 新增 | Global Plan(dense) + Waypoints 显示，见 §13 |
| `scripts/launch_waypoint_nav.sh` | 新增 | 一键启动 + tee 日志 + `/tmp/waypoint_nav_latest.log`，见 §13 |
| `test/test_grid_map_view.py` / `test/test_waypoint_sparsifier.py` | 新增 | 单测 |
| `docs/waypoint_follower_README.md` | 新增 | 用户文档 |
| `CMakeLists.txt` | 改 | install(PROGRAMS) waypoint_follower |

**与设计的偏差/补充：**
- 触发接口实际用 **`MoveTo.Goal.MODE_POSE` 常量 = 2**；`ComputePathToPose.Goal.use_start=False`
  让 planner 用机器人当前位姿起规划。接口字段已在本环境核实存在。
- `_GridMapView` 提取为公共 `grid_map_view.GridMapView`（设计 §4 已规划，落地确认行为不变）。
- 新增 §12 可视化、§13 一键启动 + RViz 集成（原 spec 未覆盖，本次对话需求）。

## 11. Nav2 = 纯规划器，controller 被替换（关键认知）

本栈在**驱动链上等于把 Nav2 的 controller 拿掉**：

- **被旁路**：`controller_server`(DWB/FollowPath) 不参与；连带其下游 `velocity_smoother`
  → `david_move` → （本分支已禁用的 `cmd_vel_separator`）在位置环这条路上都**不经过**。
  位置环走 `/cmd_vel_position → cmd_vel_mux → /cmd_vel`，直接进 wheeltec（位置环不经过
  separator，否则其 binary_output/brake 会破坏 PID——见 position_controller_README §6）。
- **保留**：`planner_server`(全局规划 `ComputePathToPose`) + `global_costmap`（出折线 + 通视校验）。
  `bt_navigator`/`NavigateToPose` BT 那套不参与（只复用了 action **类型**，没用其行为树）。
- **代价**：丢掉 DWB 的连续局部避障 → 由「每段直线无障碍 + ≤3m」+ 静态地图假设兜底。
- **本分支 cmd_vel 链**（`feature/disable-cmd-vel-separator`，见 workspace CLAUDE.md §2）：
  `DWB→/cmd_vel_nav→velocity_smoother→david_move→/cmd_vel`。注意 nav2 自带的
  `nav2_waypoint_follower` 节点**也叫 `waypoint_follower`**，与本节点同名 → 整栈启动时
  **绝不能同时起 nav2 的那个**（§13 的 planner-only launch 不起它，规避冲突）。

## 12. 可视化（本次新增）

| 发布 | 消息 | QoS | 内容 |
|---|---|---|---|
| `~/plan` | `nav_msgs/Path` | latched（Transient Local, depth 1） | `ComputePathToPose` 的原始 dense 路径（map frame） |
| `~/waypoints` | `visualization_msgs/MarkerArray` | latched | LINE_STRIP(id0, start→各 wp) + SPHERE_LIST(id1, 各 wp)，map frame |

latched 让 RViz 即使在某次目标完成后才订阅也能看到上一次的 plan/waypoints。
marker frame = `map`（waypoint 是 map 系世界坐标）。

## 13. 一键启动 + RViz 集成（本次新增）

**`launch/waypoint_nav_bringup.launch.py`（planner-only 整栈）** 只起：
`planner_server`(+`global_costmap`，发 `/global_costmap/costmap`、提供 `ComputePathToPose`) +
`lifecycle_manager`（仅管 `planner_server`，autostart） + `position_controller` +
`cmd_vel_mux` + `waypoint_follower` + 可选 `robot_state_publisher` + 延迟 RViz。
**不起** DWB/smoother/david_move/bt_navigator → `cmd_vel_mux` 是**唯一** `/cmd_vel` 发布者，
不与底盘驱动抢 `/cmd_vel`（这是选 planner-only 而非全 bringup 的根本原因）。

- **RViz 手动 goal 驱动**：launch 把 `waypoint_follower` 的 `/waypoint_follower/goal_pose`
  remap 成 **`/goal_pose`**；RViz "2D Nav Goal"(SetGoal 工具) 默认就发 `/goal_pose` → 点目标即驱动。
  "2D Pose Estimate" 仍发 `/initialpose` 给 Lightning loc。
- **`rviz/waypoint_follower.rviz`**：基于 `nav2_go2_view.rviz`，把 Global Plan 改订
  `/waypoint_follower/plan`、Frontiers 改成 Waypoints 订 `/waypoint_follower/waypoints`，
  两者 Durability=**Transient Local**（匹配 latched 发布，否则 RViz 收不到）。
- **`scripts/launch_waypoint_nav.sh`**：一键 = pre-flight 杀残留 RSP（防 TF 树分裂）+ source +
  `ros2 launch … waypoint_nav_bringup` + tee 到 `/tmp/david_nav_logs/waypoint_nav_<stamp>.log`，
  并维护 symlink `/tmp/waypoint_nav_latest.log`。支持 `BG=1`。**不强制 `ROS_DOMAIN_ID`**
  （继承环境、默认 0；强制 domain 是经典「RViz 啥都没有」坑）。

**运行顺序**：① IMU/雷达/锁频/建图（手动）② 底盘驱动 ③ `launch_waypoint_nav.sh`
④ RViz 点 2D Pose Estimate → 等 costmap → 2D Nav Goal ⑤ 看 Global Plan(dense)+Waypoints。

## 14. 构建 / 测试 / 验证状态

- **单测运行**：`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_grid_map_view.py
  test/test_waypoint_sparsifier.py -q`。环境里有损坏的 `anyio` pytest 插件（`_pytest.scope`
  缺失），不禁用自动加载则**所有** pytest 收集即崩——这是环境问题，非本特性。
- **编译**：`PATH=/usr/bin:$PATH colcon build --packages-select david_nav_msgs david_nav`。
  必须把**系统 cmake (`/usr/bin/cmake` 3.22)** 放 PATH 前——`~/.local/bin/cmake` 4.x(pip)
  是已知 build 坑（见 workspace memory）。build/install 现属主 agx。
- **已验证（headless，本机无 SLAM/底盘）**：全栈节点起来（`planner_server`、
  `global_costmap/global_costmap`、`position_controller`、`cmd_vel_mux`、`waypoint_follower`、
  lifecycle）、goal remap 生效（`waypoint_follower` 订 `/goal_pose`）、发布
  `/waypoint_follower/plan`+`/waypoint_follower/waypoints`、launch `--show-args` 正常、
  rviz yaml 合法、节点导入实例化 + marker builder 不崩。
- **未验证（需实车/SLAM）**：`/global_costmap/costmap` 真正**填充**（取决于
  `/lightning/grid_map` 进来）；端到端 plan→sparsify→逐段 MoveTo 实际驱动；总落点精度。

## 15. 实车前必查（诚实边界）

1. **costmap 填充**：首启在日志确认 `/global_costmap/costmap` 有数据，否则 follower
   打 `no global costmap` 并 abort。无 SLAM 环境只能验证到 costmap 节点存在，填充需实车确认。
2. **位置环死区/方向**：70kg 重载底盘上 `min_cmd_v/min_cmd_w/invert_linear` 大概率要在
   `config/position_controller.yaml` 按实测微调（决策：先按现有位置环跑，实车再调）。
3. **静态地图假设**：盲走直线，运行中不重规划；有动态障碍进入已规划段则不处理。
   需要时另上 collision_monitor 或改造为「监控 + 重规划」。
