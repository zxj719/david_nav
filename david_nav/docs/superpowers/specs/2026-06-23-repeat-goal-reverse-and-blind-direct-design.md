# 设计：第二个 goal 起「先后退 1m + 盲走直线到终点」

日期：2026-06-23
范围：`david_nav/david_nav/waypoint_follower.py`（外加 launch 透传新参数）

## 1. 背景与需求

抓取任务用 `start_waypoint_nav_headless.sh` 一键起导航：前台
`navigation_executor.py` 把 server 下发的 `navigate_to {x,y,theta}` 转成
`/waypoint_follower/navigate`（`NavigateToPose`）action goal，由
`waypoint_follower` 节点执行。当前每个 goal 都走同一套流程：

```
全局规划(ComputePathToPose) → 稀疏成 waypoints → 终点前插入场点 A
  → position_controller 分段直行 → 末端闭环复查
```

需求：**从第 2 个 goal 起**（第 1 个之后的每一个：第 2、3、4…），导航行为改成：

1. **先后退 1m**——沿当前朝向直线倒车 1m，不做 180° 掉头。
2. **再盲走直线到终点**——不调全局规划、不稀疏 waypoints、不插入场点，直接一段
   从当前位姿到（base_link 补偿后的）终点，带终点朝向。

第 1 个 goal 保持原流程不变。

## 2. 关键事实（来自代码勘查）

- `waypoint_follower` 是常驻节点，每个 goal 走 `_follow()`；天然适合存「已完成
  goal 计数」，**计数随栈重启自然归零**。
- `MoveTo.action` 支持三种 mode：`MODE_DISTANCE`（沿当前朝向的有符号距离）、
  `MODE_ROTATE`、`MODE_POSE`（目标点 + 终点朝向，可相对）。
- `position_control_core` 的 `MODE_DISTANCE`：`project_travelled` 沿起始朝向投影，
  `distance=-1.0` ⇒ **直线倒车 1m 且保持朝向**（不掉头）——正好满足「后退 1m」。
- `MODE_POSE` 内部是 ALIGN(转向目标点)→DRIVE(纯直行)→FINAL_TURN(转到终点朝向)。
  一段相对 POSE MoveTo 到终点 ⇒ 天然完成「转向终点 + 直行 + 终转」——正好满足
  「盲走直线」。
- 现有 `_follow()` 里两段逻辑可复用：终点 base_link 补偿（约 383–396 行）、末端
  闭环复查（约 514–558 行）。

## 3. 设计决策（已与用户确认）

| 项 | 决定 |
|---|---|
| 触发范围 | 第 2 个 goal 起全部（计数 ≥ `repeat_goal_after_count`，默认 1） |
| 「直接到终点」 | **完全盲走直线**：不调 planner，一段 POSE MoveTo 到终点 |
| 后退方式 | 先 `MODE_DISTANCE` 直线倒车 1m，再盲走直线（其 ALIGN 阶段完成「转向终点」） |
| 计数粒度 | **只计成功到点**：`goal_handle.succeed()` 时才 +1；首次失败/abort 重试仍算第 1 个 |
| 默认开关 | **默认开启**（符合原始抓取任务诉求） |

## 4. 新增参数（默认值即需求）

在 `waypoint_follower` 节点 `declare_parameter`：

| 参数 | 类型 | 默认 | 含义 |
|---|---|---|---|
| `repeat_goal_after_count` | int | `1` | 进入特殊模式前的「正常 goal」数；1 ⇒ 第 2 个起 |
| `repeat_reverse_dist` | double(m) | `1.0` | 特殊模式先倒车的距离；`0` 关闭倒车 |
| `repeat_blind_direct` | bool | `True` | 特殊模式是否盲走直线（跳过 planner/waypoints/入场点）；`False` 则特殊模式只在原规划流程前加倒车 |

launch（`scripts/launch_waypoint_nav.sh` / `waypoint_nav_bringup.launch.py`）按现
有参数透传风格把这三个新参数透出去，保持可 `arg:=val` 覆盖。

## 5. 实现：`_follow()` 重构 + 分支

把 `_follow()` 拆成共享小函数，让「规划路径」和「盲走直线」两条路径复用：

- `_compensate_goal_to_ref_pose(goal_pose, goal_yaw)` —— 抽出现有终点→base_link
  补偿块（就地改 `goal_pose`，返回补偿后的 `goal_yaw` 不变）。
- `_final_check_loop(goal_pose, goal_yaw, goal_handle) -> bool` —— 抽出现有末端
  闭环复查（停稳→等定位收敛→按残差补走→通过才成功）。
- `_drive_distance(signed_dist) -> bool` —— 新增：发 `MoveTo` `MODE_DISTANCE`
  `distance=signed_dist`，复用 `_drive_segment` 同样的 send/await 模式。倒车段
  容差用中间段松容差（`wp_dist_tol`）即可。

`_follow()` 新骨架：

```
acquire _drive_lock（失败则 reject，不变）
self._cancel_evt.clear()
is_repeat = self._completed_goals >= repeat_goal_after_count
goal_yaw = 从 goal_pose 四元数取
_compensate_goal_to_ref_pose(goal_pose, goal_yaw)   # 两条路径都先补偿，几何一致

if is_repeat and repeat_blind_direct:
    ok = _follow_direct(goal_pose, goal_yaw, goal_handle)
else:
    # 注：repeat 但 repeat_blind_direct=False 时，可选择仍先倒车再走原规划；
    #     MVP 下 is_repeat 只在 blind_direct=True 生效，规划路径保持完全不变。
    ok = _follow_planned(goal_pose, goal_yaw, goal_handle)

if ok:
    self._completed_goals += 1
```

`_follow_direct(goal_pose, goal_yaw, goal_handle)`：

```
1. 取消检查（同现有 cancel 模式）
2. if repeat_reverse_dist > 0:
       _set_move_tols(wp_dist_tol, pi)
       if not _drive_distance(-repeat_reverse_dist): abort; return False
3. 盲走直线一段：
       cx,cy,cyaw = _stable_map_pose()           # 等定位收敛
       rx,ry = relative_target(cx,cy,cyaw, gx,gy) # gx,gy = 补偿后终点
       ryaw  = relative_yaw(cyaw, goal_yaw) if not nan else nan
       _set_move_tols(goal_dist_tol, goal_yaw_tol)
       if not _drive_segment(rx,ry,ryaw): abort; return False
4. _final_check_loop(...)   # 复用末端闭环复查，终点精度锚在静止定位精度上
5. succeed; return True
```

`_follow_planned(...)` = 现有 `_follow` 主体（规划+稀疏+入场点+分段+末端复查），
原样保留，仅把已抽出的补偿/复查改成调用共享函数。

计数 `self._completed_goals` 在 `__init__` 初始化为 0。

## 6. 影响范围 / 不变量

- 第 1 个 goal、以及 `repeat_blind_direct=False` 时的所有 goal：行为与现状完全一致。
- 计数随节点（栈）重启归零——每次跑 `start_waypoint_nav_headless.sh` 都从第 1 个
  goal 重新开始。符合「一次抓取任务 = 一次启动」的用法。
- 末端闭环复查在两条路径都保留 ⇒ 终点精度不退化。
- 终点 base_link 补偿在两条路径都先做 ⇒ 抓取基准一致。

## 7. 已知取舍（按设计接受）

- **倒车段与盲走直线段都不做避障**——与现有「分段直行假设静态地图、规划时一次
  性验证」是同一假设的延伸。要求：机器人正后方 1m 空旷、且倒车后到终点之间是
  空旷直线。抓取场景（先离开操作台再去放置点）通常成立；若不成立需人工保证或
  后续加一个轻量 costmap 直线检查（本期 YAGNI，不做）。
- 盲走直线若中途被挡，`position_controller` 的 miss/re-approach 只在小范围兜底，
  不会重新规划绕行——这是「盲走」的固有限制，已在参数 `repeat_blind_direct` 上
  保留可关闭的逃生口。

## 8. 验证

1. `colcon build --packages-select david_nav` 后 `python -m py_compile` /
   `ros2 param` 干跑确认参数声明。
2. 跑 `start_waypoint_nav_headless.sh`，连发两个 goal，读
   `/tmp/waypoint_nav_latest.log`：
   - 第 1 个 goal：有 `[TRACE] plan ... wps=N`、入场点 A 日志（原流程）。
   - 第 2 个 goal：先 `MODE_DISTANCE` 倒车日志，无 planner/waypoints 日志，一段
     POSE 盲走 + `final check` 日志。
3. 确认第 2 个 goal 终点精度（`final check ... OK`）与第 1 个相当。

## 9. 不做（YAGNI）

- 倒车/盲走的 costmap 直线避障检查。
- 把计数持久化到磁盘（跨重启记忆）——需求就是每次启动从头算。
- 对 `_on_goal_pose` 话题路径做特殊处理：它与 action 共用 `_follow`，自动一致。
