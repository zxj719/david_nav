# 设计：第二个 goal 起「先后退 1m + 把终点当 waypoint 分段去」

日期：2026-06-23
范围：`david_nav/david_nav/waypoint_follower.py`（外加 launch 透传新参数）

## 1. 背景与需求

抓取任务用 `start_waypoint_nav_headless.sh` 一键起导航：前台
`navigation_executor.py` 把 server 下发的 `navigate_to {x,y,theta}` 转成
`/waypoint_follower/navigate`（`NavigateToPose`）action goal，由
`waypoint_follower` 节点执行。当前每个 goal 都走同一套流程：

```
全局规划(ComputePathToPose) → 稀疏成 waypoints → 终点前插入场点 A
  → position_controller 分段直行 → 末端闭环复查(紧容差 + 终转)
```

需求（经多轮确认收敛）：**从第 2 个 goal 起**（第 1 个之后的每一个：第 2、3、4…），
导航行为改成：

1. **先后退 1m**——沿当前朝向直线倒车 1m，不做 180° 掉头。
2. **再把终点当一个普通 waypoint 去**——不调全局规划、不插入场点；从当前位姿到
   终点连一条几何直线，按 `max_segment` 分段，每段 POSE「停-对齐-直行」。**关键：
   不是一条直线闷头开到底**——分段让长距离能周期性重新对齐；单段内若偏了/距离
   不降反增，`position_controller` 的 POSE 模式会刹停、回 ALIGN 重新对齐再前进。
3. **位置容差放宽到 1.5m、不管终点朝向**——终点和中间 waypoint 一样按 1.5m 位置
   容差到点，不做终点原地转向（position-only），也不做紧致末端复查。

第 1 个 goal 保持原流程不变（仍走规划 + 入场点 + 末端复查）。

## 2. 关键事实（来自代码勘查）

- `waypoint_follower` 是常驻节点，每个 goal 走 `_follow()`；天然适合存「已完成
  goal 计数」，**计数随栈重启自然归零**。
- `MoveTo.action` 支持 `MODE_DISTANCE`（沿当前朝向有符号距离）/ `MODE_ROTATE` /
  `MODE_POSE`（目标点 + 终点朝向，可相对，`yaw=NaN` ⇒ position-only 跳过终转）。
- `position_control_core` 的 `MODE_DISTANCE`：沿起始朝向投影，`distance=-1.0`
  ⇒ **直线倒车 1m 且保持朝向（不掉头）**——满足「后退 1m」。
- `MODE_POSE` 内部 ALIGN(转向目标点)→DRIVE(纯直行)→FINAL_TURN(转终点朝向)；
  DRIVE 阶段 `rho` 反弹超过 `drive_overshoot_margin` ⇒ 刹停回 ALIGN 重新对齐
  （`pose_max_cycles` 兜底）——即「距离增大了就停下来对齐再前进」。`yaw=NaN`
  时跳过 FINAL_TURN。
- 现有 `_follow()` 的**逐段驱动循环**（每段：`_stable_map_pose` 等定位收敛 →
  `relative_target` → `_set_move_tols` → `_drive_segment`）可直接复用，唯一差别是
  waypoints 的来源和容差/终转策略。

## 3. 设计决策（已与用户确认）

| 项 | 决定 |
|---|---|
| 触发范围 | 第 2 个 goal 起全部（计数 ≥ `repeat_goal_after_count`，默认 1） |
| 路径生成 | **几何直线分段**：当前位姿→终点连直线，按 `max_segment`(默认3m)均匀切；不调 planner、不避障、不插入场点 A |
| 单段驱动 | POSE 停-对齐-直行；偏了/距离反增由 POSE 模式自身刹停重对齐（无需额外逻辑） |
| 终点处理 | **当作 waypoint**：position-only（`yaw=NaN`，不终转）、位置容差 1.5m、**不做紧致末端复查** |
| xy 容差 | **全局**把 `wp_dist_tol` 默认 0.8 → **1.5m**（第 1 个 goal 的中间 waypoint 同样受益） |
| 计数粒度 | **只计成功到点**：最后一段成功后才 +1；首次失败/abort 重试仍算第 1 个 |
| 默认开关 | **默认开启** |

## 4. 参数变更

`waypoint_follower` 节点：

| 参数 | 类型 | 旧默认 | 新默认 | 含义 |
|---|---|---|---|---|
| `wp_dist_tol` | double(m) | 0.8 | **1.5** | 中间 waypoint 位置容差（全局，两条路径共用） |
| `repeat_goal_after_count` | int | — | `1` | 进入特殊模式前的「正常 goal」数；1 ⇒ 第 2 个起 |
| `repeat_reverse_dist` | double(m) | — | `1.0` | 特殊模式先倒车的距离；`0` 关闭倒车 |
| `repeat_goal_as_waypoint` | bool | — | `True` | 第 2 个起：把终点当 waypoint 分段去（不调 planner/不插入场点/不终转/不紧复查）；`False` ⇒ 该 goal 仍走原规划流程（仅前面加倒车，可选） |

launch（`scripts/launch_waypoint_nav.sh` / `waypoint_nav_bringup.launch.py`）按现有
透传风格把新参数透出去，保持可 `arg:=val` 覆盖。

## 5. 实现：`_follow()` 重构 + 分支

抽出共享小函数，让两条路径复用：

- `_drive_distance(signed_dist) -> bool` —— 新增：发 `MoveTo` `MODE_DISTANCE`
  `distance=signed_dist`，复用 `_drive_segment` 同样的 send/await 模式。倒车段容差
  用 `wp_dist_tol`。
- `_drive_waypoint_chain(waypoints, goal_yaw, goal_handle, last_tol, last_yaw_tol,
  do_final_check) -> bool` —— 把现有逐段循环（含取消检查、`[TRACE]` 日志、feedback、
  可选末端复查）抽成一个函数，由两条路径以不同参数调用。

`_follow()` 新骨架：

```
acquire _drive_lock（失败 reject，不变）
self._cancel_evt.clear()
is_repeat = self._completed_goals >= repeat_goal_after_count
goal_yaw = 从 goal_pose 四元数取

if is_repeat and repeat_goal_as_waypoint:
    ok = _follow_as_waypoint(goal_pose, goal_handle)
else:
    ok = _follow_planned(goal_pose, goal_yaw, goal_handle)   # 现有主体，含终点补偿/末端复查

if ok:
    self._completed_goals += 1
```

`_follow_as_waypoint(goal_pose, goal_handle)`：

```
1. 取消检查（同现有模式）
2. if repeat_reverse_dist > 0:
       _set_move_tols(wp_dist_tol, pi)
       if not _drive_distance(-repeat_reverse_dist): abort; return False
3. 几何分段（map 系，不调 planner、不避障、不补偿、不插入场点 A）：
       cx,cy,_ = _stable_map_pose()
       gx,gy   = goal_pose.position
       dist    = hypot(gx-cx, gy-cy)
       n_seg   = max(1, ceil(dist / max_segment))
       waypoints = [ (cx+frac*(gx-cx), cy+frac*(gy-cy))
                     for k in 1..n_seg, frac=k/n_seg ]   # 末点 == 终点
       _publish_waypoints((cx,cy), waypoints)            # 复用 viz
4. _drive_waypoint_chain(waypoints, goal_yaw=NaN, goal_handle,
                         last_tol=wp_dist_tol, last_yaw_tol=pi,
                         do_final_check=False)
       # 每段 ryaw=NaN（position-only，不终转）、tol=wp_dist_tol(1.5m)
       # 偏了/距离反增由 POSE 模式自身刹停重对齐
5. succeed; return True
```

`_follow_planned(...)` = 现有 `_follow` 主体，逐段循环改成调用
`_drive_waypoint_chain(...)`：`goal_yaw=真实终点朝向`、末段 `last_tol=goal_dist_tol`、
`last_yaw_tol=goal_yaw_tol`、`do_final_check=True`、并保留入场点 A 与终点 base_link
补偿。行为与现状一致。

计数 `self._completed_goals` 在 `__init__` 初始化为 0。

## 6. 影响范围 / 不变量

- 第 1 个 goal、以及 `repeat_goal_as_waypoint=False` 时的所有 goal：流程与现状一致，
  **唯一全局差异**是 `wp_dist_tol` 默认从 0.8 放宽到 1.5m（中间 waypoint 更松）。
- 第 2 个起：无 planner/入场点/终转/紧复查日志；有 `MODE_DISTANCE` 倒车日志 +
  几何分段 POSE 日志。
- 计数随节点（栈）重启归零——每次跑 `start_waypoint_nav_headless.sh` 从第 1 个
  goal 重新开始。符合「一次抓取任务 = 一次启动」。

## 7. 已知取舍（按设计接受）

- **倒车段与第 2 个起的分段都不做 costmap 避障**——延续现有「分段直行假设静态
  地图」的同类假设，但更进一步（连规划期的 LOS 校验也没有）。要求：机器人正后方
  1m 空旷、且当前位姿到终点的直线走廊空旷。抓取场景（先离台再去放置点）通常成立；
  不成立需人工保证，或后续加一个轻量直线 costmap 检查（本期 YAGNI）。
- 第 2 个起终点不做朝向控制、容差 1.5m ⇒ 终点位姿精度明显低于第 1 个 goal——
  这是「当作 waypoint」的预期结果，符合需求（放置点对位姿不敏感 / 由后续抓取流程
  自行精对）。

## 8. 验证

1. `colcon build --packages-select david_nav` 后 `python -m py_compile` /
   `ros2 param get` 干跑确认参数声明（`wp_dist_tol`=1.5 等）。
2. 跑 `start_waypoint_nav_headless.sh`，连发两个 goal，读
   `/tmp/waypoint_nav_latest.log`：
   - 第 1 个 goal：有 `[TRACE] plan ... wps=N`、入场点 A、末端复查日志（原流程）。
   - 第 2 个 goal：先 `MODE_DISTANCE` 倒车日志 → 几何分段（`n_seg` 段）POSE 日志
     （每段 `ryaw=nan`、tol≈1.5m）→ 无入场点 / 无紧致末端复查。
3. 确认第 2 个 goal 到点判定为「最后一段在 1.5m 内」，且长距离时中途有多段
   ALIGN（停下来对齐）日志。

## 9. 不做（YAGNI）

- 倒车 / 分段的 costmap 直线避障检查。
- 把计数持久化到磁盘（跨重启记忆）——需求就是每次启动从头算。
- 对 `_on_goal_pose` 话题路径做特殊处理：它与 action 共用 `_follow`，自动一致。
