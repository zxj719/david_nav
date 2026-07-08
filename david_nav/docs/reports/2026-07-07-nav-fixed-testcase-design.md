# 导航固定测试用例设计 —— 评审与补充

- 日期：2026-07-07
- 目的：为"当前导航"建立可复现的固定测试用例。用户提出四个维度：稳定性（10 次终点分布）、
  精确性（终点误差）、轨迹合理性（规划路径平滑 + waypoint 结构）、跟踪性能（controller 对
  planner 的跟踪，实际 vs 规划）。本文对这四点做评审，指出**不成立/会误导的地方**，并补充
  必要维度。
- 结论先行：**四个维度方向对，但少了地基——"真值从哪来"。在这台车上 odom 绝对漂移 >2m、
  lightning 也漂，直接拿 nav 自报位姿算"终点误差"测的是它的信念、不是物理真相。**
  不先定义真值源，稳定性和精确性两项数字全是自欺。

---

## 最终定稿(2026-07-07,用户拍板 + 已落地 harness)

评审后用户确定的范围,**以此为准**(下方 §0-§9 是当初评审的推导过程,留作依据):

- **真值:统一用 `david_slam_gt`**(已有的包,见
  [../../../david_slam_gt/](../../../david_slam_gt/) 及其
  [管路报告](../../../david_slam_gt/docs/reports/2026-06-17-slam-toolbox-groundtruth-pipeline.md))。
  它把 slam_toolbox localization 锁图后的 `map->base_footprint` 发成 `/groundtruth/pose`
  (20Hz)。bench 直接消费这个话题。**忽略定位漂移**,当绝对真值。**位置含 yaw。**
  (诚实边界:slam_toolbox 同时也是 nav 的定位源,抓不了"定位本身导致的误差"——要那种得上 apriltag 外部真值。)
- **只测 controller,不测 planner** → **砍掉"轨迹合理性"维度**(那是 planner 质量)。
  planner 对所有 controller 都绕过,大家吃**同一条固定 map 折线**。
- **一条固定折线覆盖 S1-S4**(起点/终点不同朝向且有距离):直行 / 原地转 / L 形
  turn-drive-turn / 入位,一次跑完。不再拆 4 次。
- **保留维度**:可靠性(成功率+失败分类)、稳定性(终点重复性)、精确性(终点 vs goal 含 yaw)、
  跟踪(实际轨迹 vs 固定折线的 cross-track)。
- **统一框架**:现在测 HEAD 的 `position_controller`(waypoint 栈),以后 DWB 等只加一个
  `ControllerAdapter`,参考路径/记录器/指标脚本都不动。

**已落地代码**:[`scripts/controller_bench/`](../../scripts/controller_bench/)
(`reference_path.yaml` 固定折线占位坐标 · `run_bench.sh`/`run_bench.py` 驱动 ·
`record_track.py` 记 `map->base` 真值轨迹 · `bench_metrics.py` 汇总三张表 · `README.md`)。
用户建完图后把 `reference_path.yaml` 的占位坐标换成本地图实测 start/waypoints 即可跑。

---

## 0. 一句话给每个维度打分

| 用户维度 | 判断 | 关键问题 |
|---|---|---|
| 稳定性（10 次终点分布） | 方向对，欠 3 处 | ① 真值源未定 ② 缺失败率（只统计成功=幸存者偏差）③ 起点复位漂移会污染下一次 |
| 精确性（终点误差） | **有陷阱** | 混淆了"控制误差"和"定位漂移"；且缺**朝向误差**（这车是操作机器人，yaw 要紧） |
| 轨迹合理性 | 用错基准 | 当前 HEAD 是 waypoint 盲走直线，执行轨迹**设计上**就不跟全局规划曲线；该比的是 vs 稀疏 waypoint 折线，不是 vs 全局 plan |
| 跟踪性能 | 依赖前提 | "actual vs planned"取决于跑哪套栈（DWB 才有 local 跟踪；waypoint 模式没有 DWB） |

---

## 1. 最关键的缺口：真值源（决定前两个维度成不成立）

`brake_trace` CSV 记的 `odom_x/odom_y` 在 **odom 系**、`plan_goal_x/y` 在 **map 系**——
直接相减本身就是错的坐标混用。更根本的问题：

- **odom 绝对漂移巨大（实测 >2m）** → 不能当终点真值。
- **lightning `map→odom` 也漂**（本来就是要测的东西）→ 用它算误差 = 用尺子量尺子自己。
- nav 报"到达"（`d2g < tol`）只说明**它以为**到了。机身可能物理上偏出去几米而 action 照样
  SUCCEEDED。

所以"终点误差""终点分布"**必须绑定一个独立真值**。可选（按精度/成本）：

1. **AprilTag 车载相机真值**（见 [2026-06-17 报告](2026-06-17-apriltag-truth-oracle-feasibility-and-bugfix.md)）：
   补齐外参+场地图后 **厘米级位置 / 1~2° yaw**。**推荐作为固定用例的真值源**——可自动读数、可脚本化。
2. **地面贴胶带 + 卷尺人工量**：零成本、最可信、但慢、只测终点不测轨迹。做基线校核用。
3. **slam_gt 真值**（见 [[project_nav2_on_slam_gt_recipe]]）：适合离线复放整条轨迹。

> 没有 1/2/3 任一，稳定性和精确性两个维度不要做——数字会骗人。

---

## 2. 精确性维度的陷阱：拆成两个正交误差

"终点误差"其实是两件事叠加，必须分开测，否则调参时不知道该改哪层：

| 误差 | 定义 | 真值 | 反映 |
|---|---|---|---|
| **控制误差** | nav 自报到达位姿 − 指令 goal | 无需外部真值（都在 nav 信念系内） | 位置环/DWB 收敛质量、容差设置 |
| **定位漂移** | 物理真值位姿 − nav 自报位姿 | AprilTag / 卷尺 | lightning/odom 漂移 |
| **总精度** | 物理真值位姿 − 指令 goal | AprilTag / 卷尺 | 用户真正关心的"车停哪了" |

补充：**朝向误差**用户漏了。这台是操作机器人，终点 yaw 决定手臂能不能够到；
`waypoint_follower` 有 `goal_yaw_tol=0.4rad`，caster 拖尾还有残余 ~2.5° 过冲
（见 [2026-07-01 复盘](2026-07-01-rotation-overdamping-retrospective.md)）。**终点 yaw 误差必须单列**。

---

## 3. 轨迹/跟踪维度：先说清跑的是哪套栈

workspace 现在有两条并列执行路径，"轨迹合理性"和"跟踪性能"的含义完全不同：

- **Nav2 全栈**（DWB → separator）：有 local controller，"actual vs planned"成立，
  跟踪 = DWB 对全局 plan 的横向跟踪。
- **`waypoint_follower`（当前 HEAD）**：Nav2 **只 PLAN**（ComputePathToPose）→ 稀疏成
  waypoint 折线 → `position_controller` 逐段**盲走直线**，DWB 被绕过。此模式下：
  - 执行轨迹是 turn-drive-turn 折线，**设计上就不跟全局 plan 的曲线**。拿它 vs 全局 plan 算
    偏差，测出来的"跟踪差"是伪命题。
  - 正确基准 = **稀疏后的 waypoint 折线**。横向误差应对折线算，不对原始 plan 算。

**必须同时录三条线**才能把这个维度说清：① 原始全局 plan ② 稀疏 waypoint（`waypoint_follower`
发的 Marker/`MarkerArray`）③ 实际轨迹（真值系，非 odom）。

轨迹合理性的可量化指标（别用"平滑"这种词）：
- 路径长度比 = 实际 / 直线距离（越接近 1 越好）
- 最大横向误差（cross-track）+ RMS，对 waypoint 折线算
- 每段的停-走次数 / 抽搐次数（caster 死锁、sign-latch 翻向的残留）
- waypoint 数量 & 段长分布（稀疏器是否过密/过疏）
- 曲率或 heading 变化率（DWB 模式才有意义）

---

## 4. 稳定性维度补三条

1. **失败率是第一位的稳定性指标，用户漏了。** 只统计 10 次成功的终点分布 = 幸存者偏差。
   必须分类记录：SUCCEEDED / ABORTED / 超时 / 起步 caster 死锁 / 中途卡住 / 撞。
   `record_brake_trace` 已经在抓 `action_status`（4/6…），直接用。
2. **起点复位会污染样本。** 每次跑完机身停在带漂移的终点，若不物理复位到同一起点、且不
   重新给 lightning `/initialpose`，第 N 次就带着第 N-1 次的漂移。**每次跑前：胶带复位机身 +
   RViz 重给初始位姿**（否则测的是累积漂移不是单次重复性）。
3. **10 次只够看均值+方差和抓粗大故障，撑不起 CEP95/尾部。** 分布尾部（偶发大偏）要 ~30 次。
   建议：10 次做冒烟 + 均值/std；要报 CEP95 就上 30。

---

## 5. 补充维度：单点 A→B 覆盖不了失败模式

一个固定 goal 往往只考直行。这台车的历史坑分布在不同动作里，固定用例应是**小场景套**：

| 场景 | 考什么 | 关联历史坑 |
|---|---|---|
| S1 纯直行 3~5m | 起步静摩擦、终点过冲、min_cmd_v | 起步死锁 / 1m goal_brake |
| S2 原地转 90° / 180° | 旋转超调、caster 拖尾、sign-latch | [2026-07-01 旋转复盘] / 180°翻向 |
| S3 L 形 turn-drive-turn | 段间衔接、waypoint 稀疏、入位几何 | waypoint_follower 分层容差 |
| S4 近距离入位（approach_dist） | 终点 yaw、进场直入 | 操作前对位 |

每个场景独立跑 N 次、独立统计。一张总表能立刻看出"哪个动作最不稳"。

---

## 6. 混淆变量（这台车物理上敏感，必须记录）

这套硬件对以下量敏感（见根 CLAUDE.md §7），不控/不记就无法复现：
- **电池电压**：低力矩底盘，电压掉 → 力矩掉 → 过冲/死锁行为变。**每次跑记电压。**
- **地面/caster 摩擦**：换地面轨迹就变。固定同一块地。
- **陈旧 pyc**：改完 python 若没清 `__pycache__`+rebuild，测的是旧 bytecode
  （见 [[project_david_stale_pyc_trap]]）。**测前清 pycache + 验证 param 真生效。**
- 冷启动 vs 热机、是否带臂负载。

---

## 7. 建议的固定用例落地（工具已基本齐）

现成件：`semantic_pose_navigator.py`（从 `named_poses.json` 发固定 goal，可脚本化）+
`record_brake_trace.py`（20Hz 全链 CSV）+ AprilTag 真值。缺的是**一个串起来的 runner**：

```
每次 run:
  1. 人工把机身复位到胶带起点 (S1..S4 各自固定起点)
  2. RViz / initialpose 重新初始化 lightning 定位
  3. 记录: 电池电压、场景 id、run 序号
  4. 起 record_brake_trace → 发固定 goal (semantic_pose_navigator --target <id>)
  5. 等 action 结果 (SUCCEEDED/ABORTED/超时)
  6. 读 AprilTag 真值终点位姿 → 落一行 (真值 x,y,yaw + goal + action_status)
  7. 停 trace，归档 CSV
汇总: 每场景 N 行 → 出下面的指标表
```

CSV 里注意：现有 `odom_x/y` 保留做**相对轨迹形状**可以，但**终点误差一律用真值系**，
别用 odom。三条轨迹（plan / waypoint / 真值实际）分开存。

---

## 8. 指标速查表（建议最终报告就按这张出数）

| 维度 | 指标 | 真值需求 |
|---|---|---|
| 可靠性 | 成功率、各失败类型计数 | action_status |
| 稳定性(位置) | 终点 x/y 的 std、CEP50/CEP95、散点图 | AprilTag/卷尺 |
| 稳定性(朝向) | 终点 yaw 的 std、max | AprilTag |
| 精确性(控制) | 自报到达 − goal（位置+yaw，均值/max） | 无需外部 |
| 精确性(总) | 真值 − goal（位置+yaw，bias vs 散布分开看） | AprilTag/卷尺 |
| 定位漂移 | 真值 − 自报（终点处） | AprilTag |
| 轨迹合理性 | 路径长度比、waypoint 数/段长、停走次数 | 真值轨迹 |
| 跟踪性能 | cross-track RMS/max（对 waypoint 折线或 plan）、每 wp 过冲、到达用时 | 真值轨迹 |

> bias（系统性偏移）和 spread（随机散布）**一定分开报**：bias 大 = 标定/容差问题（可调掉）；
> spread 大 = plant 随机性/摩擦（难消，改硬件）。混在一个"平均误差"里就丢了这个判断。

---

## 9. TL;DR 给用户的四条改动建议

1. **先定真值源**（AprilTag 推荐）——否则精确性/稳定性两项无意义。
2. **终点误差拆三层**（控制误差 / 定位漂移 / 总精度）+ **加终点 yaw 误差**。
3. **轨迹/跟踪先声明跑哪套栈**；waypoint 模式下基准是稀疏折线不是全局 plan，录三条线。
4. **补：失败率、场景套（S1-S4）、电池/地面/pyc 混淆控制、起点每次复位**。10 次做冒烟、
   要尾部上 30。
