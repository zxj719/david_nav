# 底盘控制参数确定方法 — 基于 slam_gt 真值 + 轮速计（设计/Spec）

> 2026-06-23 · feature/david_main
>
> 把"换底盘后重新确定底盘控制参数"固化成一套**可复跑的方法**：用 **轮速计
> (`/odom` twist)** 作速度真值定 **速度区间**，用 **slam_gt (`/groundtruth/pose`)**
> 作位置真值定 **位置环 PID** 并做 **导航测试**。本 spec 既是方法文档，也是本次
> 执行依据（"按照其去进行"）。
>
> 关联：开环/闭环表征脚本与首轮结果见
> [2026-06-23-new-chassis-retune.md](../../reports/2026-06-23-new-chassis-retune.md)；
> 首版设计见
> [2026-06-23-new-chassis-speed-interval-and-position-pid-retune-design.md](2026-06-23-new-chassis-speed-interval-and-position-pid-retune-design.md)。
> 本 spec 在其上**增加 slam_gt 真值 + 导航测试**两层。

## 1. 目标与适用场景

**何时跑**：底盘硬件变化（电机/轮子/轮距/负载/caster）后，旧
`position_controller.yaml` 不再适配时。

**产出**：
1. 速度区间：`min_cmd_v / v_max / a_max`、`min_cmd_w / w_max / alpha_max`。
2. 位置环：`dist.Kp/Kd`、`yaw.Kp/Kd`（结构固定 **PD，`Ki=0`**，理由见 §6）。
3. 导航测试结论：在真值定位下，调好的位置环跑实际导航的成功率/真实精度/避障余量。

## 2. 核心原则 — 按"谁擅长"分配真值

| 真值源 | 量什么 | 为什么 | 用在 |
|---|---|---|---|
| `/odom` `twist.twist`（轮速计） | **速度** | 轮速是本地瞬时量，**不积分→不漂** | Phase A 开环 |
| slam_gt `/groundtruth/pose`（map 帧） | **绝对位置/朝向** | `/odom` **位置**漂移巨大（实测一轮旋转漂 >2m） | Phase B 精度、Phase C 导航 |
| `/scan`（雷达） | **障碍距离** | 真值定位丢失时的兜底安全；忽略本体臂自回波(~0.74m) | 全程安全 |

**这是本方法的中心**：速度标定不需要也不该用会漂的 odom 位置；位置精度与导航
**必须**用 slam_gt 真值，不能信 odom 绝对位置。

## 3. 链路与依赖

```
# Phase A（开环，纯 plant）
chassis_characterize.py ──Twist──▶ /cmd_vel ──▶ wheeltec ──▶ STM32
                                     ▲                  │
                                     └──── /odom ◀───────┘ (twist=速度真值)

# Phase B/C（闭环 + 导航，slam_gt 真值）
slam_gt localization:  map ─(slam_toolbox 锁图)→ odom ─(odom_to_base_tf)→ base_link
                       groundtruth_pose_publisher: map->base_footprint → /groundtruth/pose
Phase B: field_test ─▶ /position_controller/move ─▶ /cmd_vel ─▶ wheeltec
         （同时订 /groundtruth/pose 记真实位姿；看门狗也用真值）
Phase C: nav_trial_runner ─▶ waypoint_follower ─▶ /position_controller/move ─▶ /cmd_vel
         nav2 planner_server + global_costmap（规划/代价图，不接管执行）
```

**slam_gt 启动**（已有脚本，不重写）：
`david_slam_gt/scripts/launch_slam_gt.sh localization`，定位锁 `maps/david_map`。
雷达/IMU 若已在跑用 `USE_LIDAR=0 USE_IMU=0` 复用，避免重复驱动 + sudo。
脚本已处理 `set -u` 下 source ROS 的坑（`set +u; source; set -u`）。

**与 nav2 同跑约定**（来自 slam_gt launch 注释 + 项目记忆）：nav2 侧
`publish_robot_state:=false enable_scan_bridge:=false`、**不起 lightning**
（机器人 TF + /scan_slam 由 slam_gt 自洽提供，避免重复/冲突）。

## 4. Phase A — 速度区间（开环，轮速真值）

**已实现**：`scripts/chassis_characterize.py`。直发恒定 `Twist` 到 `/cmd_vel`
（绕过 separator/david_move = 纯 plant），分直行/旋转扫描，每档保持 ≥2s、末
0.7s 取**中位数**（抗起步 stick-slip lurch），每档重复 3–5 次，**最坏 caster 预置**
（直行档前小转甩定脚轮；旋转档前小直行 = "直行后"最坏工况），取 min/median/max。

**判读**：死区下限 = 每次重复都起步的最低档 + 裕量；饱和拐点 = actual 中位增幅
连续 <8% 的档（本底盘直行无饱和，旋转破缸高）。

**slam_gt 在此可选**：仅用于扫描期间监控离墙余量；速度判读不依赖它。

**首轮结果（2026-06-23，已落 yaml）**：`min_cmd_v 0.14`、`v_max 0.20`、
`min_cmd_w 0.55`、`w_max 0.75`、`a_max 0.5`、`alpha_max 0.8`、`rotate_lead 0.40`。

## 5. Phase B — 位置环 PID（闭环，slam_gt 真值重验）

**做什么**：`scripts/pid_sweep.py` 套 `position_controller_field_test.py` 扫
`dist.Kp/Kd`、`yaw.Kp/Kd` 与 `w_max × rotate_lead`，按 超调+残余+重复散布 排序。

**本次增量（slam_gt 真值）**：给 `position_controller_field_test.py` 加
`--groundtruth` 开关：
- 订阅 `/groundtruth/pose`（map 帧）。
- 指标 = **真实世界**位置/朝向误差（goal 起点真值 → 终点真值），而非 odom 相对残余。
- **安全框看门狗切到真值位置**（odom 安全框跨漂移无意义）；真值短暂丢失则保守急停。
- 真值不可用时回退 odom 相对（保持向后兼容）。

**结构固定 PD**（§6）；增量只换"量得更真"，不改控制律。
**产出**：dist/yaw 增益的真实精度 + 超调；确认 `w_max=0.75` 的 ALIGN-后-直行可靠性。

## 6. 位置环控制结构论证 — 为什么 PD（`Ki=0`）

plant 是 **速度入 / 位置出 = 自带积分器**（type-1）。

- **P**：对积分器 plant，纯 P 即给到位置 setpoint 的**零稳态误差**，是主力。
- **I — 不用且有害**：(1) plant 已含积分，加 I→type-2→过冲/振荡；(2) **死区 + I = 极限环**
  （误差小→P 出力 < 死区 floor→不动→I 积起→破缸 lurch→过冲→反号→反复 hunting），
  本 caster 底盘死区大（`min_cmd_w 0.55`）尤甚；(3) 主误差是 caster **随机散布**
  （前进 +2.3 / 后退 −2.8cm 反号，pose σ6cm），I 只能修恒定偏置，修不了随机噪声。
- **D — 留小（0.1/0.15）**：对积分器是速度阻尼，但超调真正靠**前馈**（`approach_decel`
  刹停、`rotate_lead` 提前停）压；20Hz stick-slip 上微分有噪，大 Kd 反害。

**结论**：dist/yaw 保持 **PD，`Ki=0`**。稳态精度交给 plant 积分 + 死区 floor；
超调交给前馈；到位交给 latch。仅当出现**恒定单向 droop**（坡/负载）才考虑极小、
带 anti-windup、到位带内冻结的条件积分（`Ki`/`I_max` 已预留）；平地此条件不成立。

## 7. Phase C — 导航测试（新）

**栈**：slam_gt localization（david_map）+ nav2 planner_server + global_costmap +
`waypoint_follower` → `/position_controller/move`（被测=调好的位置环）。

**新工具 `scripts/nav_trial_runner.py`**：
- 订 `/groundtruth/pose`（真值）；按一组**固定 map 目标**（3–4 个，组成回路，
  全在 david_map 已知可通行区、留足离墙余量）依次发导航 goal（经 waypoint_follower）。
- 每个 goal 到达后记：**真值位置/朝向误差 vs goal**、过程 **min `/scan` 余量**
  （排除本体臂自回波）、成功/失败、耗时、段数。
- 安全：真值越界（离场地中心超阈值）或 `/scan` 余量 < 阈值 → cancel 当前 goal。
- 输出 CSV + 汇总 + tee 日志（沿用 `_latest` 软链约定）。

**验收**：
- 每 goal 真值位置误差 ≤ `goal_dist_tol 0.2m`、朝向 ≤ `goal_yaw_tol ~10°`
  （对照 caster ~20cm 散布天花板，达不到则记录并归因 plant 而非增益）。
- 全程零碰撞：`/scan` 余量（除自回波）始终 > 0.30m。
- 回路全 goal 成功，无段间死锁/超时。

## 8. 安全（全程）

- Phase B/C 看门狗用 **slam_gt 真值位置**（非 odom）；真值丢失→保守急停。
- `/scan` 离障门限，忽略本体臂 ~0.74m 自回波（超出 self_filter 本体盒）。
- 速度温和（`v_max 0.20`、`a_max 0.5`、`alpha_max 0.8`；用户：上肢不稳）。
- 全程 `--no-confirm` 自主跑，**人守急停**；越界/碰撞预警自动停。
- 日志 tee 到 `/tmp/david_nav_logs/…` + `_latest` 软链。

## 9. 执行顺序（本次）

1. 核对 wheeltec/odom 与 slam_gt 在跑（不重复起；pre-flight pgrep）。
2. 起 slam_gt localization（david_map），确认 `/groundtruth/pose` 输出 + 真值合理
   （静止时与场地中心一致、离墙余量正常）。
3. **Phase B 重验**：给 field_test 加 `--groundtruth`，跑 straight+rotate+pose+repeat，
   读真实世界误差；必要时微调增益（结构仍 PD）。
4. **Phase C 导航测试**：写 `nav_trial_runner.py`，起 waypoint_nav（slam_gt 配方），
   跑固定回路，量真值精度 + 避障，落 CSV + 报告。
5. 报告写入 `docs/reports/`，commit。

## 10. 交付物

- 本 spec（方法文档）。
- `position_controller_field_test.py` 的 `--groundtruth` 增量（Phase B）。
- `scripts/nav_trial_runner.py`（Phase C）。
- 导航测试报告 `docs/reports/2026-06-23-nav-test-on-slamgt.md` + CSV。

## 11. 风险

| 风险 | 缓解 |
|---|---|
| slam_gt 定位丢失（场地特征少，墙超量程） | 真值丢失→急停；必要时重建图/给初值；Phase A 不依赖真值 |
| odom 绝对漂移污染任何吃 odom 位置的逻辑 | Phase B/C 一律用真值；看门狗切真值 |
| caster 随机散布使单次 nav 误差噪声大 | 回路多 goal + 报告按散布天花板归因，不盲目改增益 |
| `w_max 0.75` ALIGN 最坏偶发转不动 | Phase C 真实导航复核；必要时抬 0.80（代价超调↑） |
| 本体臂入雷达视野误判障碍 | 离障判定排除 ~0.74m 自回波（self_filter 盒外） |
