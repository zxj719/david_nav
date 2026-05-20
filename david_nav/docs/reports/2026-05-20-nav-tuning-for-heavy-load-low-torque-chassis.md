# 2026-05-20 高负载低力矩底盘的 Nav2 调试方法

> 背景：David 是 70 kg 量级差速底盘 + 双臂 + 双手，主驱动是小电机大负载（cmd 0.7
> m/s 实际跑不到 0.5），且带被动万向轮（caster）。这种"动力受限 + 拖尾"组合在
> 标准 nav2 stack 上会暴露一系列**结构性**问题：原地旋转打滑、直行后旋转启动死
> 锁、目标点过冲、180° 对称点 ω 翻向震荡。这份文档汇总了一整轮迭代后行之有效的
> 配方。

## 1. 核心机制概览

整套链路在 nav2 之外**新增一层 `cmd_vel_separator`**，夹在
`david_move → /cmd_vel → wheeltec_robot_node` 之间，是所有底盘补偿逻辑的归宿：

```
DWB → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed
    → collision_monitor → /cmd_vel_safe
    → david_move → /cmd_vel_raw
    → cmd_vel_separator → /cmd_vel
    → wheeltec_robot_node (STM32 PI 电机环)
```

separator 同时订阅 `/odom`（实测速度）、`/plan` 或 `/goal_pose`（目标）+ `/tf`。

## 2. 高负载特定问题与配方

### 2.1 万向轮拖尾导致的角速度过冲（不是 PI 过冲）

**现象**：cmd 0.5 rad/s 旋转 180°，实际转 200° 才停。

**机理**：原地旋转时 caster 被甩斜，机身有角动量 (½·I·ω²)；caster 横向静摩擦
是唯一减速力。cmd 切到 0 后，机身按角动量 / 摩擦比继续滑过去，过冲 ~20-45°。

**配方**：separator 在 `|angle_to_goal| < angle_brake_threshold_deg`（默认 30°）
时**主动反向制动**：
```
cmd_w = -k_brake * actual_w   (clamp 到 ±max_brake_w=0.5)
```
反向 cmd 让 driven wheels 改向，caster 摩擦反相，从被动拖尾变成主动制动。

参数：`angle_brake_enabled`, `angle_brake_threshold_deg`, `k_brake`, `max_brake_w`,
`brake_release_w`, `brake_timeout`。

### 2.2 旋转后直行启动死锁（万向轮卡）

**现象**：纯旋转结束、给 0.1 m/s 直行，一侧轮 stall，机器人绕 stall 轮转。

**机理**：caster 在旋转方向上锁定，给小线速度不足以踢开横向摩擦力。

**配方**：**二值化** `binary_output: True`。separator 在 LINEAR 模式时强制输出
`v_max_ref`（默认 0.5 m/s），不再让 smoother 的 ramp 中间值（0.05, 0.1, 0.2）
落进 caster 死区。任何 LINEAR 命令都是"满速一脚踹"。

### 2.3 终点不能停（goal 过冲）

**现象**：robot 到达 goal 仍以 0.5 m/s 冲出去 0.5-0.7 m 才停。

**机理**：
- BT goal_checker 在 `xy_tolerance` 触发 success；controller_server 发 0 cmd
- velocity_smoother 按 decel_lim 慢慢 ramp 到 0
- 期间 separator binary mode 一直吐 v_max_ref（吃掉 smoother 的减速）
- chassis 物理 decel 慢（实测 0.29 m/s²）

**配方组合**：
1. **1m goal_brake**：dist_to_goal < `goal_brake_dist`（默认 1.0m）且 actual_v
   > release 时，**绕过 input_dead 检测直接进 BRAKE_LINEAR**，发反向 cmd。
2. **主动 brake**：`cmd_v = -k_brake * actual_v`（clamp ±max_brake_v=0.30）。

参数：`goal_brake_enabled`, `goal_brake_dist`, `brake_release_v`, `max_brake_v`。

### 2.4 180° 对称点 ω 翻向震荡

**现象**：goal 在身后时，DWB 在 +0.5 / 0 / -0.5 之间随机翻向，净旋转 ≈ 0。

**机理**：DWB 用 binary `vtheta_samples=3` ⇒ 候选 (−max, 0, +max)。当 |angle_to_goal|
≈ π，两个方向的 GoalAlign/GoalDist 得分对称，Oscillation critic 又因为底盘还没
真转所以没标记 → 翻向自由。

**配方**：**angular sign-latch**。separator 进入 ANGULAR mode 时锁定 ω 符号，
ANGULAR ↔ BRAKE_ANGULAR 切换不解锁；只有进 LINEAR / 真正 IDLE 才解锁。

### 2.5 DWB 单 tick 抽风触发 BRAKE 来回切

**现象**：旋转过程中，STATE 在 ANGULAR ↔ BRAKE_ANGULAR 间高频跳，actual_w 被
反向 cmd 抹掉，净旋转速度极低。

**机理**：DWB 偶尔吐 `w=0.01` 一两个 tick，`ang_norm=0.024 < min_norm_threshold=0.05`
触发 input_dead → BRAKE → 下一 tick 又 ANGULAR。

**配方**：**input_dead 时间滞回**。`input_dead_persist_sec`（默认 0.30s）—— 必
须**持续**低于阈值才算真死，单 tick 抽风被忽略。

### 2.6 速度上限四层不一致 → 隐藏 bug

**坑**：nav2 stack 至少四个地方限 max speed，任何一处不一致都会无声 bug。

| 层 | 字段 |
|---|---|
| DWB | `max_vel_x`、`max_speed_xy`（**两者必须相等**否则 isValidSpeed 过滤端点）|
| velocity_smoother | `max_velocity[0]`、`max_accel[0]`、`max_decel[0]` |
| david_move | `max_linear_x`、`min_linear_x` |
| cmd_vel_separator | `v_max_ref`（仅归一化，不卡值）|

**配方**：调参时**四处同步**。例如目标 0.5 m/s 就把 DWB 两个、smoother、
david_move 都设 0.5；separator 的 v_max_ref 也跟着 0.5。

特别注意：DWB 内部 `xy_theta_iterator::isValidSpeed()` 会把
`|v_xy|² > max_speed_xy²` 的样本直接拒掉。如果 `max_vel_x=0.7, max_speed_xy=0.5`，
端点 vx=0.7 会被过滤，机器人完全不前进。

## 3. DWB 极致离散化

binary_output 要 work，DWB 必须也只输出端点轨迹，否则 smoother 平滑后中间值在
separator 端被强制抬，等于 separator 给 chassis 一个**伪稳态**误差（cmd=0.7 但
upstream 想 0.3）。配置：

```yaml
FollowPath:
  vx_samples: 2                # 只采 [0, max_vel_x]
  vy_samples: 1
  vtheta_samples: 3            # 采 [-max, 0, +max]
  min_vel_theta: 0.0           # 必须为 0！否则 ω=0 那档被 deadband 过滤掉
  min_speed_theta: 0.0
  # 关键 critic 配比 (用户保证起点→终点无障碍场景)
  critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "GoalDist"]
  # 注意：移除了 PathAlign / PathDist。empty costmap 时它们等同于 GoalAlign/GoalDist
  # 但额外的"贴 path 奖励"会阻碍纯前进决策。
  RotateToGoal.scale: 1.0
  GoalAlign.scale: 6.0
  GoalDist.scale: 10.0
```

## 4. 完整的 separator 参数表（生产配置）

```yaml
cmd_vel_separator:
  input_topic: /cmd_vel_raw
  output_topic: /cmd_vel
  odom_topic: /odom
  # 模式归一化
  v_max_ref: 0.50
  w_max_ref: 0.50
  hysteresis_margin: 0.10
  min_norm_threshold: 0.05
  input_dead_persist_sec: 0.30  # 关键：抵消 DWB 单 tick 抽风
  binary_output: True            # caster 友好
  # 发布节奏 + 日志
  publish_hz: 20.0
  log_period_sec: 1.0
  timeout_sec: 0.5
  # 主动制动 (Plan D)
  brake_enabled: True
  k_brake: 1.0
  max_brake_v: 0.30
  max_brake_w: 0.50
  brake_release_v: 0.05
  brake_release_w: 0.10
  brake_timeout: 2.0
  # 距离-触发制动 (1m brake)
  goal_brake_enabled: True
  goal_brake_dist: 1.0
  # 预测式角制动
  angle_brake_enabled: True
  angle_brake_threshold_deg: 30.0
  # 让位 (键盘遥控时不要抢话题)
  silent_on_idle: True
  idle_grace_sec: 0.5
  # heading guard 默认关闭（DWB 自然处理 + sign-latch 已经足够）
  heading_guard_enabled: False
```

## 5. 经过验证的调参经验值

| 参数 | 推荐起点 | 调整方向 |
|---|---|---|
| `max_vel_x = max_speed_xy` | 0.50 m/s | 终点过冲大就降；万向轮卡死就升（升前先确认 stm32 P 控有效跟踪范围）|
| `acc_lim_x` | 2.5 m/s² | 启动想更猛就升；smoother 同步 |
| `decel_lim_x` | -2.5 m/s² | 配套 |
| `xy_goal_tolerance` (general) | 0.30-0.50 m | 旋转后航向误差大、横向飘出 tolerance 就升 |
| `xy_goal_tolerance` (FollowPath) | < goal_checker | 否则 RotateToGoal critic 先于 BT-success 触发 |
| `angle_brake_threshold_deg` | 30° | 过冲多少度就调多少 |
| `max_brake_w` | 0.50 | brake 不够狠就升到 0.7 |
| `inflation_radius` | 0.40 m | 必须 ≥ inscribed (0.375)，否则机身角伸进未膨胀区域 |

## 6. 何时考虑改 STM32 固件

separator 的 brake 是闭环（对 actual_v/actual_w 反向 P 控）。但有两个 chassis-端
顽疾它没法根治：

1. **静态摩擦死区**：cmd < ~0.05 m/s 时电机不响应。pass-through 减速段在 deadzone
   以下时 chassis 已经在物理滑停，不再受 cmd 控制。
2. **稳态跟随误差**：纯 P 控制 cmd 0.5 实测 0.35。Plan D 起步通过 binary 把 cmd
   抬到顶端，但 actual 仍是 P 跟踪结果。

要根治：把 STM32 的纯 P 改成 PID（至少加 I 项消除稳态误差）。改完后可以考虑：
- 关 binary_output（让 smoother 自然减速）
- 终点 brake 不需要那么早
- 角速度 brake 也可以不那么狠

## 7. 验证 checklist

跑完一次完整导航后，看 `/tmp/david_nav_logs/brake_trace_*.csv`：

1. **起点旋转**：`OUT mode=ANGULAR w=±0.5` 持续，**不出现 sign 翻转**（sign-latch
   生效）
2. **进直行**：`STATE ANGULAR → LINEAR`，`OUT mode=LINEAR v=+0.5`
3. **接近 goal 时**（dist < 1m）：`STATE LINEAR → BRAKE_LINEAR`，`OUT v=-0.X`
   （反向 cmd 真的发了）
4. **`odom_v`** 在 BRAKE 期间快速从 0.4 衰减到 0（< 0.5s）
5. **过冲距离** < 0.30m（合理）

如果第 3 步 `STATE LINEAR → BRAKE_LINEAR` 永远不出现 → 大概率是 TF 算不出
dist_to_goal（d2g=n/am）。检查 `/initialpose` 是否设置过、TF 树是否完整。

## 相关文档

- [一键启动集成 debug 经验](2026-05-20-integrated-debug-launch-experience.md)
- [Nav2 调试常见坑（RViz 视角）](2026-05-20-rviz-and-tf-pitfalls.md)
- [Plan D 落地报告](2026-05-19-plan-d-cmd-vel-mutex-and-caster-deadlock.md)
