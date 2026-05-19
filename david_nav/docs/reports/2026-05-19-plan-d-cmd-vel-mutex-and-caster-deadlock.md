# 2026-05-19 Plan D 落地报告：cmd_vel 互斥硬切 + 万向轮拖尾死锁

> 范围：导航底层"先转后走"重构 (方案 D)。本 session 完成的改动 + 实跑暴露
> 的结构性遗留问题（万向轮拖尾 + STM32 纯 P 控制）+ 后续可选对策。

---

## 1. 背景与目标

需求方原话："新的 nav param 会 100% 目标方向，100% 在走直线时不加入旋转，
旋转时 100% 不加入走直线。完全无视障碍物，起点到终点的过程中我会保证是
不会遇到障碍物"。

工程语言：
1. 整段轨迹只允许"纯旋转"或"纯直行"两种态，物理输出严格互斥。
2. 障碍/膨胀层全关——costmap 全 free，NavfnPlanner 自然产出直线 path。
3. 自然纠偏：直行偏航 → 由控制律自然切回旋转态再走，不引入混合速度。

## 2. 本 session 的改动

### 2.1 整体拓扑
```
DWB → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed
    → collision_monitor → /cmd_vel_safe → david_move → /cmd_vel_raw
    → cmd_vel_separator → /cmd_vel → wheeltec_robot_node
                  ▲
                  └── 本次新增节点：硬切线/角互斥
```

### 2.2 文件清单

| 文件 | 改动 |
|---|---|
| `david_nav/config/nav2_params_2d.yaml.bak.20260519_160901` | 改前快照（备份） |
| `david_nav/david_nav/cmd_vel_separator.py` | **新增**节点：v/ω 归一化比较 + 滞回硬切 |
| `david_nav/CMakeLists.txt` | 注册 `cmd_vel_separator` executable |
| `david_nav/launch/2d_david_nav2_bringup.launch.py` | david_move 输出改 `/cmd_vel_raw`；加入 separator |
| `david_nav/config/nav2_params_2d.yaml` | costmap 全关障碍 + DWB 离散化 + 速度上限同步至 0.5 |

### 2.3 cmd_vel_separator 行为
- 每条 Twist：`lin_norm = |v_x|/v_max_ref`，`ang_norm = |ω_z|/w_max_ref`
- 状态机 IDLE / LINEAR / ANGULAR，0.10 滞回避免抖切
- 双轴 < 0.05 → 双零；输入 0.5 s 无更新 → 兜底零速
- 默认 `v_max_ref=0.50, w_max_ref=0.50, hysteresis_margin=0.10`

### 2.4 nav2_params 关键改动
| 字段 | 旧 | 新 | 说明 |
|---|---|---|---|
| `local_costmap.plugins` | `[static_layer, scan_priority_layer, inflation_layer]` | `[static_layer]` 全 `enabled: False` | 局部代价图全 free |
| `global_costmap.plugins` | `[static_layer, obstacle_layer, inflation_layer]` | `[static_layer]` 全 `enabled: False` | 全局代价图全 free |
| `track_unknown_space` (两 costmap) | true | **false** | 无层数据的 cell 默认 FREE，否则 NO_INFORMATION 卡住 planner |
| `NavfnPlanner.allow_unknown` | false | true | 兜底，lifecycle 初始化时段不会失败 |
| `FollowPath.vx_samples` | 5 | **2** | DWB 只采 `[0, max]` 两档 |
| `FollowPath.vtheta_samples` | 20 | **3** | DWB 只采 `[-max, 0, +max]` |
| `FollowPath.min_vel_theta / min_speed_theta` | 0.10 | **0.0** | 否则 ω=0 那档被 deadband 过滤掉，"纯前进"轨迹永远采不到 |
| `FollowPath.max_speed_xy` | 0.20 | **0.50** | 必须与 `max_vel_x` 同步，详见 §3 根因 |
| `FollowPath.RotateToGoal.scale` | 1.0 → 100 → **1.0** | 还原 | 该 critic 只在 xy<0.25 时激活，平时 score=0，不影响"起点先转后走" |
| `velocity_smoother.max_velocity` | `[0.2, 0, 0.5]` | `[0.50, 0, 0.5]` | 否则下游硬卡 0.2 |
| `david_move.max_linear_x` (param) | 0.20 默认 | **0.50** | 否则 david_move 又把 DWB 的 0.5 再压回 0.2 |

## 3. Debug 时间线 & 关键发现

### 3.1 第一发：参数改完后机器人原地无限旋转

**根因**：`max_vel_x=0.5` 与 `max_speed_xy=0.2` 不一致。

DWB 的 `xy_theta_iterator.cpp::isValidSpeed()`（line 99）：
```cpp
if (vmag_sq > max_speed_xy_sq + EPSILON) return false;
```

`vx_samples=2` ⇒ 端点 `[0, max_vel_x=0.5]`。`vx=0.5` 时 `vmag²=0.25 > 0.04=max_speed_xy²`
**整条前进采样被拒**。再加上 line 107 "vmag²==0 且 ω==0 → 拒"，**最终唯一存活的
轨迹是 (0, ±0.5) 两条纯旋转**。机器人物理上无前进选项。

修前 `vx_samples=5` 时端点 `[0, 0.125, 0.25, 0.375, 0.5]` 还有 0.125 漏过过滤
（0.0156 < 0.04），robot 仍能爬行至 0.125 m/s——所以这是个 Plan D 二值化**暴露**
的旧潜在 bug。

### 3.2 第二发：速度封顶在 0.2 m/s

每一层都独立地卡在 0.2：
- DWB `max_speed_xy=0.2`
- `velocity_smoother.max_velocity[0]=0.2`
- `david_move.max_linear_x=0.20`（默认）

抬一层不够，必须四层同步。键盘遥控能跑更快是因为它直接发 `/cmd_vel`，绕过整条链。

### 3.3 修后状态
- `/cmd_vel_nav` 应能看到 `(linear.x=0.5, angular.z=0)` 出现
- `/cmd_vel` 输出严格 v 或 ω 一项为 0

---

## 4. **仍未解决：万向轮拖尾死锁**（结构性问题）

### 4.1 现场表象

实跑后："可以走直线，但是效果令人窒息"。

原地旋转结束、给一个小的线速度时，**机身一侧的轮子可能直接停住**，机器人
绕该被卡轮做圆周——"直线指令"实际变成了大半径绕轮转。

### 4.2 根因分析（差速底盘 + 被动万向轮 + 纯 P 控制）

1. **拖尾不对齐**：原地旋转结束的瞬间，前/后万向轮的脚轮（caster）方向
   滞留在旋转切线方向，与即将开始的直线方向**正交或近正交**。
2. **静摩擦放大**：万向轮要把脚轮甩到与新方向对齐，需要克服一个**横向静
   摩擦峰值**。这个反作用力直接作用在驱动轮的轴向上，相当于给某一侧驱动轮
   加了一个阻力矩。
3. **STM32 是纯 P 控制**：底层电机环只看 `(target - measured) * Kp`。当一侧
   阻力矩远大于另一侧时，**两个轮子 PWM 不变但实际转速差异巨大**，且 P 控制
   有稳态跟随误差——目标 0.5 m/s 实际可能只到 0.30，而**有阻力侧可能干脆
   stall**（PWM 不够大到突破静摩擦）。
4. **不平衡正反馈**：一侧 stall → 机身绕该轮转 → 万向轮被甩得更歪 → 阻力
   不再下降 → 死循环。
5. **DWB 不知情**：上层算法看到的只是 cmd `linear.x=0.5`，没办法察觉底层
   一边在跑一边在 stall。

### 4.3 为什么是结构性问题
- 不是 cmd_vel_separator 的问题——它的输出本身是正确的 `(0.5, 0)`。
- 不是 DWB 决策的问题——纯前进的 cmd 在物理上**就是会因为非对称阻力**
  退化成转弯。
- **本质矛盾**：上层假设 v 与 ω 解耦，但物理上**万向轮姿态**这个隐藏自由
  度让 v 输入耦合出了 ω 输出。

---

## 5. 应对策略

### 5.1 方案 A：DWB 线速度极致下采样（**短期推荐**）

只保留 `vx ∈ {0, v_target}` 两档，且 **`v_target` 取高于"静摩擦临界 cmd"
的值**（如 0.5 m/s 甚至 0.7 m/s）。

**理由**：STM32 纯 P 控制有跟随误差，目标 0.5 实际可能跑 0.35。但只要 0.35
**足够踢穿万向轮静摩擦**，机器人就能进入"稳态低速直线 + 万向轮被动跟随"
的工况。

**操作**（现状已 50% 实施）：
```yaml
# 在已有 vx_samples=2 / vtheta_samples=3 之上：
max_vel_x: 0.70      # 抬到 0.70（如果 odom 反馈 0.5 实际只跑到 0.30）
max_speed_xy: 0.70   # 同步
# velocity_smoother / david_move / separator 同步抬到 0.70
```

**注意点**：
- 必须确保 acc_lim_x 足够大，让 0→0.7 在 < 1s 内完成，否则瞬态太长，
  万向轮没机会被"踢"过去。当前 `acc_lim_x=1.0` ⇒ 0.7s 加到 0.7，**偏慢**。
  建议同时把 `acc_lim_x` 抬到 2.0–3.0。
- 万向轮预对齐 cycle 见 5.4。

**缺点**：仍依赖底层 P 控制能够踢过死锁。如果某些地面摩擦极高，0.7 也不够。

### 5.2 方案 B：STM32 固件改 PID（**根治**）

把 STM32 的纯 P 电机闭环改成 PID（至少加 I 项消除稳态误差，可选 D 项抑振）。

**优点**：
- 目标速度真的能跑到，cmd 与实际 v 一致 ⇒ 上层模型对得上。
- 万向轮死锁时 PID 的 **I 项会累积误差** ⇒ PWM 自动上抬 ⇒ 能踢穿静摩擦。

**缺点**：
- 改固件涉及 STM32 烧录、回归测试，节奏慢。
- I 项调不好容易 wind-up 起飞。

**建议**：作为下一个迭代周期目标。即使方案 A 短期能用，B 是长期正确路径。

### 5.3 方案 C：突发 + 保持速度 profile（**软件层补丁**）

由 `david_move`（或者新加一个 caster_kicker 节点）实现：
- 检测 cmd_vel 从"纯 ω"切到"纯 v" 的瞬间
- 前 100–200 ms 强制以 `v_kick = 0.8 m/s` 输出（突发踢万向轮）
- 之后回落到 separator 给的目标速度

**优点**：不需要改 STM32，逻辑层就能做。
**缺点**：开环 magic number，不同摩擦地面要重调。

### 5.4 方案 D：万向轮预对齐 spin（**机械友好型软件补丁**）

每次纯旋转动作结束后、转线速度之前，**主动微动作整脚轮**：
- 旋转结束 → cmd_vel = (+0.05, 0) 持续 0.15 s（拖一下让脚轮甩前向）
- 紧接 cmd_vel = (-0.05, 0) 持续 0.15 s（再倒一下，让脚轮稳定锁定纵向）
- 再切回 (target_v, 0)

**优点**：物理上把"脚轮对齐"这个隐藏自由度显式控制掉，根治不对称阻力。
**缺点**：每次旋转→直行的过渡增加 ~0.3 s 延迟；微动作幅度太小可能没效果，
太大反而引入 yaw 偏差。

### 5.5 推荐组合
1. **立刻**：方案 A 把 `v_target` 抬到 0.7 + `acc_lim_x` 抬到 2.5，看是否
   能避开死锁（这是改 yaml 1 分钟的事）。
2. **本周**：方案 D 在 `cmd_vel_separator` 内加一个"rotation → forward 过
   渡时插入对齐微动作"的分支（< 50 行代码），无需改 STM32。
3. **下个迭代**：方案 B，把 STM32 电机环改 PID。这是长期正确解。

---

## 6. 验证 checklist（应用 5.5.1 之后）

1. `ros2 topic echo /cmd_vel_nav`：DWB 起点旋转完毕后立刻能看到
   `(linear.x ≈ 0.7, angular.z = 0)`。
2. `ros2 topic echo /cmd_vel`：separator 输出严格互斥。
3. `ros2 topic echo /lightning/odometry`（或 `/odom`）：直行段 `twist.linear.x`
   实测值。如果稳定 < 0.5 ⇒ STM32 P 控制跟踪误差严重，**需要方案 B**。
4. 录 `/tf` 看 `base_footprint` 的实际轨迹是否仍出现"大圆"（围绕单轮转）。
   如果仍有 ⇒ 方案 A 没踢穿，**进方案 D 或 B**。
5. separator stats（每 2 s 一次 INFO）：LINEAR / ANGULAR 计数都应非零。

## 7. 回滚

```bash
cp /home/agx/ros2_ws/src/david_nav/config/nav2_params_2d.yaml.bak.20260519_160901 \
   /home/agx/ros2_ws/src/david_nav/config/nav2_params_2d.yaml
# launch 文件中删除 cmd_vel_separator_node 节点和 david_move 的 parameters 覆盖。
```

或者用 git revert 这次 commit 即可。
