# 2026-05-18 Nav2 直行左偏诊断 + david_move 弧形角速度地板修复

## 摘要

**症状**：nav2 走直线总是朝左偏；teleop 直接发同样幅值的 `linear.x=0.2, angular.z=0` 不偏。

**根因**：`david_move.py` 的弧形角速度地板 `min_angular_z = 0.20 rad/s` 把 DWB
为跟踪路径而发出的小角速度修正（如 ±0.026 rad/s）放大 5–8 倍至 ±0.20 rad/s，
导致直行段被强迫执行明显左转 → DWB 后续发 `-0.5` 满负满方向反向纠正 → 机器人
来回摆动并整体偏一侧。

**修复**：`DEFAULT_MIN_ANGULAR_Z: 0.20 → 0.0`。带线速度运动时不再抬升小角速度；
原地转向地板 `min_stationary_angular_z = 0.35` 保留（仍需克服静摩擦）。

**验证**：单测从 9 增到 11 条全过，含新行为回归覆盖。`colcon build` 通过。

---

## 1. 现象

| 触发方式 | linear.x | angular.z | 实际表现 |
|---|---:|---:|---|
| teleop 持续按 'i' | 0.20 | 0.00 | **直行**，无偏移 |
| nav2 直行 goal | DWB 决定 | DWB 决定 | **持续朝左偏**，超出 xy_goal_tolerance |

teleop 不偏 ⇒ 排除底盘标定（轮径不对称、encoder 计数偏置、center-of-mass 偏一侧）
⇒ 问题在 nav2 上层链路。

## 2. 诊断方法：cmd_vel 链路全量监控

写了独立 Python 脚本 `david_cmd/scripts/record_nav_drift.py`，同时订阅 cmd_vel
管线 4 段 + 反馈 odom：

```
DWB ── /cmd_vel_nav ── velocity_smoother ── /cmd_vel_smoothed
    ── collision_monitor ── /cmd_vel_safe
    ── david_move ── /cmd_vel ── wheeltec_robot_node
                                       └── /odom
```

按时间分箱算 `angular.z` 的均值/标准差/极值，只统计 `|linear_x| > 0.02 m/s` 的
"运动期间"样本，避免 idle 段稀释。

执行：

```bash
source /opt/ros/humble/setup.bash
source /home/agx/ros2_ws/install/setup.bash
python3 /home/agx/ros2_ws/src/david_cmd/scripts/record_nav_drift.py \
    --duration 30 --label drift_run1
# 同步在另一 shell 发 nav goal
```

输出 `/tmp/nav_drift/drift_run1_{summary.txt, *.csv}`。

## 3. 实测数据

### 3.1 全段统计

| 话题 | n_total | n_active | ang_mean | ang_std | ang_min | ang_max |
|---|---:|---:|---:|---:|---:|---:|
| `/cmd_vel_nav` (DWB) | 86 | 85 | −0.192 | 0.313 | −0.500 | +0.500 |
| `/cmd_vel_smoothed` | 226 | 224 | −0.057 | 0.234 | −0.500 | +0.500 |
| `/cmd_vel_safe` | 226 | 224 | −0.057 | 0.234 | −0.500 | +0.500 |
| `/cmd_vel` (final) | 1400 | 1125 | −0.058 | 0.241 | −0.500 | +0.500 |
| `/odom` | 600 | 451 | −0.056 | 0.229 | −0.596 | +0.465 |

观察：

- DWB 极化采样到 ±0.5（max_vel_theta）—— 不是细修正，是"非满即零"的来回摆。
- 进 smoother 后均值从 −0.192 缓和到 −0.058 —— smoother 在做时域平均。
- `cmd_vel_safe ≡ cmd_vel_smoothed` ⇒ collision_monitor 没改 cmd_vel（这次没触发）。
- `/cmd_vel` 与 `/cmd_vel_safe` 接近 ⇒ david_move **看似没大幅改 angular**（但这是
  全段均值，掩盖了局部放大）。

### 3.2 时间分箱（关键）

把 nav 期间分 4 段平均：

`/cmd_vel_nav` (DWB) 分箱：

| 时间窗 | n_active | ang_mean |
|---:|---:|---:|
| 5.2 – 8.7 s | 31 | **+0.018**（"直行"段） |
| 8.7 – 12.2 s | 13 | −0.460（强右转）|
| 12.2 – 15.7 s | 19 | +0.004（"直行"段）|
| 15.7 – 19.2 s | 22 | −0.500（强右转）|

`/odom` 分箱：

| 时间窗 | n_active | ang_mean |
|---:|---:|---:|
| 0.1 – 7.6 s | 41 | **+0.098**（机器人左转 ~0.1 rad/s）|
| 7.6 – 15.1 s | 150 | −0.126 |
| 15.1 – 22.6 s | 150 | −0.092 |
| 22.6 – 30.1 s | 110 | +0.029 |

**关键对照（bin 1，DWB 试图"直行"）**：

- DWB 指令均值 `+0.018`（几乎零）
- 实际 odom 反馈均值 `+0.098`（左转 0.1 rad/s）

**机器人在 DWB 命令"几乎不转"时，实际左转 0.1 rad/s。**

### 3.3 单帧追踪

DWB 在直行段的典型 raw 输出（来自 `drift_run1_cmd_vel_nav.csv`）：

```
t=5.60s  linear=+0.200  angular=+0.026
t=6.10s  linear=+0.200  angular=+0.079
t=7.62s  linear=+0.200  angular=-0.079
```

这都是小幅角速度精细修正。但经 david_move 之后变成 ±0.20 / ±0.50。

## 4. 根因定位

`david_cmd/david_cmd/david_move.py` 的 `sanitize_twist()`：

```python
def angular_deadzone_minimum(self, msg):
    if msg.angular.z == 0.0:
        return 0.0
    linear_motion = abs(msg.linear.x) > 0.0 or abs(msg.linear.y) > 0.0
    if linear_motion:
        return self.min_angular_z          # ← 旧默认 0.20 rad/s
    if abs(msg.angular.z) >= abs(self.stationary_angular_threshold):
        return self.min_stationary_angular_z   # 0.35
    return self.min_angular_z              # 0.20

twist.angular.z = lift_deadzone(
    clamp(msg.angular.z, self.max_angular_z),
    min(angular_minimum, self.max_angular_z),
)
```

`lift_deadzone` 行为：

```python
def lift_deadzone(value, minimum):
    if value == 0.0 or minimum <= 0.0 or abs(value) >= minimum:
        return value
    return minimum if value > 0.0 else -minimum
```

代入 DWB 实际输出 `(linear=+0.20, angular=+0.026)`：

1. `linear_motion = True`
2. `angular_minimum = min_angular_z = 0.20`
3. `lift_deadzone(+0.026, 0.20)`：因 `|0.026| < 0.20`，返回 **+0.20**

输入 +0.026 → 输出 **+0.20**，放大 7.7×。

### 4.1 为什么旧默认值是 0.20？

来自更早的"低速底盘起步不灵"经验：底盘电机有静摩擦，命令 0.05 rad/s 不会动，
所以在控制层把"任何非零的小命令"抬到一个能动的值（0.20）。

**但这逻辑只在原地转向时成立**（两轮要从零开始反向旋转）。当车身已经在
前进时，两轮已经在转，差速即角速度，电机不再受静摩擦限制 —— 此时抬升纯属
对 DWB 闭环跟踪的暴力放大。

### 4.2 振荡链条

完整的反馈环：

1. DWB 看 robot 略偏路径，发 angular=+0.026（小修正）
2. david_move 抬到 angular=+0.20 (5×放大)
3. 机器人实际 ang ≈ +0.10（85% 跟踪）—— 明显左转
4. DWB 下一周期看到机器人左转过头，发 angular=-0.5（max 满方向反向）
5. david_move 透传 -0.5（已在 max，不再 lift）
6. 机器人实际 ang ≈ -0.40 —— 明显右转
7. DWB 看见过冲，发 0
8. Goto 1

观察数据完美对应：cmd_vel_nav 极化在 ±0.5，odom 持续抖动并带左偏。

## 5. 修复

### 5.1 代码改动

`david_cmd/david_cmd/david_move.py`：

```python
# 旧
DEFAULT_MIN_ANGULAR_Z = 0.20

# 新
DEFAULT_MIN_ANGULAR_Z = 0.0   # 弧形运动时不抬升角速度
DEFAULT_MIN_STATIONARY_ANGULAR_Z = 0.35   # 原地转向地板，保留
```

**只动默认值，不动算法**。如果未来确认有底盘弧形死区，能用启动参数把它调回。

新行为：

| 输入 (linear, angular) | 旧输出 | 新输出 | 含义 |
|---|---|---|---|
| (0.20, +0.026) | (0.20, +0.20) | (0.20, +0.026) | 弧形修正不再放大 ✅ |
| (0.20, +0.50) | (0.20, +0.50) | (0.20, +0.50) | clamp 不变 |
| (0.00, +0.05) | (0.00, +0.35) | (0.00, +0.35) | 原地转向仍抬 ✅ |
| (0.00, +0.01) | (0.00, +0.20) | (0.00, +0.01) | 极小原地转向不再抬（无影响，远低于物理死区）|
| (0.00, 0.00) | (0.00, 0.00) | (0.00, 0.00) | 零保持 |

### 5.2 单测覆盖（TDD）

`david_cmd/test/test_david_move_node.py` 新增 2 条：

| 测试 | 覆盖 |
|---|---|
| `test_arc_motion_passes_small_angular_through_by_default` | 新默认配置下，arc 模式小角速度透传 |
| `test_stationary_yaw_still_lifts_after_arc_default_change` | 新默认配置下，原地转向仍抬升 0.35 |

旧测试 `test_moving_yaw_uses_lower_arc_deadzone` 保留，但显式传入
`min_angular_z=0.20` —— 文档化"如果设非零 lift 还能用"。

执行：

```bash
cd /home/agx/ros2_ws/src/david_cmd && source /opt/ros/humble/setup.bash && \
  PYTHONPATH=".:$PYTHONPATH" python3 -m pytest test/test_david_move_node.py -v
# 11 passed in 1.75s
```

构建：

```bash
cd /home/agx/ros2_ws && colcon build --packages-select david_cmd
# OK
```

## 6. 相关同期改动（本日 2026-05-18）

| 改动 | 文件 | 目的 |
|---|---|---|
| `target_frame: base → base_footprint` | `2d_david_nav2_bringup.launch.py` | 修 /scan 前 180° 实际是机身左半圆的问题（base_link 是 IMU 系非 REP-105；base_footprint 才是 +X=前）|
| `angle_min/max: ±π → ±π/2` | 同上 | 过滤后方 180° 噪声 / 自影 |
| `min_height: 0.0 → 0.10` | 同上 | 过滤地面反射、机身底部自影 |
| `local_costmap.inflation: enabled=False/0.20 → True/0.40` | `nav2_params_2d.yaml` | 改回保守路线，inflation ≥ inscribed 0.375 |
| `global_costmap.inflation: enabled=False/0.30 → True/0.40` | 同上 | 同上 |
| `DWB vx_samples: 15 → 5` | 同上 | 粒度 0.013→0.05 m/s，避免 DWB 选不可达低速 |
| `velocity_smoother.feedback: OPEN_LOOP → CLOSED_LOOP` | 同上 | /odom 修复到 22Hz 后可用闭环（待验证）|
| `velocity_smoother.odom_topic: /lightning/odometry → /odom` | 同上 | 修正为 wheeltec 编码器 odom |
| **`david_move.min_angular_z: 0.20 → 0.0`** | `david_move.py` | **本报告主修复** |

## 7. 下次验证清单

按这个顺序跑：

1. 重启 nav2（让 launch yaml 改动 + 新 build 的 david_cmd 都生效）。
2. 用 `record_nav_drift.py` 跑同样直线 goal，复测 30s：
   ```bash
   python3 /home/agx/ros2_ws/src/david_cmd/scripts/record_nav_drift.py \
     --duration 30 --label after_fix_run1
   ```
3. 期望 summary：
   - `/cmd_vel_nav.ang_mean` 接近 0，std 远低于旧的 0.31（不再极化采样）
   - `/cmd_vel.ang_mean` 与 `/cmd_vel_smoothed.ang_mean` 接近（david_move 不再放大）
   - `/odom.ang_z.ang_mean` 接近 0
4. 物理观察：机器人直线段无明显偏移。
5. 也可同时观察 rviz：
   - `/scan` 应是机身前向半圆
   - 全局/局部 costmap 启动时不再报 "inflation < inscribed" ERROR
   - global path 不再因 0.30 < 0.375 inflation 贴墙规划

如果还偏，下一个怀疑是 CLOSED_LOOP smoother 在重载下的反馈过补偿 ——
切回 OPEN_LOOP A/B 对比即可定位。

## 8. 文件改动总览

```
david_cmd/david_cmd/david_move.py            # 核心修复
david_cmd/test/test_david_move_node.py       # +2 单测
david_cmd/scripts/record_nav_drift.py        # 新增诊断脚本
david_cmd/docs/report/2026-05-18-nav2-left-drift-david-move-deadzone.md  # 本报告
```

原始诊断数据：
- `/tmp/nav_drift/drift_run1_*.csv` — 5 个话题 raw 时序
- `/tmp/nav_drift/drift_run1_summary.txt` — 全段统计表
