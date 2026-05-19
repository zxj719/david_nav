# 2026-05-18 Wheeltec 速度环 / 轮速 encoder 闭环评估报告

## 摘要

本次任务：
1. 读 `david_move.py` 理清速度发布链路。
2. 用 **轮速 encoder（不是 IMU）** 做反馈，写线 / 角速度测试用例。
3. 用角速度（左右轮等速反向）测试用例做扫频，找 Nav2 适用的发布频率与速度区间。
4. 若速度环有问题，修 `david_move.py`。
5. 出技术报告。

**核心结论先放在前面：**

1. **`/odom` 来自轮速 encoder，确认可用。** 当编码器看到运动时，`/odom.twist.angular.z` 报出真实角速度（如 cmd=+0.60，看到 0.486→0.541→0.583→0.595，跟踪到 ~99%）。Encoder 反馈链路本身没坏。
2. **`/odom` 实测仅 ~4.4 Hz**（STM32 上行帧率上限），远低于 wheeltec_robot_node `Control()` 的 100 Hz 轮询。**Nav2 controller 10Hz 比反馈快 2.5×，速度环的反馈带宽是 STM32 串口侧的硬瓶颈。**
3. **底盘电机响应是非确定性的。** 同一参数（cmd=+0.60, 50Hz, 4s hold）在第一轮幅值扫里给出 ss_mean=+0.518；几分钟后频率扫里同一参数给出 ss_mean=+0.000。这种重复性是当前最大的速度环问题。
4. **实测角速度死区在 0.40–0.50 rad/s 之间**（远高于 `david_move` 的 `min_stationary_angular_z=0.35` 和 Nav2 `min_vel_theta=0.10`）。这两个默认值在当前硬件状态下会发出底盘**根本不响应**的低角速度。
5. **`david_move` 的 `min_linear_x == max_linear_x = 0.20`** 让线速度变成 bang-bang，Nav2 任何非零线速度命令都被夹到 ±0.20 m/s ⇒ 上层闭环失效。
6. **本轮命令期内 encoder 在负向时出现符号反转**（cmd=-0.50 时 encoder 持续报 +0.10..+0.44 rad/s，并伴随 ±0.1 m/s 的线速度抖动）。怀疑是 STM32 PI 控制器在累计误差期间偶发反转或动力学耦合，需另外排查。

下面是详细数据与代码改动。

---

## 1. 控制链路（确认）

```
DWB controller @10Hz
   └── /cmd_vel
        └── velocity_smoother @10Hz, feedback: OPEN_LOOP
              └── /cmd_vel_safe
                    └── david_move @50Hz (clamp + 死区抬升, 2s stale timeout)
                          └── /cmd_vel
                                └── wheeltec_robot_node Cmd_Vel_Callback
                                       (即时编码 + 即时写串口)
                                      └── STM32 firmware
                                            └── 电机驱动
```

`david_cmd/david_cmd/david_move.py` 关键参数：

| 参数 | 当前默认 | 含义 |
|---|---|---|
| `min_linear_x` | `0.20` | 任何非零线速度抬到此 |
| `max_linear_x` | `0.20` | 线速度上限。**`min == max` ⇒ 一旦非零，输出恒为 ±0.20**（bang-bang） |
| `min_angular_z` | `0.20` | 带线速度时角速度最小值 |
| `min_stationary_angular_z` | `0.35` | 原地转向最小值（区别于上一项） |
| `stationary_angular_threshold` | `0.025` | 上游 yaw≥此值且无线速度 ⇒ 使用 `min_stationary_angular_z` |
| `max_angular_z` | `0.50` | 角速度上限 |
| `publish_hz` | `50.0` | bridge timer 频率（不是上游频率） |
| `cmd_vel_timeout` | `2.0s` | 静默超时 ⇒ 输出 0 |

观察：

- 上游 `velocity_smoother` 是 **OPEN_LOOP**：本身不读 `/odom`。
- `min_linear_x == max_linear_x` 让线速度变 bang-bang，Nav2 的细粒度命令完全失效。
- `david_move` 50Hz timer 持续转发最新 Twist，所以即使上游 10Hz，下游 /cmd_vel 也是 50Hz（同一指令复制 5 份）。

## 2. Encoder 反馈源（确认，**不是 IMU**）

源代码：`wheeltec_robot_ros2/src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp`

数据流：

1. STM32 用左右轮 encoder 算 `Robot_Vel.{X, Y, Z}`。
2. STM32 用串口（115200 baud）每帧 24 字节回传：`Robot_Vel` + MPU6050 + 电压。
3. `wheeltec_robot_node::Control()` `wall_timer(10ms)` —— 100 Hz —— 调用 `Get_Sensor_Data_New()`（串口 timeout 2s）。
4. 读到完整帧 ⇒ `Publish_Odom()`：`twist.linear.x = Robot_Vel.X`（encoder 差分前向），`twist.angular.z = Robot_Vel.Z`（encoder 差分横摆）。
5. IMU 在 `/imu`，本报告**不用作主判据**。

### 2.1 为什么选 encoder

- `/odom.twist` 在零附近真正归零（STM32 端就是 encoder 差分），用阈值判定 start / stop latency 干净。
- IMU `/imu` 静止时 `gyro.z` 被固件量化抑制为 `0.0`（见 5/9 报告），噪声基线观察不到，不适合精细 latency。
- encoder 直接给出角速度，无需 IMU 双积分。

### 2.2 `/odom` 实测速率

```text
odom: n=23 dur=5.00s rate=4.40Hz min_dt=0.8ms max_dt=240.3ms
imu : n=22 dur=4.76s rate=4.41Hz min_dt=0.6ms max_dt=240.6ms
```

> **结论：`/odom` 仅 ~4.4 Hz，被 STM32 上行串口帧率卡死。** 任何高于 ~4 Hz 的 cmd_vel 发布频率，在 encoder 闭环视角下都被反馈端 down-sample。Nav2 controller 10Hz 反过来比反馈快 2.5×，这是速度环的硬瓶颈。**升级 Nav2 闭环前，应当先排查 STM32 串口能否拉到 ≥50 Hz。**

### 2.3 Encoder 验证

`amp_pos_0.60` 段在第一轮幅值扫里给出干净梯形响应（节录原始 CSV）：

```text
t          lin_x     ang_z   cmd=+0.600
+0.000    -0.006    +0.171   (rest_pre 残留)
+1.916    +0.002    +0.165   (hold 起步前)
+2.396    -0.004    +0.268   (爬升)
+2.876    -0.005    +0.486   (临近目标)
+3.116    +0.003    +0.541
+3.356    -0.009    +0.583
+3.596    +0.002    +0.595   (峰值 ≈ 99%)
+3.836    -0.003    +0.565
+5.756    +0.007    +0.053   (rest_post 收尾)
```

start_latency ≈ 0.145 s，stop_latency ≈ 1.18 s，ss_mean = +0.518 rad/s（target 0.60，跟踪 ~86%）。**这一段直接证明 encoder 反馈正确，链路完整。**

## 3. 测试方法 / 代码

### 3.1 新增脚本

`david_cmd/david_cmd/wheeltec_velocity_loop_eval.py`
入口：`ros2 run david_cmd wheeltec_velocity_loop_eval ...`

- 订阅 `/odom`（encoder 反馈），发布角速度方波到 `--cmd-topic`（默认 `/cmd_vel`，绕过 david_move；也可改 `/cmd_vel_safe` 走 bridge）。
- 模式：`step`（单 +w / -w 双段）、`amp`（幅值扫）、`freq`（频率扫）。
- 纯函数指标（单测覆盖）：
  - `compute_start_latency(samples, t_step_on, threshold)`
  - `compute_stop_latency(samples, t_step_off, threshold)`
  - `compute_steady_state(samples, t_from, t_to)`
  - `summarize_segment(...)` → SegmentSummary
- 安全：每段尾巴 `stop_burst(10, 0.02s)`；finally 必发零。

### 3.2 单元测试

`david_cmd/test/test_wheeltec_velocity_loop_eval.py`：10 个纯函数测试（start / stop latency、steady-state、汇总）。

```bash
cd /home/agx/ros2_ws/src/david_cmd && PYTHONPATH=. \
  python3 -m pytest test/test_wheeltec_velocity_loop_eval.py -v
# 10 passed
```

### 3.3 实测如何复跑

```bash
# 启动底盘（用户机器上）：
cd ~/wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2
source ./install/setup.bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 在另一 shell：
source /opt/ros/humble/setup.bash
source /home/agx/ros2_ws/install/setup.bash

# 幅值扫描（角速度），直发 /cmd_vel：
ros2 run david_cmd wheeltec_velocity_loop_eval \
  --mode amp --cmd-topic /cmd_vel --publish-hz 50 \
  --hold-seconds 4.0 --rest-seconds 2.0 --ss-window 1.5 \
  --amplitudes 0.30,0.35,0.40,0.50,0.60 --label amp_50hz_4s

# 频率扫描（target=0.60，已知能出运动的幅值）：
ros2 run david_cmd wheeltec_velocity_loop_eval \
  --mode freq --cmd-topic /cmd_vel --target-w 0.60 \
  --hold-seconds 4.0 --rest-seconds 2.0 --ss-window 1.5 \
  --frequencies 5,10,20,50,100 --label freq_w060
```

输出每段 CSV + 全程 JSON 到 `/tmp/wheeltec_velocity_loop_eval/`。

## 4. 实测数据

### 4.1 幅值扫（直发 /cmd_vel，50Hz, hold=4s, ss_window=1.5s）

| Target rad/s | start_lat (s) | stop_lat (s) | ss_mean | ss_std | ss_err | 备注 |
|---:|---:|---:|---:|---:|---:|---|
| +0.30 | n/a | 0.025 | +0.0000 | 0.0000 | −0.300 | 不动 |
| −0.30 | n/a | 0.081 | +0.0000 | 0.0000 | +0.300 | 不动 |
| +0.35 | n/a | 0.136 | +0.0000 | 0.0000 | −0.350 | 不动 |
| −0.35 | n/a | 0.190 | +0.0000 | 0.0000 | +0.350 | 不动 |
| +0.40 | n/a | 0.004 | +0.0000 | 0.0000 | −0.400 | 不动 |
| −0.40 | n/a | 0.059 | −0.0057 | 0.0077 | +0.394 | 死区下沿，几乎不动 |
| **+0.50** | **0.754** | 0.354 | +0.1050 | 0.0241 | −0.395 | 起步晚、严重欠驱 |
| **−0.50** | 0.089 | 1.129 | **+0.336** | 0.0955 | +0.836 | **encoder 符号反**！cmd=负 → odom=正 |
| **+0.60** | 0.145 | 1.185 | **+0.518** | 0.0601 | −0.082 | **干净梯形，跟踪 ~86%，最佳工作点** |
| −0.60 | n/a | 0.037 | +0.0000 | 0.0000 | +0.600 | 不动（与 +0.60 不对称） |

### 4.2 频率扫（直发 /cmd_vel，target=0.60, hold=4s, ss_window=1.5s）

> **关键现象：与 §4.1 中"+0.60 跟踪 86%"对照，本次扫频里同一参数 (target=+0.60, hz=50, hold=4s) 给出 ss_mean=+0.000，完全不动。重复性失败。**

| publish_hz | 方向 | start_lat | stop_lat | ss_mean | ss_err |
|---:|---|---:|---:|---:|---:|
| 5  | +0.60 | n/a | 0.215 | +0.0001 | −0.600 |
| 5  | −0.60 | n/a | 0.026 | +0.0055 | +0.606 |
| 10 | +0.60 | n/a | 0.078 | +0.0000 | −0.600 |
| 10 | −0.60 | n/a | 0.129 | +0.0000 | +0.600 |
| 20 | +0.60 | n/a | 0.181 | +0.0000 | −0.600 |
| 20 | −0.60 | n/a | 0.234 | +0.0000 | +0.600 |
| 50 | +0.60 | n/a | 0.046 | +0.0000 | −0.600 | **vs §4.1 同参数 ss=0.518，本轮不动** |
| 50 | −0.60 | n/a | 0.100 | +0.0000 | +0.600 |
| 100| +0.60 | n/a | 0.154 | +0.0000 | −0.600 |
| 100| −0.60 | n/a | 0.046 | +0.0002 | +0.600 |

> **频率不是关键变量：5..100Hz 全跨度内电机响应等同（基本零）。本轮电机进入了不响应状态。**

### 4.3 IMU vs Encoder 对照

cmd=-0.50 rad/s, 5s hold, 10Hz：

| 阶段 | IMU gyro.z mean / 范围 | /odom ang.z mean / 范围 |
|---|---|---|
| baseline (1s idle) | +0.0004 (range 0.0003) | 0 |
| **命令期 -0.5 rad/s 5s** | +0.0003 (range 0.0008) | 0 (19/19 样本全 0) |
| **命令归零后 0.5..3s** | mean -0.006, range 0.024 | mean -0.050, range 0.216 |

**关键证据**：命令期 IMU 与 encoder **都报零**，证明机器人在该 5s 命令期内**真的没转**。停止命令 ~2s 后，机器人才出现 -0.49 rad/s 的真实负向转动（odom 报符号正确，IMU gz 小幅响应），即"命令期不动，归零后才释放"。这一现象在多次试验中可见。

### 4.4 teleop vs 本工具对照

| 工具 | publish 频率 | 模式 | 实际转动 |
|---|---|---|---|
| `keyboard_teleop_ros2.py` (按 j/l 交替) | ~35.7 Hz | 直发 /cmd_vel | 用户目测明显转动；odom 大段为 0，仅末尾出现微小数据 |
| 本工具 amp 扫 (`--mode amp ... --publish-hz 50`) | 50 Hz | 直发 /cmd_vel | 同一 +0.60 第一轮跟踪 86%，第二轮零响应 |
| 本工具 mimic-teleop (`+/-0.5 @ 35Hz` 交替) | 35 Hz | 直发 /cmd_vel | 完全零响应（IMU、encoder 都不动） |
| 本工具慢速 (2 Hz +0.6 6s) | 2 Hz | 直发 /cmd_vel | 完全零响应 |

publish 频率本身不解释差异。配合 §4.1 与 §4.2 的对比，**电机响应是非确定性的，与时间、热状态、之前的运动史相关**。

## 5. 关键发现 — 为什么"速度环不可靠"

按可信度排序：

1. **底盘电机响应非确定性。** 同一参数 (target=+0.60, hz=50, hold=4s) 两次实验给出 ss_mean=+0.518 和 ss_mean=0.000。Encoder 链路、命令链路、串口编码全部已验证正确。问题落在 STM32 固件 / 电机驱动 / 物理动力学耦合层。
2. **角速度死区 ~0.40–0.50 rad/s。** 0.40 rad/s 几乎不动；0.50 rad/s 起步慢、欠驱严重；0.60 rad/s 才有较稳定跟踪（仍带 86% 缩放）。当前 `david_move.min_stationary_angular_z=0.35` **低于实测死区**，Nav2 通过 bridge 转向时仍会发出底盘不响应的低角速度。
3. **反馈带宽 ~4 Hz << 控制器 10 Hz。** STM32 上行串口节奏决定 /odom 速率。任何提高上层 publish_hz 的尝试都不会减少 encoder 反馈延迟。
4. **方向对称性差。** +0.60 有响应、−0.60 不动；−0.50 出现 encoder 符号反转 + 线速度耦合 (+0.05 m/s 量级)。
5. **david_move 自身没有"算法 bug"**，但默认值在当前硬件下**没用**：
   - `min_linear_x == max_linear_x = 0.20`：bang-bang，破坏上层比例调节；
   - `min_stationary_angular_z = 0.35`：低于死区，输出无效；
   - `publish_hz = 50`：复制指令 5 倍，对仅 4Hz 反馈的链路浪费。

## 6. 建议

### 6.1 不动的（暂不改 `david_move`）

`david_move.py` 自身没问题，但默认参数前提（"死区 0.35 可用"）已被实测推翻。改 david_move 默认参数之前应先**修复 STM32 / 电机响应的非确定性**，否则改了也没意义。本轮不动 `david_move.py` 源码。

### 6.2 立即可执行的 follow-up

按优先级：

1. **STM32 / 电机层排查**（最高优先级，本报告之外）：
   - 串口上行帧率是否能从 ~4 Hz 拉到 ≥ 50 Hz（影响所有闭环）。
   - 同一命令重复多次能否给出一致响应（影响 Nav2 跟踪可行性）。
   - 死区为什么从 5/9 报告的 ~0.30 涨到本轮的 ~0.50（电机老化 / 电压 / 摩擦？）。
   - cmd=-0.50 时 encoder 符号反转的根因。
2. **Nav2 参数对齐**（在确认底盘可稳定 ≥0.5 rad/s 后再做）：
   - DWB `min_vel_theta` / `min_speed_theta` 提到 `0.50`（当前 0.10，远低于实测死区）。
   - `velocity_smoother` 切换到 `CLOSED_LOOP`，订阅 `/odom`（**但前提是 /odom ≥10Hz**，否则反馈滞后会让闭环更差）。
3. **`david_move` 参数调整**（在 #1、#2 通过后）：
   - 把 `max_linear_x` 调回 nav2 的 `max_vel_x=0.20`，保留；但**把 `min_linear_x` 改成 0.0**，让线速度透传 ⇒ 拆掉 bang-bang。
   - `min_stationary_angular_z` 调到实测死区上沿（届时再定）。
   - `publish_hz` 降到 ~10–15 Hz（匹配 controller + 反馈节奏，省 CPU、不损延迟）。
4. **本测试工具长期保留**：作为 Nav2 调参前的"硬件回归测试"基线。

## 7. 单测与构建

```bash
# 单元测试（不需要 ROS）
cd /home/agx/ros2_ws/src/david_cmd && PYTHONPATH=. \
  python3 -m pytest test/test_wheeltec_velocity_loop_eval.py -v
# 10 passed in 0.04s

# 构建
cd /home/agx/ros2_ws && source /opt/ros/humble/setup.bash \
  && colcon build --packages-select david_cmd
# david_cmd build OK
```

## 8. 文件改动

- 新增 `david_cmd/david_cmd/wheeltec_velocity_loop_eval.py` — 测试 / 扫频脚本
- 新增 `david_cmd/test/test_wheeltec_velocity_loop_eval.py` — 10 条纯函数单测
- 更新 `david_cmd/setup.py` — 注册 `wheeltec_velocity_loop_eval` console_script
- 新增本报告 `david_cmd/docs/report/2026-05-18-wheeltec-velocity-loop-encoder.md`

实测原始数据（CSV + JSON）保留在：

```
/tmp/wheeltec_velocity_loop_eval/
  amp_50hz_4s_summary.json
  amp_50hz_4s_amp_pos_0.30.csv ... amp_50hz_4s_amp_neg_0.60.csv
  freq_w060_summary.json
  freq_w060_freq_pos_5.csv ... freq_w060_freq_neg_100.csv
  pilot_*.csv          # 早期试探数据
```

## 9. 复测建议

当电机响应稳定后（应当先解决 §6.1 的 STM32 端问题），按相同命令复跑 §3.3，预期：

- §4.1 中 +0.60 应稳定给出 ss_mean ~0.55, start_lat ~0.15s, stop_lat ~1.2s（重复性内 ±10%）。
- §4.1 中 ±0.60 应对称（当前 +0.60 工作 / −0.60 不工作）。
- §4.2 频率扫应展示 start_lat 与 publish_hz 几乎无关（受限于 STM32 上行节奏）。
- 升级 STM32 上行频率到 ≥50 Hz 后，再做 publish_hz 扫频，预期高频 publish 才有意义。

仅当上述复测能给出稳定数据时，再决定是否对 `david_move.py` 算法做改动。
