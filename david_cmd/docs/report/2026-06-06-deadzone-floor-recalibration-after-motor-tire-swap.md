# 2026-06-06 换电机+轮胎后的死区 floor 重标定协议

> **状态**：测试协议（待执行）。本文不臆测新 floor 数值，只给出在**轮速计可信**
> 前提下测量它的方法、判据与结果表。测完按 §7 更新参数，再做 §8 的 separator 决策。

## 0. 背景

底盘换了**电机 + 轮胎**。david_move 里这两个静摩擦起步地板是给**旧硬件**标定的，
现在全部失效：

| 参数 | 旧值 | 物理含义 | 文件 |
|---|---:|---|---|
| `min_linear_x` | 0.20 m/s | 线速度起步地板：命令低于此值底盘不动（克服静摩擦）| [david_move.py:16](../../david_cmd/david_move.py) |
| `min_stationary_angular_z` | 0.35 rad/s | 原地转向起步地板 | [david_move.py:25](../../david_cmd/david_move.py) |
| `min_angular_z` | 0.0 rad/s | 弧形运动角速度地板（2026-05-18 修复后置 0）| [david_move.py:23](../../david_cmd/david_move.py) |
| `stationary_angular_threshold` | 0.025 rad/s | 判定「原地 vs 弧形」的角速度阈值 | — |

换硬件对 floor 的两个方向影响，**必须实测、不能算**：

- **电机扭矩变化** → 击穿静摩擦所需的最小命令变了（扭矩更大 → floor 通常**下降**）。
- **轮胎变化** → ① 胎面/接触面/橡胶硬度改变静摩擦系数（floor 变）；② **轮径变化
  直接改变 encoder→速度 的标度**，于是 `/odom` 报的 m/s、rad/s 本身可能不准。

第 ② 点就是用户强调「**在轮速计正常的条件下**测」的原因：odom 标度若没先校正，
所有以 m/s 表达的 floor 数值都是错的。所以 §3 是 §4 的硬前置。

## 1. 目标

1. 先确认 `/odom`（wheeltec 编码器推算的 twist）对新轮胎是准的（§3）。
2. 在此前提下测出**新的**线速度起步 floor、原地转向起步 floor、弧形角速度 floor（§4）。
3. 给出**旧 floor vs 新 floor 的 delta**（用户直接要的「死区 floor 变了多少」）（§6）。
4. 把测得值写回 david_move（§7），然后才做拖尾 vs 起步地板的归因测试（§8）。

## 2. 测量工具（david_cmd 现成脚本，无需新写）

| 脚本 | 作用 | 关键点 |
|---|---|---|
| `wheeltec_micro_eval` | **主力**：扫 `linear_levels`/`angular_levels`，每档发 0.30s 脉冲，**同时用 odom 和 IMU 判定是否动了** | 直发 `/cmd_vel`（绕过 david_move 的 lift）；`hard_distance_limit=0.10m` 自动急停；odom 与 IMU 两路独立判定 → 顺带校验 odom 可信度 |
| `wheeltec_imu_cmd_probe` | **odom 不可信时的后备**：纯 IMU 判定是否动 | odom 还没校准时用它独立确认「底盘到底动没动」 |
| `wheeltec_base_eval` | **闭环复测**：命令走 0.25m / 转 60°，比对 odom 实际位移、反向漂移、起停延迟 | 用于 §3 odom 标度校验 + §7 改完参数后的回归 |
| `wheeltec_velocity_loop_eval` | 阶跃+扫频，测起停延迟与稳态 | 表征响应滞后（与 bang-bang 那条线相关，本协议非必需）|

入口（已在 [setup.py](../../setup.py) 注册）：

```bash
ros2 run david_cmd wheeltec_micro_eval     --ros-args -p <key>:=<val> ...
ros2 run david_cmd wheeltec_imu_cmd_probe  --ros-args -p <key>:=<val> ...
ros2 run david_cmd wheeltec_base_eval      --ros-args -p <key>:=<val> ...
```

`micro_eval` 关键默认值（[wheeltec_micro_eval.py:111-138](../../david_cmd/wheeltec_micro_eval.py)）：
`forwarded_cmd_vel_topic=/cmd_vel`、`odom_topic=/odom`、`imu_topic=/imu/data_raw`、
`probe_duration=0.30`、`settle_duration=0.65`、`hard_distance_limit=0.10`、
`soft_distance_limit=0.06`、`odom_vx_threshold=0.015`、`odom_wz_threshold=0.025`、
`move_distance_threshold=0.003`、`imu_gyro_delta_threshold=0.025`。

## 3. 前置条件：验证 `/odom` 对新轮胎是准的

> 没过这一关，§4 的数字不要采信。

**只起底盘，不起 nav2 / 不起 david_move**（避免任何 lift 干扰；直发 /cmd_vel）：

```bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

### 3.1 线速度标度校验（轮径对不对）

```bash
ros2 run david_cmd wheeltec_base_eval --ros-args \
  -p linear_speed:=0.10 -p target_distance:=0.25 -p cmd_vel_topic:=/cmd_vel
```

- 在地面**贴卷尺**，量机器人实际走的物理距离。
- 判据：odom 报告位移 ≈ 卷尺实测 ≈ 0.25m（误差 < 5%）。
- **不符** → wheeltec 固件/驱动的轮径（wheel diameter）参数没跟着换胎更新 →
  先去 `turn_on_wheeltec_robot` 改轮径，再回到这里。

### 3.2 角速度标度校验（轮距/陀螺一致性）

`micro_eval` 同时录 odom 和 IMU。原地转一档，比对 **odom 的 wz vs IMU 陀螺 gz**：

```bash
ros2 run david_cmd wheeltec_micro_eval --ros-args \
  -p angular_levels:="[0.40]" -p linear_levels:="[]" \
  -p forwarded_cmd_vel_topic:=/cmd_vel
```

- 判据：`max_odom_wz` ≈ IMU 实测角速度峰值（两者比值 ≈ 1）。
- **odom_wz 系统性高/低于 IMU** → 轮距（track width）标度错 → 同样先修固件。

> 通过 3.1 + 3.2 才算「轮速计正常」。记录通过时的 odom/IMU 比值，附在结果里。

## 4. 测量新的死区 floor

> 仍然**只起底盘**，直发 `/cmd_vel`。每档脉冲后看 odom 与 IMU 是否判定「moved」。
> **floor = 能可靠（≥4/5 次）击穿静摩擦、让底盘从静止真正动起来的最低命令值。**

每个 level 重复 ≥5 次（连续多次跑同一 `*_levels` 或重跑脚本），统计成功率。

### 4.A 原地转向起步 floor（替代旧 0.35）—— 最安全，先做

原地转不平移，最安全。从远低于旧值开始往上扫：

```bash
ros2 run david_cmd wheeltec_micro_eval --ros-args \
  -p linear_levels:="[]" \
  -p angular_levels:="[0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35]" \
  -p max_angular_command:=0.40 \
  -p forwarded_cmd_vel_topic:=/cmd_vel
```

- 读每档 `moved`（odom）与 IMU 陀螺是否抬头。
- 新 floor = 最低那个 ≥4/5 次 `moved=True` 的 angular level。

### 4.B 线速度起步 floor（替代旧 0.20）—— 需开阔地 + E-stop

`hard_distance_limit=0.10m` 会自动急停，但仍在**前方 ≥1.5m 无障碍**处做，人手握急停：

```bash
ros2 run david_cmd wheeltec_micro_eval --ros-args \
  -p angular_levels:="[]" \
  -p linear_levels:="[0.03, 0.05, 0.08, 0.11, 0.14, 0.17, 0.20]" \
  -p max_linear_command:=0.22 \
  -p hard_distance_limit:=0.10 \
  -p forwarded_cmd_vel_topic:=/cmd_vel
```

- 新 floor = 最低那个 ≥4/5 次 `moved=True` 的 linear level。

### 4.C 弧形角速度 floor（确认 `min_angular_z` 是否仍可保持 0）

带线速度时给小角速度，看新底盘是否需要弧形地板（旧结论是不需要，置 0）：

```bash
ros2 run david_cmd wheeltec_micro_eval --ros-args \
  -p linear_levels:="[0.15]" \
  -p angular_levels:="[0.03, 0.05, 0.08]" \
  -p forwarded_cmd_vel_topic:=/cmd_vel
```

- 若带线速度时这些小角速度都能体现在 odom/IMU 上 → 维持 `min_angular_z=0.0`。
- 若被静摩擦吃掉 → 记录需要的弧形地板值（但注意 2026-05-18 报告：弧形地板会放大
  DWB 修正导致左漂，非必要不要重新开启）。

## 5. 数据与判读

- `micro_eval` 每档输出一条 `ProbeResult`：`command / displacement / max_odom_vx /
  max_odom_wz / max_imu_*_delta / *_latency / moved`。把 stdout 存盘：
  `... 2>&1 | tee /tmp/deadzone_<axis>_<date>.log`。
- 「moved」的双判据：odom（`max_odom_vx≥0.015` 或 `max_odom_wz≥0.025` 或位移
  `≥0.003m`）**且** IMU（陀螺/加速度 delta 超阈）。两路一致才算可靠击穿。
- **odom 说没动、IMU 说动了** → odom 在低速段不可信（§3 没过关），回 §3。

## 6. 结果表（待填）

### 6.1 odom 健康度（§3）

| 项 | 命令 | 物理实测 | odom 报告 | 比值/误差 | 通过? |
|---|---:|---:|---:|---:|:--:|
| 线速度标度 | 0.10 m/s × 0.25m |  |  |  |  |
| 角速度标度 | 0.40 rad/s 原地 | （IMU 峰值） | （odom wz 峰值） |  |  |

### 6.2 新死区 floor 扫描

| 轴 | level 序列 | 各档成功率 | **新 floor** | 旧 floor | **Δ** |
|---|---|---|---:|---:|---:|
| 原地转向 (4.A) | 0.05…0.35 |  |  | 0.35 |  |
| 线速度 (4.B) | 0.03…0.20 |  |  | 0.20 |  |
| 弧形角速度 (4.C) | 0.03…0.08 |  |  | 0.0 |  |

> 预期方向：新电机扭矩更大 → 原地/线速度 floor **下降**；具体多少以实测为准。

## 7. 测完更新参数（david_move）

把测得 floor + 小裕量（建议 floor × 1.1～1.2，覆盖电压/温度波动）写回：

- `min_linear_x` ← 4.B 结果
- `min_stationary_angular_z` ← 4.A 结果
- `min_angular_z` ← 维持 0.0（除非 4.C 推翻）

改的是 [launch 里 david_move 的 parameters](../../../david_nav/launch/2d_david_nav2_bringup.launch.py)
（当前分支 `feature/disable-cmd-vel-separator` 显式传了 `min_stationary_angular_z:=0.35`）
或 [david_move.py 的 DEFAULT_*](../../david_cmd/david_move.py)。改完用 `wheeltec_base_eval`
闭环回归（命令 0.25m/60°，确认无反向漂移、起步无 lurch）。

## 8. 然后才做 separator 归因（接上次分析）

floor 更新后，再跑上次说的拖尾 vs 起步地板判别（见
[2026-05-18 左漂报告](2026-05-18-nav2-left-drift-david-move-deadzone.md)）：

1. `min_stationary_angular_z` 设为新测值后重发正前方 goal：
   - 硬左甩消失 → 是起步地板（软件，已被新 floor 修正）。
   - 仍在 → 指向**万向轮拖尾**（物理）。
2. 若是拖尾 → 结论是 separator **不该整个拿掉**，保留它的「纯转→纯进」时序
   （起步先把 caster 摆正），只砍 bang-bang（满速 binary + 反向刹车）那部分。
3. 用 `record_nav_drift.py`（订阅 `/cmd_vel`+`/odom`，本分支可用）确认偏从链路哪段起。

## 9. 安全

- 70kg 重底盘，**4.A 原地转先做**（不平移）；4.B 线速度务必开阔地 + 手握急停。
- `hard_distance_limit=0.10` / `soft_distance_limit=0.06` 是软急停，**不能替代物理急停**。
- 全程 nav2 / david_move **不要起**，避免 lift 污染原始 floor 测量。
- 所有进程同一 `ROS_DOMAIN_ID`（参考 workspace CLAUDE.md §4，通常 0）。

## 10. 命令速查

```bash
# 起底盘（仅此，不起 nav2）
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# §3.1 线速度标度
ros2 run david_cmd wheeltec_base_eval --ros-args -p linear_speed:=0.10 -p target_distance:=0.25 -p cmd_vel_topic:=/cmd_vel
# §3.2 角速度标度（odom vs IMU）
ros2 run david_cmd wheeltec_micro_eval --ros-args -p angular_levels:="[0.40]" -p linear_levels:="[]" -p forwarded_cmd_vel_topic:=/cmd_vel
# §4.A 原地转向 floor
ros2 run david_cmd wheeltec_micro_eval --ros-args -p linear_levels:="[]" -p angular_levels:="[0.05,0.10,0.15,0.20,0.25,0.30,0.35]" -p max_angular_command:=0.40 -p forwarded_cmd_vel_topic:=/cmd_vel 2>&1 | tee /tmp/deadzone_ang_20260606.log
# §4.B 线速度 floor
ros2 run david_cmd wheeltec_micro_eval --ros-args -p angular_levels:="[]" -p linear_levels:="[0.03,0.05,0.08,0.11,0.14,0.17,0.20]" -p max_linear_command:=0.22 -p forwarded_cmd_vel_topic:=/cmd_vel 2>&1 | tee /tmp/deadzone_lin_20260606.log
# §4.C 弧形角速度 floor
ros2 run david_cmd wheeltec_micro_eval --ros-args -p linear_levels:="[0.15]" -p angular_levels:="[0.03,0.05,0.08]" -p forwarded_cmd_vel_topic:=/cmd_vel
```

---

**附：本协议未执行任何硬件动作，纯文档。** 数值列待你按上面跑完回填；跑的时候有
任何一档 odom 与 IMU 判定打架，先回 §3，别往下测。
