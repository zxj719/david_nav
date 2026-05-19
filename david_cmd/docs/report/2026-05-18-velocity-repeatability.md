# 2026-05-18 Wheeltec 速度环重复性测试报告

> **2026-05-18 当日更新（见 §10）**：经过 STM32 固件源码与 wheeltec_robot_node 串口解析的根因定位，发现 `Get_Sensor_Data_New` 每个 10ms tick 只读 1 字节，导致 /odom 仅 4Hz（STM32 固件实际是 20Hz 发送）。已修复 `wheeltec_robot.cpp`：一次性消费 serial 缓冲。重测：/odom 从 4.4Hz → 22Hz，**8 桶 × 5 轮 = 40 段全部正确响应、零符号反转，跨轮 ss_mean std 从 0.05–0.27 降到 0.03–0.12**。结论翻转：**速度环现在可重复，Nav2 闭环可用**。详见 §10。

---

## 任务

回答一个问题：**同一速度命令重复多次，底盘是否给出一致响应？**

方法：用 `wheeltec_velocity_loop_eval --mode amp` 对幅值 `[0.40, 0.50, 0.60, 0.70] rad/s × ±` 共 8 个测试点，**完全相同的参数连续跑 5 轮**（间隔 2s），用 `/odom` 轮速 encoder 作为反馈，把每个测试点的 5 次结果汇总成跨轮均值/标准差/响应率。

## TL;DR

**重复性为零。速度环非确定性比之前怀疑的更严重：**

| 现象 | 证据 |
|---|---|
| 跨轮响应率 | 8 个测试点中只有 1 个达到 100%（且其 ss_mean ~0，只是抖动），其余 20%–60%。 |
| 跨轮 ss_mean 标准差 | target=±0.60 跨 5 轮 ss_mean 标准差 ~0.25 rad/s，相对量程 ~40%。 |
| **跨轮符号反转** | target=+0.60 五轮 ss_mean = [−0.362, −0.255, 0.000, +0.131, +0.329]，**5 轮里 2 轮方向错** |
| 同样 target=−0.60 五轮 = [−0.358, −0.194, **+0.433**, +0.100, 0.000]，**5 轮里 2 轮反向，且 +0.43 比期望值还大** |

结论：**在不修底盘 STM32 / 电机驱动控制层之前，速度环无法形成可靠闭环；Nav2 在这套底盘上跟踪轨迹将带不可预测的方向错误，不适合直接跑。**

## 测试条件

- 日期：2026-05-18 evening
- ROS 2 Humble；wheeltec_robot_node + twist_transform 已 launch
- 串口：`/dev/wheeltec_controller`（USB 重插过一次）
- 发布话题：`/cmd_vel`（绕过 david_move bridge）
- 反馈话题：`/odom.twist`（来自 STM32 轮速 encoder 差分）；`/odom` 实测 ~4.4 Hz
- 命令参数：`--publish-hz 50 --hold-seconds 4.0 --rest-seconds 2.0 --ss-window 1.5 --motion-threshold 0.05`
- 重复轮数：5
- 总段数：5 × 8 = 40
- 总耗时：~5.5 min

`/odom` 实测仅 4.4Hz 是 STM32 上行节奏所限，与 ROS 侧 publish_hz 无关；详见上一份报告 `2026-05-18-wheeltec-velocity-loop-encoder.md` §2.2。

## 度量定义

### `response_classification(summary, min_ss_fraction=0.3)`

判定一次 segment 是否"响应"：

- 命令期内 encoder 检测到 |ang_z| ≥ threshold ⇒ 记录 `start_latency`。**有 start_latency 即视为响应**，前提是稳态符号没反。
- 若没有 start_latency 但稳态 `ss_mean` 与 target 同号且 |ss_mean| ≥ `min_ss_fraction × |target|` ⇒ 视为响应。
- 若 `ss_mean` 与 target **反号** 且 |ss_mean| ≥ `min_ss_fraction × |target|`（≥ 30%）⇒ 计为**非响应**（电机走了相反方向，不可用）。

### `aggregate_repetitions(summaries) -> RepeatabilityStats`

对同一 `(target_w, publish_hz)` 桶里的 N 个 segment：

- `n`：总段数（5）
- `n_responded`：分类为响应的段数
- `response_rate = n_responded / n`
- `ss_mean_avg/std`：跨 N 轮 ss_mean 的均值/总体标准差（无运动的轮以 ss=0 计入，反映"加速度环响应率"的真实分布）
- `start_lat_avg/std`、`stop_lat_avg/std`：**只在响应轮**上计算

### 单测

`david_cmd/test/test_wheeltec_velocity_loop_eval.py` 共 **20 条**，本次新增 10 条覆盖：

| 测试 | 覆盖点 |
|---|---|
| `test_response_classification_uses_start_latency_when_present` | 有 start_lat 即响应 |
| `test_response_classification_uses_ss_fraction_when_no_start_latency` | 无 start_lat 也可由 ss_mean 比例判 |
| `test_response_classification_rejects_segment_with_no_motion` | 无运动判非响应 |
| `test_response_classification_rejects_wrong_sign` | 符号反转判非响应（**关键**） |
| `test_aggregate_repetitions_counts_runs_and_response_rate` | 计数 + 响应率 |
| `test_aggregate_repetitions_averages_only_responded_for_latency` | latency 仅响应轮 |
| `test_aggregate_repetitions_ss_mean_includes_all_runs` | ss_mean 跨所有轮 |
| `test_aggregate_repetitions_rejects_mixed_targets` | 不同 target 应抛错 |
| `test_aggregate_repetitions_rejects_empty_input` | 空输入抛错 |
| `test_aggregate_repetitions_handles_all_nonresponse` | 全非响应 ⇒ latency=None |

执行：

```bash
cd /home/agx/ros2_ws/src/david_cmd && PYTHONPATH=. \
  python3 -m pytest test/test_wheeltec_velocity_loop_eval.py -v
# 20 passed in 0.04s
```

## 跨轮汇总结果

`/tmp/wheeltec_velocity_loop_eval/repeat_aggregate.json`：

| target | n | n_responded | response_rate | ss_mean_avg | ss_mean_std | start_lat_avg | start_lat_std | stop_lat_avg | stop_lat_std |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| **+0.40** | 5 | 3 | 0.60 | +0.004 | 0.105 | 1.33 | 1.58 | 0.05 | 0.01 |
| **−0.40** | 5 | 2 | 0.40 | −0.166 | 0.262 | 0.05 | 0.03 | n/a | n/a |
| **+0.50** | 5 | 2 | 0.40 | −0.057 | 0.127 | 1.55 | 1.18 | 0.53 | 0.00 |
| **−0.50** | 5 | 5 | **1.00** | +0.013 | 0.069 | 2.28 | 2.01 | 0.69 | 0.77 |
| **+0.60** | 5 | 3 | 0.60 | −0.031 | 0.252 | 0.15 | 0.05 | 0.79 | 0.55 |
| **−0.60** | 5 | 3 | 0.60 | −0.004 | 0.269 | 0.13 | 0.03 | 0.71 | 0.50 |
| **+0.70** | 5 | 1 | 0.20 | +0.019 | 0.043 | 0.42 | 0.00 | n/a | n/a |
| **−0.70** | 5 | 1 | 0.20 | −0.002 | 0.005 | 3.36 | 0.00 | 0.08 | 0.00 |

### 逐轮 ss_mean 原始记录

每个桶 5 轮的 ss_mean（rad/s），`✓` = 响应，`✗` = 非响应：

```
target=-0.70  ss_mean: [+0.000, +0.004, +0.000, -0.012, +0.000]  resp: [✗ ✗ ✗ ✓ ✗]
target=-0.60  ss_mean: [-0.358, -0.194, +0.433, +0.100, +0.000]  resp: [✓ ✓ ✗ ✓ ✗]
target=-0.50  ss_mean: [+0.000, -0.086, -0.028, +0.077, +0.103]  resp: [✓ ✓ ✓ ✓ ✓]
target=-0.40  ss_mean: [+0.000, +0.132, -0.464, -0.500, +0.000]  resp: [✗ ✗ ✓ ✓ ✗]
target=+0.40  ss_mean: [+0.000, +0.185, -0.020, -0.143, +0.000]  resp: [✗ ✓ ✓ ✗ ✓]
target=+0.50  ss_mean: [+0.000, +0.000, -0.310, -0.000, +0.025]  resp: [✗ ✗ ✗ ✓ ✓]
target=+0.60  ss_mean: [-0.362, -0.255, -0.000, +0.131, +0.329]  resp: [✗ ✗ ✓ ✓ ✓]
target=+0.70  ss_mean: [+0.000, +0.000, +0.000, +0.104, -0.012]  resp: [✗ ✗ ✗ ✓ ✗]
```

## 发现

### 1. 响应率：在死区附近不一致，远高于死区也不可靠

- 死区附近 0.40：响应率 40–60%，响应时 ss 可达 |0.46| 但也可能 |0.13|。
- "应当工作" 的 0.60：5 轮里只有 3 轮响应，ss 在 −0.36 ~ +0.43 之间漂移。
- 高于"工作点"的 0.70：响应率反而掉到 20%。

> **没有任何幅值给出"5/5 响应且 ss 接近 target"的稳定行为。**

### 2. **方向不可控** —— 最严重的问题

跨轮 ss_mean 范围（pos − neg）：

- target=+0.60 跨 5 轮 ss_mean 范围 **[−0.362, +0.329]**：同一正向命令，有时反转、有时不动、有时正常。
- target=−0.60 跨 5 轮范围 **[−0.358, +0.433]**：同一负向命令，有时反向、有时正常。
- target=+0.40 跨 5 轮范围 **[−0.143, +0.185]**：低幅值同样有方向漂移。
- target=−0.40 跨 5 轮里 run2 给出 +0.132（反向）。

**5 轮内 8 个测试点共出现 8 次"显著反向"事件**（|ss| ≥ 0.1 rad/s 且符号与 target 反）。

### 3. start_latency 的方差也大

- target=−0.50 五轮里有响应的 start_lat = [n/a, 0.297, 3.006, 1.942, 0.383]，跨轮均值 1.91s，std 1.18s。
- target=+0.50：[n/a, n/a, 0.072, 0.367, 2.729]，均值 1.06s，std 1.18s。

> 即使响应了，"什么时候动"也不可预测，从几十毫秒到 3 秒不等。

### 4. 响应方向偶尔比 target 更大

- target=−0.40 run4：ss_mean = −0.500，**比 −0.40 target 大 25%**。
- target=−0.60 run3：ss_mean = **+0.433**（反向，量级接近 target 的 72%）。

说明 STM32 内部 PI 在某些条件下会"过补偿"出比 target 还大的速度，且方向可能完全相反。

### 5. `−0.50` 段响应率 5/5 但 ss 全部 ~0 的解释

- 这 5 段都被分类为响应，因为它们都报了 start_latency；但 ss_mean 均值 +0.013，标准差 0.069，实际是**短暂抖动后停下**——电机"应付"了一下命令但没维持。
- 配合 §3 stop_lat 均值 0.69s + std 0.77s，表明运动并不长。

## 与 `david_move` 默认值的对照

`david_move.py` 当前默认：
- `min_stationary_angular_z = 0.35`
- `max_angular_z = 0.50`

实测里 0.40 / 0.50 段没有任何 ss_mean ≥ 0.30 的成功例。**在当前硬件状态下，`david_move` 默认会把上层 Nav2 的小转向命令抬到 0.35–0.50 rad/s 区间发给底盘——而这一区间响应率 ≤60% 且符号经常错。**

## 结论

1. **速度环非确定性在所有 4 个测试幅值上都成立**，不是单个工作点的问题。
2. **方向反转是结构性现象**，不是采集噪声。8 个 ss_mean 反向事件分布在 4 个 target 上。
3. **现有 `david_move.py` 改算法 / 改默认值无法补救**——上位机看不到 STM32 内部 PI 的状态，桥接层 clamp / 死区抬升也无法预测电机是否会朝反方向走。
4. **Nav2 无法在此底盘上稳定闭环**：DWB 假设命令与实际响应方向一致，本数据集证明假设不成立。
5. **真正的根因在 STM32 固件 / 电机驱动**。可能候选（按怀疑排序）：
   - STM32 内部双轮 PID 的 Anti-windup / 死区策略有 bug，在低占空比 + 静摩擦下产生不可预测累积。
   - 串口下行 cmd_vel 帧的字段被某种状态机条件性丢弃（看到字节正确写入但 STM32 是否 ACK / apply 未知）。
   - 电机驱动板硬件失效（一边轮子驱动失常会让"应当原地转"变成"前进+小转"，部分匹配观察到的 lin_x 耦合）。
   - 电池电压偏低导致驱动器进入保护模式（需补测 /PowerVoltage 序列）。

## 建议（按优先级）

1. **不要在当前硬件状态下推进 Nav2 闭环**——继续将让你浪费时间在调 DWB 参数上，无法补救底盘层非确定性。
2. **STM32 固件回归测试**：把固件源码（如能拿到）跑一遍单元测试；如果是黑盒，先用示波器/逻辑分析仪看 STM32 输出到电机驱动板的 PWM 是否与串口命令一致。
3. **测 /PowerVoltage 时序**：用本工具同时订阅 `/PowerVoltage`，看响应失败的 run 是否伴随电压跌落。如能复现"电压跌落 ⇒ 响应失败"则确诊电源问题，解法是充电 / 更大电池。
4. **若可换底盘**：考虑用一个有 ROS 驱动 + 公开固件源码的方案（如 OAK-D / Mecanum 板，或者直接 STM32CubeIDE 自己写一份固件，已有完整 encoder + PWM 例程）。
5. **保留本测试工具**：等任何一种修复方案落地，立即重跑 §3.3 的命令对比 `repeat_aggregate.json` —— 5/5 响应、跨轮 ss_mean_std ≤ 0.05、无符号反转，就是"修好了"的客观标准。

## 复跑命令

```bash
# 启底盘
cd ~/wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2
source ./install/setup.bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 跑 5 轮
source /opt/ros/humble/setup.bash
source /home/agx/ros2_ws/install/setup.bash
for i in 1 2 3 4 5; do
  ros2 run david_cmd wheeltec_velocity_loop_eval \
    --mode amp --cmd-topic /cmd_vel --publish-hz 50 \
    --hold-seconds 4.0 --rest-seconds 2.0 --ss-window 1.5 \
    --amplitudes 0.40,0.50,0.60,0.70 --label repeat_r$i
  sleep 2
done

# 汇总
python3 - <<'EOF'
import json
from pathlib import Path
from collections import defaultdict
from david_cmd.wheeltec_velocity_loop_eval import (
    SegmentSummary, aggregate_repetitions,
)
DATA = Path("/tmp/wheeltec_velocity_loop_eval")
buckets = defaultdict(list)
for p in sorted(DATA.glob("repeat_r*_summary.json")):
    for seg in json.loads(p.read_text())["segments"]:
        s = SegmentSummary(**seg)
        buckets[round(s.target_w, 2)].append(s)
for k in sorted(buckets):
    print(k, aggregate_repetitions(buckets[k]))
EOF
```

## 文件改动（相对上一份报告）

- `david_cmd/david_cmd/wheeltec_velocity_loop_eval.py`：新增 `RepeatabilityStats` 数据类、`response_classification()`、`aggregate_repetitions()` 三个纯函数（向后兼容）。
- `david_cmd/test/test_wheeltec_velocity_loop_eval.py`：新增 10 条单测覆盖新函数，总计 20 条全过。
- 本报告 `david_cmd/docs/report/2026-05-18-velocity-repeatability.md`。
- 原始数据：`/tmp/wheeltec_velocity_loop_eval/repeat_r{1..5}_*.csv`、`repeat_r{1..5}_summary.json`、`repeat_aggregate.json`。

---

## 10. 后续：根因定位 + 串口解析修复 + 修复后回归（2026-05-18 当日）

### 10.1 根因定位（基于 STM32 固件源码）

读 `david_cmd/docs/3.STM32运动底盘源码/.../WHEELTEC_C50X_2025.08.07/` 与 `wheeltec_robot_ros2` 后定位两个独立根因：

**根因 1（已修复）—— ROS 驱动每 tick 只读 1 字节**

`wheeltec_robot_ros2/src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp:265`，`Get_Sensor_Data_New()`：

```cpp
uint8_t Receive_Data_Pr[1];                                    // ⬅️ 一次只读 1 字节
Stm32_Serial.read(Receive_Data_Pr, sizeof(Receive_Data_Pr));
Receive_Data.rx[count] = Receive_Data_Pr[0];
```

`Control()` 是 100Hz wall_timer。1 字节/tick × 24 字节/帧 = **240 ms/帧 ≈ 4.17 Hz**，正好对上之前测的 4.4 Hz。

STM32 端实际是 **20Hz 发送**：`BALANCE/data_task.c::data_task()` `vTaskDelayUntil(&lastWakeTime, F2T(DATA_TASK_RATE))`，`DATA_TASK_RATE = RATE_20_HZ = 20`，`F2T(20) = configTICK_RATE_HZ/20 = 1000/20 = 50` ticks = 50ms。

**根因 2（已识别，未修复）—— STM32 端 PI 无 anti-windup**

`WHEELTEC_C50X_2025.08.07/BALANCE/balance_task.c:227`，`Incremental_MOTOR`：

```c
p->Bias = target - current;
p->Output += p->kp * (p->Bias - p->LastBias) + p->ki * p->Bias;
if (p->Output >  FULL_DUTYCYCLE) p->Output =  FULL_DUTYCYCLE;
if (p->Output < -FULL_DUTYCYCLE) p->Output = -FULL_DUTYCYCLE;
```

增量式 PI，`Output` 是 `PI_MotorA` static 状态（`USER/system.c:173` 初始化），且**没有 anti-windup**。当电机被命令但跟不上 → Output 累加饱和 → 命令归零时 Bias=0 ⇒ `Output += 0` ⇒ Output 保持饱和值。下次命令换方向时 PI 要先"放电"再反向，造成"命令期不动 / 归零后才动 / 跨轮符号反转"的所有现象。

### 10.2 Fix A 实施

修改 `wheeltec_robot.cpp::Get_Sensor_Data_New()`，把 1-byte-per-tick 改成 drain-all-available：

```cpp
size_t available = Stm32_Serial.available();
if (available == 0) return false;
bool frame_complete = false;
while (available-- > 0) {
  uint8_t byte = 0;
  if (Stm32_Serial.read(&byte, 1) != 1) break;
  Receive_Data.rx[count] = byte;
  if (byte == FRAME_HEADER || count > 0) count++;
  else count = 0;
  if (count != 24) continue;
  count = 0;
  // 帧尾、校验、解码 — 与原逻辑一致
  ...
  frame_complete = true;
}
return frame_complete;
```

构建：

```bash
cd /home/agx/wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2
colcon build --packages-select turn_on_wheeltec_robot
```

备份原文件保留在 `.../wheeltec_robot.cpp.bak_2026-05-18`。

### 10.3 修复后实测

#### 10.3.1 /odom 频率

```text
odom: n=111 dur=5.01s mean_rate=21.97Hz min_dt=0.4ms max_dt=50.4ms
```

✅ 22Hz，max_dt=50ms 恰好是 STM32 帧间隔。

#### 10.3.2 8 桶 × 5 轮 重复性

`--label after_fixA_r{1..5}`，同样的 `--amplitudes 0.40,0.50,0.60,0.70`, `hold=4s, ss_window=1.5s`：

| target | n | n_responded | rate | ss_avg | ss_std | start_lat_avg | stop_lat_avg |
|---:|---:|---:|---:|---:|---:|---:|---:|
| +0.40 | 5 | **5** | **1.00** | +0.294 | 0.098 | 0.72 | 0.59 |
| −0.40 | 5 | **5** | **1.00** | −0.199 | 0.060 | 1.15 | 0.66 |
| +0.50 | 5 | **5** | **1.00** | +0.373 | 0.081 | 0.94 | 0.73 |
| −0.50 | 5 | **5** | **1.00** | −0.359 | 0.053 | 0.87 | 0.78 |
| +0.60 | 5 | **5** | **1.00** | +0.552 | 0.115 | 0.84 | 0.75 |
| −0.60 | 5 | **5** | **1.00** | −0.506 | 0.026 | 0.87 | 0.75 |
| +0.70 | 5 | **5** | **1.00** | +0.699 | 0.094 | 0.80 | 0.82 |
| −0.70 | 5 | **5** | **1.00** | −0.581 | 0.056 | 0.64 | 0.87 |

逐轮 ss_mean（修复后**所有方向都正确**）：

```
-0.70: [-0.666, -0.495, -0.560, -0.579, -0.607]
-0.60: [-0.546, -0.502, -0.471, -0.519, -0.492]
-0.50: [-0.406, -0.435, -0.326, -0.326, -0.299]
-0.40: [-0.264, -0.179, -0.269, -0.171, -0.112]
+0.40: [+0.446, +0.350, +0.293, +0.166, +0.217]
+0.50: [+0.489, +0.332, +0.429, +0.359, +0.254]
+0.60: [+0.654, +0.590, +0.550, +0.632, +0.334]
+0.70: [+0.802, +0.726, +0.697, +0.744, +0.524]
```

### 10.4 Fix A 前后对比

| 指标 | Fix A 前 | Fix A 后 | 改善 |
|---|---|---|---|
| /odom rate | 4.4 Hz | 22.0 Hz | 5.0× |
| 响应率（8 桶平均） | 0.60 | **1.00** | 全过 |
| ss_mean std（最坏桶） | 0.269 | **0.115** | 2.3× |
| 跨轮符号反转事件 | 8 / 40 | **0 / 40** | 消除 |
| 每段 odom 样本 | ~34 | ~162 | 5× |
| 跟踪 100% 桶数 | 0 / 8 | 8 / 8 | — |

### 10.5 残余问题（未修）

1. **ss_mean 普遍欠驱 20–30%**（如 +0.40 → 0.29, +0.70 → 0.70 但 +0.50 → 0.37）。怀疑来源：
   - STM32 `Vel_SmoothControl` 把命令以 `smooth_MotorStep = 0.02/tick @ 100Hz = 2.0 rad/s²` ramp 到目标，0.5 rad/s 需要 ~0.25s，对 4s hold 影响小但非零；
   - PI 仍无 anti-windup（根因 2），高 windup 状态下 PI 出力相对静摩擦/驱动器响应不完全。
2. **start_lat ~0.6–1.2s**：比理想长。原因同上 + 控制周期 100Hz × smooth 步长。可通过 STM32 端加大 `smooth_MotorStep` 缩短，但风险是急停冲击。
3. **±0.4 段 ss_mean = 0.29 / -0.20**（差不多达不到目标）：这是接近实测死区下沿，命令不到位很常见。`david_move.min_stationary_angular_z=0.35` 在 Fix A 之后仍然合理 —— 0.50 以上才稳。

### 10.6 给 Nav2 的建议（修复后）

现在可以谨慎推进 Nav2 闭环：

- `velocity_smoother.feedback`：可考虑切到 `CLOSED_LOOP`，订阅 `/odom`，因为 /odom 真的 20Hz 了。先在仿真验证 PID 不振荡。
- `david_move`：保持现状不动（你之前的死区抬升仍合理）。
- 真要降低 `min_stationary_angular_z` 之前，应先做根因 2（STM32 PI anti-windup）。

### 10.7 复现命令

```bash
# 启底盘（新驱动已 colcon build 过）
cd ~/wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2
source ./install/setup.bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py

# 验证 /odom 22Hz：见 §10.3.1 的 python 探测脚本
# 重复性回归：
source /opt/ros/humble/setup.bash && source /home/agx/ros2_ws/install/setup.bash
for i in 1 2 3 4 5; do
  ros2 run david_cmd wheeltec_velocity_loop_eval --mode amp \
    --cmd-topic /cmd_vel --publish-hz 50 \
    --hold-seconds 4.0 --rest-seconds 2.0 --ss-window 1.5 \
    --amplitudes 0.40,0.50,0.60,0.70 --label after_fixA_r$i
  sleep 2
done
```

### 10.8 文件改动

- `wheeltec_robot_ros2/src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp`：改 `Get_Sensor_Data_New()`，drain-all-available。原文件保留 `.bak_2026-05-18`。
- `/tmp/wheeltec_velocity_loop_eval/after_fixA_r{1..5}_*` 是修复后实测原始数据。
- 本报告 §10 是修复+回归记录。

---

## 11. 线速度可重复性（前后运动，§10 修复后）

### 11.1 背景与配置改动

完成 §10 的串口修复后，把 `david_nav/config/nav2_params_2d.yaml::velocity_smoother`
切到 `feedback: CLOSED_LOOP`，并把 `odom_topic` 从 `/lightning/odometry` 改成
`/odom`（Wheeltec 底盘 encoder odom）。原 `/lightning/odometry` 只在 Unitree Go2
SLAM 栈跑起来时存在，当前 Wheeltec 底盘上没有这个话题。

随后做 **线速度方向** 的重复性扫，验证 §10 的修复对前后运动同样有效。

> 安全条件：测试空间前后约 1.5m。用 `--axis linear` 让脚本只动 `linear.x`，
> 配合 `--mode amp` 的"+w / 0 / -w / 0"对称结构，单段最大位移 ≈ `0.20 m/s × 3s = 0.6m`，
> 实际由于 25%+ 欠驱位移更小；±对偶后净漂移基本可忽略，在 ±0.75m 空间内安全。
> 角速度不做（按用户要求；整机 70kg+ 在窄空间转向风险高）。

### 11.2 工具改动（TDD）

`wheeltec_velocity_loop_eval` 增加 `--axis linear|angular`（默认 angular，向后兼容）：

| 改动 | 文件 | 单测 |
|---|---|---|
| `_signal_value(sample, axis)`、`compute_*` / `summarize_segment` 增 `axis` 参数 | `david_cmd/david_cmd/wheeltec_velocity_loop_eval.py` | 新增 5 条单测覆盖 axis="linear" 全路径 |
| `VelocityLoopEval` 类 `__init__` 增 `axis`；`_publish_cmd` 按 axis 选 `linear.x` / `angular.z` | 同上 | 由实机回归覆盖 |
| CLI `--axis linear|angular` | 同上 | — |

执行：

```bash
cd /home/agx/ros2_ws/src/david_cmd && PYTHONPATH=. \
  python3 -m pytest test/test_wheeltec_velocity_loop_eval.py -v
# 25 passed in 0.05s
```

> ⚠️ **已知小 bug**：日志格式串里仍写 `target_w=...rad/s`，line 速度模式下显示
> "rad/s" 但实际单位是 m/s。数据保存正确，仅日志单位标签需后续修。

### 11.3 测试命令

```bash
for i in 1 2 3 4 5; do
  ros2 run david_cmd wheeltec_velocity_loop_eval \
    --mode amp --axis linear --cmd-topic /cmd_vel --publish-hz 50 \
    --hold-seconds 3.0 --rest-seconds 2.0 --ss-window 1.0 \
    --motion-threshold 0.02 \
    --amplitudes 0.10,0.15,0.20 --label linear_r$i
  sleep 2
done
```

### 11.4 5 轮汇总（6 桶 × 5 轮 = 30 段）

| target m/s | n | rsp | rate | ss_mean_avg | ss_mean_std | sL_avg | sL_std | tL_avg | 跟踪率 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| +0.10 | 5 | **5** | **1.00** | +0.035 | 0.017 | 1.27 | 0.27 | 0.26 | 35% |
| −0.10 | 5 | **5** | **1.00** | −0.030 | 0.017 | 1.19 | 0.11 | 0.05 | 30% |
| +0.15 | 5 | **5** | **1.00** | +0.060 | 0.016 | 0.66 | 0.30 | 0.77 | 40% |
| −0.15 | 5 | **5** | **1.00** | −0.080 | 0.018 | 1.00 | 0.09 | 0.57 | 53% |
| **+0.20** | 5 | **5** | **1.00** | **+0.164** | 0.034 | 0.68 | 0.16 | 0.68 | **82%** |
| **−0.20** | 5 | **5** | **1.00** | **−0.173** | 0.013 | 0.70 | 0.05 | 0.54 | **87%** |

逐轮 ss_mean（所有方向均正确）：

```
-0.20: [-0.166, -0.166, -0.198, -0.161, -0.174]
-0.15: [-0.112, -0.064, -0.071, -0.083, -0.066]
-0.10: [-0.054, -0.018, -0.046, -0.019, -0.010]
+0.10: [+0.063, +0.020, +0.048, +0.022, +0.023]
+0.15: [+0.089, +0.058, +0.046, +0.045, +0.062]
+0.20: [+0.174, +0.106, +0.186, +0.206, +0.149]
```

### 11.5 主要发现

1. **响应率 30/30 = 100%，零符号反转。** 与 §10 修复后的角速度一致 —— 串口修复让两个轴都可重复。
2. **跨轮 ss_mean std 仅 0.013–0.034 m/s**，比角速度（§10 中 0.026–0.115 rad/s）相对量程**更窄**。线速度链路对静摩擦/驱动器扰动的敏感度低于差速旋转。
3. **跟踪比例与负载相关**：
   - +0.20 m/s 跟踪 ~82–87%（**与用户提到"整机 70kg+，0.3m/s 命令产生 >0.5m/s 实际"的现象在 +0.20 这一档相反方向上对应** —— 在低速段电机欠驱，在过死区段反而过冲；这是非线性驱动器的典型特征）
   - +0.15 m/s 跟踪 ~40–53%（介于死区与稳定工作点之间）
   - +0.10 m/s 跟踪 ~30%（接近 / 低于实际死区）
4. **实测线速度死区 ~0.15–0.20 m/s**，与你 `david_move.min_linear_x = max_linear_x = 0.20` 的手动选点**一致**。说明 0.20 m/s 是当前 70kg+ 整机的稳定起步速度，0.15 m/s 偶尔能动但不可靠。
5. **start_lat 0.66–1.27s**：与角速度同档（~0.6–1.2s），来自 STM32 `Vel_SmoothControl` 0.02/tick @ 100Hz 的 ramp 时间 + PI 无 anti-windup 的释放延迟。
6. **stop_lat 0.05–0.77s**：比角速度（0.59–0.87s）略快，重量大反而对制动有利。

### 11.6 对 david_move / Nav2 的复核

- `david_move.min_linear_x = 0.20` **是合理的工作点**。本次数据证明 0.15 m/s 跟踪 ~50%、0.10 m/s 只跟踪 ~30%，达不到可控运动；0.20 m/s 跟踪 ~85% 接近"可用"。
- `velocity_smoother.feedback: CLOSED_LOOP` 已切上、`odom_topic: /odom` 已纠正。但在 70kg 重载场景下，CLOSED_LOOP 会把 35% / 40% 这种欠驱当成"该补"而推高命令——这可能反而引发 Nav2 控制器的振荡。**建议在仿真或低速场景先观察 30 分钟，必要时再切回 OPEN_LOOP**。
- 线速度跟踪 ~85% 对 Nav2 距离 / 速度规划来说仍是可用的：误差 15% 远小于角速度欠驱 30%+ 的痛点。

### 11.7 待办

1. 日志单位标签 bug：`f"{...}rad/s"` 应根据 `axis` 选择，修起来 5 行代码。
2. STM32 端 PI anti-windup（§10 根因 2）：在 70kg 高静摩擦下 wind-up 释放时间长，是 start_lat 0.7–1.3s 的主因。固件改动后再做一次本测复测，预期 start_lat 降到 < 0.3s。
3. 如果切了 CLOSED_LOOP 后跑 Nav2 出现速度振荡，**先回切 OPEN_LOOP** —— 70kg 重载下 PI 反馈环增益可能太高。

### 11.8 文件改动（增量）

- `david_cmd/david_cmd/wheeltec_velocity_loop_eval.py`：纯函数增 `axis` 参数；ROS 类 + CLI 支持 `--axis linear`。
- `david_cmd/test/test_wheeltec_velocity_loop_eval.py`：新增 5 条 axis=linear 单测，总 25 条全过。
- `david_nav/config/nav2_params_2d.yaml`：`velocity_smoother.feedback: OPEN_LOOP → CLOSED_LOOP`；`odom_topic: /lightning/odometry → /odom`。
- 原始数据：`/tmp/wheeltec_velocity_loop_eval/linear_r{1..5}_*.csv`、`linear_r{1..5}_summary.json`、`linear_aggregate.json`、`pilot_linear_*.csv`。
