# 新底盘速度区间 + 位置环 PID 重标定 — 设计

> 2026-06-23 · feature/david_main · 起因：换大电机+大轮子的新底盘，原
> `position_controller.yaml` 的速度区间/死区/PID 是旧硬件标定值，需要用 `/odom`
> 轮速真值重新标定。

## 1. 目标

用 **`/odom` 的 `twist.twist`（轮速编码器速度）作为真值**，分 **直行 / 旋转**
两类，在 **3×3 m 场地（机器人居中）** 上：

1. **开环找速度区间**：每个方向的 **死区下限**（最小能可靠起步的 cmd）和
   **饱和上限**（actual 不再随 cmd 增长的 cmd）→ 给出
   `min_cmd_v / v_max / a_max` 和 `min_cmd_w / w_max / alpha_max`。
2. **闭环调位置环 PID**：在上面速度区间内，重调 `dist.Kp/Kd`、`yaw.Kp/Kd`，
   在 **高噪声万向轮（有时严重拖尾）** 下做到低超调 + 好重复性。

成功标准：更新后的 `position_controller.yaml` 跑 `position_controller_field_test.py`
的 straight / rotate / repeat 项达到（或逼近）原验收线：直线残余 ≤2cm、旋转残余
≤~1.2°、定点重复散布小；且这些是在 3–5 次重复的 **最坏 caster 状态** 下成立。

## 2. 现有可复用资产（不重写）

- `position_control_core.py`：纯算法位置环（位置式 PID + 梯形限速 + 死区 floor +
  turn-drive-turn）。**不改算法**，只改 yaml 参数。
- `position_controller.yaml`：所有待标定参数都在这里。
- `position_controller_field_test.py`：闭环验收 harness，自带 **3×3 安全框看门狗**
  （odom 越界即 cancel→位置环发零速）、`--no-confirm`、CSV、`_latest` 软链日志。
  Phase B 直接复用它，只在外面套一层「换 yaml → 跑一项 → 读 CSV → 排序」。

缺口：**没有开环表征脚本**（harness 是闭环的）→ Phase A 需要新脚本。

## 3. 链路与真值

```
chassis_characterize.py ──Twist──▶ /cmd_vel ──▶ wheeltec_robot_node ──▶ STM32
                                      ▲                    │
                                      └────── /odom ◀───────┘ (twist=轮速真值)
```

- 开环直接发 `/cmd_vel`（wheeltec 直接订阅；**不经** separator / david_move →
  纯 plant，无适配层污染）。Phase A 期间确保没有别的节点发 `/cmd_vel`
  （已 pre-flight：nav/separator 都没跑）。
- 稳态速度 = `/odom` `twist.twist.linear.x` / `angular.z`（轮速），**不是**对 pose
  求导。控制频率 ≤ odom 频率（启动时实测 odom rate）。

## 4. Phase A — 开环速度区间（`scripts/chassis_characterize.py`，新）

### 4.1 直行 sweep
- 对每个 `v_cmd`（低→高，例如 0.02,0.04,…,0.6）：软斜坡升到 v_cmd → 保持
  ~1–1.5s → 在稳态窗口平均 `twist.linear.x` → 减速停 → **odom 闭环慢速 P 回中**
  （仅用于复位，限速很低、安全）→ 反向再测一遍（免费第二样本）。
- 空间：±1.05m 可用。每档前向 burst 估算 ≤0.5m（0.6s 加速 + ~1s 保持），装得下。
- 产出：`v_cmd → v_actual` 映射；**死区下限** = 最坏重复仍起步的最低档；
  **饱和上限** = v_actual 不再随 v_cmd 上升的拐点。

### 4.2 旋转 sweep
- 同上扫 `w_cmd` → `twist.angular.z`，**原地、无空间限制**，可多档多次。
- 产出 `min_cmd_w`、`w_max`（饱和拐点）。

### 4.3 加速度上限
- 从斜坡阶跃响应的 actual 速度上升斜率估 `a_max` / `alpha_max`。

### 4.4 高噪声万向轮处理（关键）
- 每档 **重复 3–5 次**，记录稳态速度 min/median/max。
- **最坏 caster 预置**：直行档前先做一个小幅原地转（甩定脚轮到逆向 =
  ALIGN→DRIVE 最坏工况）；旋转档前先走一小段直行（= "直行后" 旋转最坏工况）。
- **死区 floor = 最坏重复仍能动的档**，不是平均。给 yaml 时再加一点裕量
  （沿用现有 +0.01~0.02 的做法）。

### 4.5 安全
- 自己的 odom 看门狗：越出 ±1.05m 框立即发零速并停当前 sweep。
- 全程 `--no-confirm`，用户在机器人旁守急停。
- 日志 tee 到 `/tmp/david_nav_logs/…` + `_latest` 软链；CSV 落盘。

## 5. Phase B — 闭环位置环 PID（`scripts/pid_sweep.py`，新，套现有 harness）

1. 把 Phase A 结果写进 `position_controller.yaml`：
   `v_max/w_max/a_max/alpha_max/min_cmd_v/min_cmd_w`。
2. **粗扫**：
   - 旋转环：对 ±90° rotate 扫 `yaw.Kp × yaw.Kd` 网格，每候选跑 3× 取最坏。
   - 直行环：对 1m straight 扫 `dist.Kp × dist.Kd`，每候选跑 3×。
   - 每候选：生成临时 yaml → `position_controller_field_test.py --no-confirm
     --tests {rotate|straight} --controller-params <tmp>` → 解析 CSV。
3. **排序**：按 `超调 + |残余| + 重复std` 加权选最小；偏好低超调且对 caster
   噪声稳健的解。
4. **手工精修**：我读粗扫结果，对最优邻域手动微调 1–2 轮确认。

## 6. 交付物

- 更新后的 `config/position_controller.yaml`（速度区间 + PID）。
- 原始 CSV（开环 sweep + 闭环候选）。
- `docs/reports/2026-06-23-new-chassis-retune.md`：速度区间表 + 选定增益 + 依据。

## 7. 执行顺序（= 实施计划）

1. 起 wheeltec（我），等 `/odom`，实测 odom rate。
2. 写 `chassis_characterize.py`，先 **干跑校验**（ast/小幅低速点动确认方向、看门狗）。
3. 跑 Phase A 直行 + 旋转 sweep，落 CSV，定速度区间。
4. 写 Phase A 结果进 yaml，`colcon build`（如需）。
5. 写 `pid_sweep.py`，干跑校验。
6. 跑 Phase B 粗扫 → 手工精修 → 定 PID。
7. 全量 `field_test --tests straight,rotate,pose,repeat` 复验。
8. 写报告，commit + push `feature/david_main`。

## 8. 风险

| 风险 | 缓解 |
|---|---|
| 70kg 自主驱动失控 | ±1.05m 看门狗发零速 + 人守急停 + 低速起步 |
| 直行 sweep 空间不够 | 每档 burst ≤0.5m + odom 闭环回中 + 越界看门狗兜底 |
| caster 噪声让单次读数不可信 | 每档 3–5 次重复 + 最坏预置 + 用 min/median/max |
| 旧 PID/死区使新硬件起步打滑或超调 | 先开环定区间再闭环，区间为 PID 的 clamp/floor 边界 |
