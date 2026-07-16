# DWB 控制优化：参数复核 + 调参方向(feature/dwb-control-opt)

- 日期：2026-07-07
- 分支：`feature/dwb-control-opt`
- 目标：用**同一固定用例**(`controller_bench/reference_path.yaml`)测/调 nav2 DWB,对比
  position_controller 的 10 次 baseline,冲门框验收(cross-track ≤10cm、yaw ≤5°、平滑连续)。
- 测法:`DwbFollowPathAdapter` 把固定折线 densify 成 dense Path,发 `controller_server` 的
  `FollowPath`(绕 planner)→ DWB 跟踪。`run_bench.py --controller dwb`。

---

## 1. position_controller 短板 → DWB 对应旋钮

| baseline 短板(10 次) | DWB 里对应的调节点 |
|---|---|
| 终点 yaw 超调 +5.8°、max 10.3° | **`general_goal_checker.yaw_goal_tolerance`(现 3.14=不管!)** + `RotateToGoal` critic |
| 位置误差均值 9.8cm | `general_goal_checker.xy_goal_tolerance`(现 0.30 太松)+ `GoalDist` |
| cross-track RMS 17cm / max 45–67cm | `PathDist` / `PathAlign` scale、`controller_frequency`、`sim_time` |
| 直行侧偏、拐点过冲 | 同上 + `vx/vtheta_samples`、`Oscillation` |
| turn-drive-turn 不流畅(停-转-走) | **DWB 本身连续跟踪 = 天生比 turn-drive-turn 平滑**(最大优势) |

---

## 2. 当前 DWB 参数复核(config/nav2_params_2d.yaml)

按"问题严重度"排:

| # | 参数 | 现值 | 问题 | 建议 |
|---|---|---|---|---|
| ★1 | `general_goal_checker.yaw_goal_tolerance` | **3.14** | **末端 yaw 完全不约束** → 终点朝向随机,操作/穿门必废 | **0.087(5°)**,配 RotateToGoal 收尾 |
| ★2 | `general_goal_checker.xy_goal_tolerance` | 0.30 | 位置容差 30cm,终点可差 30cm | **0.08–0.10** |
| ★3 | `controller_frequency` | 10 Hz | 控制周期粗 → cross-track/抖动大 | **20 Hz**(看 CPU) |
| 4 | `critics` 权重 | PathDist 24 / PathAlign 12 / GoalDist 10 / GoalAlign 6 | 跟踪权重已偏高(好),但穿门要更贴 | 试 PathAlign 12→16、PathDist 24→30;`PathAlign.forward_point_distance` 0.2 偏小易抖,试 0.3 |
| 5 | `BaseObstacle.scale` | 0.2 | 偏低 → 靠近门框可能切角/不避让 | 穿门场景试 0.3–0.5(太高会拒绝规划) |
| 6 | `sim_time` | 1.2 | 偏短,轨迹前瞻不足→不够平滑 | **1.5**(更平滑,注意别过冲) |
| 7 | `vtheta_samples` / `vx_samples` | 20 / 5 | 已非 binary(好);分辨率够 | 保留;要更平滑可 vx→7 |
| 8 | `max_vel_x` / `max_vel_theta` | 0.20 / 0.50 | 上身晃动/硬件上限 | 保留(硬件标定值) |
| 9 | `acc/decel ±2.5`, `theta ±5.0` | | bench 直连时无 velocity_smoother,DWB 端点可能阶跃 | 见 §4 决策 |

> `precise_goal_checker`(xy 0.25 / yaw 3.14)也是 yaw 不管;要精确朝向就切
> `general_goal_checker` 并收紧,或新建一个 tol 紧的 goal_checker 供 bench 用。

---

## 3. 优化方向(先改这三个,再迭代)

1. **收紧 goal_checker**(★1 ★2):yaw 3.14→5°、xy 0.30→0.10。**不改这个,终点精度/朝向永远不达标。**
   这是 DWB 相对 position_controller 最容易拿分的地方。
2. **抬 controller_frequency 10→20 Hz**(★3):直接改善 cross-track 与抖动。
3. **跟踪权重 + sim_time 微调**(#4 #6):PathAlign/PathDist 抬一档、sim_time 1.5,追 cross-track ≤10cm。

每改一组 → `run_bench.py --controller dwb` 同用例跑 ≥10 次 → `bench_metrics.py` 对比 baseline。

---

## 4. 诚实边界(务必先对齐预期)

- **DWB 也是差速**(`max_vel_y: 0.0`)—— **不能侧移修正 cross-track**,只能靠转航向拐回来。所以
  caster 拖尾导致的直行漂移 / 旋转过冲,**DWB 同样吃**(plant 问题,不是 controller 能根治的)。
  → DWB 预期能赢 position_controller 的是**流畅度**(连续跟踪,无停-转-走)和**部分 cross-track**
  (连续闭环 vs 盲走直线),但**终点 yaw 超调、直行侧偏的物理根因仍在**。
  **《验收标准》里"换全向轮"的结论不因换 DWB 而改变。**
- **测 DWB 有两种口径,先定**:
  - (A) **纯 DWB**:controller_server 的 cmd 直接接 `/cmd_vel`(隔离 DWB 本体,和 position_controller
    直连口径一致)——推荐先测这个;
  - (B) **生产口径**:DWB → cmd_vel_nav → velocity_smoother → cmd_vel_separator → `/cmd_vel`
    (带 caster 适配层)。两者结果会差很多,别混。
- bench 直连(A)没有 velocity_smoother,DWB 端点速度可能阶跃 → 若 #9 抖,再决定要不要把 smoother 串进来。

---

## 5. 还差一步才能跑:最小 DWB bringup

`DwbFollowPathAdapter` 已就绪,但需要 **controller_server + local_costmap** 在跑(slam_gt 上):
- 需要:`map->odom->base_footprint` TF(slam_gt 有)、`/odom`、local_costmap 的 scan 源
  (slam_gt 是 `/scan_slam`,需 remap `/scan:=/scan_slam` 或先禁 obstacle_layer 只用 static+inflation)。
- cmd 输出:口径 (A) 需把 controller_server 的 `cmd_vel` 接到 `/cmd_vel`(底盘)。
- 现成 `2d_david_nav2_bringup` 起的是**全栈(含 separator 链)= 口径 (B)**;口径 (A) 需要一个
  **精简 bringup**(只 controller_server + local_costmap + lifecycle,cmd→/cmd_vel)。

**下一步(待确认口径 A/B 后我来做)**:写这个精简 DWB bringup(仿 `position_controller.launch.py`),
然后就能 `run_bench.py --controller dwb` 上机跑对比。

---

## 附:测试命令(bringup 就绪后)

```bash
# (口径A 精简 DWB bringup —— 待建) 起 controller_server + local_costmap on slam_gt
# 然后同用例、同真值:
cd ~/ros2_ws/src/david_nav/scripts/controller_bench
# run_bench.sh 会把额外参数透传给 run_bench.py,所以 --controller dwb 直接加在后面:
./run_bench.sh S1S4_dwb01 --controller dwb --battery-v <V>
# (等价直调) python3 run_bench.py --controller dwb --run-id S1S4_dwb01 --battery-v <V>
python3 bench_metrics.py --controller dwb --out /tmp/dwb_report.md   # 对比 position_controller
```

---

## iter3(2026-07-08):拐角圆弧化 —— 消"停-转-走"

**用户诉求**:转弯丝滑、**中途不许停顿做 turn-straight-turn**(最后入位除外)。

**根因(几何事实,非调参猜测)**:喂给 FollowPath 的 dense path 是带 3 个直角尖角的折线。
尖角处切向瞬跳 90°,差速底盘**不可能连续跟踪**——DWB 只剩两个选择:停下原地转(观察到的
停顿),或冲出绕环(图上拐角过冲环,ct max 22–29cm 全在拐角)。PathDist/PathAlign 越强,
"停在角上转"评分越优。**这是参考路径要求了不可能机动,任何 DWB 参数都解不了。**

**本轮唯一控制侧变量**:`densify_with_fillets`(bench_common)——拐角用切圆弧替代
(`--corner-radius` 默认 0.30m,按相邻段长自动收紧,C3 收到 0.27;`0` =回旧尖角做 A/B)。
DWB 参数一个未动。运动学:r=0.3 × w_max=0.5 → 弧上连续 v≈0.13–0.15 m/s,恰好落在 vx
采样档 [0,0.05,0.10,0.15,0.20] —— "不停车过弯"首次成为可行解。离线已验:采样距≤0.10、
朝向连续≤10°/点、末点=goal+goal_yaw、**圆弧对旧尖角折线最大偏差仅 8.8cm(小于现在拐角
过冲的 22–29cm)**、R=0 完整复现旧行为。

**配套(harness,非控制变量)**:
- run_bench 落 `track_<id>_commanded.csv`(实际下发路径);plot_run 叠画 commanded;
- `analyze_smoothness.py`:停顿事件(位置/时长/分类:拐角停★/直线停★/终点区✓)、
  拐角 min_v(>0=不停车过弯)、w 反向/抖动、双口径 cross-track;
- **bench 输出迁出 /tmp → `~/david_nav_logs/bench`**(重启丢过一整天 raw CSV)。

**iter3 验收观察点**(跑完 `analyze_smoothness.py` 直接给数):
1. 拐角停-转事件 = 0(★项);拐角 min_v > 0(预期 ≈0.10–0.15);
2. w 反向较 iter2(23–34)进一步降;
3. ct vs commanded(圆弧口径)RMS ≤ iter2 的 9.4cm;终点精度不回退。

---

## iter4(2026-07-14):弧可行速度 —— 消"拐角走过头 + 末端高成本调整"

**iter3 实测**:流畅度达标(全程无拐角停顿,C2/C3 min_v 0.15/0.10,蛇行 35→15),
但暴露新问题:**拐角走过头,末端不能一次到位**。

**逐帧证据**(dwb_20260714_111824 最后一段):机器人以 cmd 0.20(**odom 0.24,
线速 plant ~1.2×**)冲进 C3;r=0.27 弧需 w=v/r≈0.74 > max_vel_theta 0.5 → **弧物理
不可行** → 冲出弧外朝 +x(离 goal 0.55m 时 yaw 仅 5°)→ w=0.5 螺旋绕回 7s,期间
**slam 定位跳 3 次**(0.1s 跳 8–23cm,快转+caster 打滑)→ 三次逼近才擦进 0.10 泡,
stateful checker 位置锁存后又漂出 → 在偏 19.6cm 处再花 6s 猎振对 yaw(plant 2.5×)
→ SUCCEEDED 但 err 19.6cm。ABORTED 那次同签名:C2 过头 0.19m 卡 15s 被 progress 杀。

**根因**:巡航 0.20 超过弧可行速度(r0.30@w0.5→0.15;C3 r0.27→0.135)。C1/C2 靠长
直线让 DWB 来得及降速,C3 前仅 1.02m,重新加速即撞弧。

**本轮单假设**:"让每个弧在巡航速度下可行" —— 两个耦合旋钮共同服务这一个假设:
1. `max_vel_x`/`max_speed_xy` 0.20→**0.15**(两值必须相等;降速方向安全);
2. `min_seg_frac` 0.45→**0.50**(C3 弧 0.27→0.30,三弧统一 0.30)。
   ⇒ w=v/r=0.15/0.30=0.50=w_max,弧恰好可行;末段留 0.30m 已对齐 90° 直线,
   终点旋转量趋近 0。离线已验:三弧 r=0.300/扫角 90°、末段 0.30m@90°、不变量全过。

**iter4 验收观察点**:
1. C3 出弧朝向 ≈90°(不再朝 +x 冲出);无螺旋绕回,d2g 单调收敛一次到位;
2. 终点区停顿(旋转整定)从 6.1s 显著缩短,终点 err 从 19.6cm 回到 ≤8cm;
3. 定位跳变消失(慢转不甩 slam);拐角仍无停顿(min_v≈0.15);
4. 代价:总时长 +20~25%(直线段 0.15)。
