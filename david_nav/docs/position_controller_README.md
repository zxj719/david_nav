# position_controller — David 上位机位置环

> Nav2 之外的并列速度来源。对**车体位姿**做闭环 (PID → cmd_vel)，驱动 David 走精确
> 距离 / 转精确角度 / 定点泊车。直线 + 原地转 + turn-drive-turn，**不做避障/规划**。
> 实现规格见 `weekly_report/位置环实现方案_报告.md`。

两轮差速 + 从动脚轮，非完整约束 (vy=0)：节点只发 `linear.x` / `angular.z`。

---

## 1. 交付物

| 文件 | 说明 |
|---|---|
| `david_nav/position_control_core.py` | 纯控制数学 (PID / 梯形限速 / 距离投影 / turn-drive-turn / 到位锁存)，无 ROS 依赖，可单测 |
| `david_nav/position_controller.py` | rclpy 节点：action server `~/move` + topic `~/goal_pose`，发 `/cmd_vel_position` |
| `david_nav/cmd_vel_mux.py` | 自包含优先级 mux + `~/enable` 服务（不依赖 twist_mux） |
| `david_nav_msgs/action/MoveTo.action` | 自定义 action（独立接口包，避免 rosidl 与 david_nav python 包同名冲突） |
| `config/position_controller.yaml` | §4 参数默认值 |
| `config/twist_mux_position.yaml` | 等价的 twist_mux 配置（需 `apt install ros-humble-twist-mux`） |
| `launch/position_controller.launch.py` | 起 controller + mux |
| `test/test_position_control_core.py` | 13 个单测：math + 三种 mode 收敛 + 验收边界 + 到位锁存 |
| `scripts/sim_fake_chassis.py` | 理想差速底盘仿真（积分 `/cmd_vel_position` → `/odom`），用于 §8 仿真先行 |
| `scripts/position_controller_field_test.py` | 3m×3m 场地验收脚本：依次发 DISTANCE/ROTATE/POSE/重复/取消，量残余误差·超调·散布·停车时间，带**场地边界守护**（越界自动 cancel） |

> **注意**：自定义 action 放在**新建的 `david_nav_msgs` 包**（与 `david_hand_msgs`
> 同惯例）。原因：`david_nav` 是 ament_cmake + ament_python，若在其内部
> `rosidl_generate_interfaces`，生成的 `david_nav` python 模块会与现有的
> `david_nav/` 节点包命名空间冲突，colcon build 失败。

---

## 2. 编译

`david_nav` 现在依赖 `david_nav_msgs`（colcon 会自动排序先建接口包）：

```bash
cd ~/ros2_ws
colcon build --packages-select david_nav_msgs david_nav
source install/setup.bash
```

> 本工作空间属主是 `lrx`。最终落盘编译请用 `lrx` 账号执行（agx 对 build/install
> 无写权限）。开发期算法验证可用单测（见 §7），无需整树编译。

---

## 3. 接口

### 订阅
- `/odom` (`nav_msgs/Odometry`)：位姿反馈，坐标系 = odom。yaw 由 quaternion 取
  `atan2(2(wz+xy), 1-2(yy+zz))`。**直接信 odom 数值**，不对"机身 -y 为前进"做二次
  旋转修正（已被 odom/TF 吸收）。仅校验：正 `linear.x` → odom 前进为正；若为负，置
  `invert_linear:=true`。

### 发布
- `/cmd_vel_position` (`geometry_msgs/Twist`)：位置环速度输出。**仅在 goal 激活时发布**，
  空闲不发 → mux 自动回落到 Nav2。

### 目标接口
**(A) Action（主）** `~/move` = `david_nav_msgs/action/MoveTo`：

```
uint8 mode            # 0=DISTANCE 1=ROTATE 2=POSE  (常量 MODE_DISTANCE/ROTATE/POSE)
float64 distance      # mode0: 沿当前朝向有符号距离(m)
float64 yaw           # mode1: 相对旋转角(rad); mode2: 目标朝向(rad)
geometry_msgs/Point target  # mode2: 目标点(x,y)
bool relative         # mode2: true=相对当前位姿, false=odom 绝对
---
bool success
float64 final_error   # 残余误差(m 或 rad)
---
float64 error_remaining
float64 current_speed
```

支持 **cancel**：取消 → 立即发零速、放弃目标（≤1 周期）。

**(B) Topic（便捷）** `~/goal_pose` (`geometry_msgs/PoseStamped`) → 触发 POSE。
frame 为 `base_footprint`/`base_link`/空 → 相对当前位姿；`odom` → 绝对。
（本节点不做 map→odom 的 TF 变换；map frame 会被当 odom 处理并 WARN。）

### 用法示例
```bash
# 走直线 1 m
ros2 action send_goal /position_controller/move david_nav_msgs/action/MoveTo "{mode: 0, distance: 1.0}"
# 原地左转 90°
ros2 action send_goal /position_controller/move david_nav_msgs/action/MoveTo "{mode: 1, yaw: 1.5708}"
# 定点：相对 (0.5, 0.5)，终点朝向 0
ros2 action send_goal /position_controller/move david_nav_msgs/action/MoveTo \
  "{mode: 2, target: {x: 0.5, y: 0.5}, yaw: 0.0, relative: true}"
```

---

## 4. 控制算法（与 SPEC §3 对应）

- **DISTANCE**：`travelled = (Δx·cosψ0 + Δy·sinψ0)`（沿起点朝向投影）；`err_d = distance - travelled`。
  同时 `err_yaw = wrap(ψ0 - ψ)` 小幅纠偏保持直线。
- **ROTATE**：`err_yaw = wrap((ψ0 + yaw) - ψ)`。
- **POSE**（caster-safe turn–drive–turn）：ALIGN 对准 → DRIVE **纯直行(ω=0)** → FINAL_TURN 终定向。
  **不变量:任何时刻只给 v 或只给 ω,绝不同时(不走弧线)** —— 差速+被动万向轮同时给 v+ω 会卡 caster
  (这正是 cmd_vel_separator 存在的理由)。脚轮拖尾处理:① 相位间**先停稳**(等 caster 摆正/转动量耗散)
  再切;② DRIVE 中航向漂移 >`pose_realign_tol` 则**停下回 ALIGN 纯转**纠正,不靠弧线;③ 末端原地转会
  蹭动落点(前后脚轮摆动),FINAL_TURN 后**复检位置**,偏了就再逼近(上限 `pose_max_cycles` 次)。
  相对目标在 goal 接受时一次性锁存换算到 odom,全程基于锁存值(SPEC §5)。
  > 2026-06-09 实车第一版 POSE 边走边纠(v+ω 同时给)导致 caster 卡死、30s 超时、终点偏 35cm/103°;
  > 改为本严格 stop-and-turn 后仿真 1.8cm/0.49° 通过。直行/原地转早已单独验证通过。
- **位置式 PID**（例程 6 结构）：积分限幅抗饱和（必须），导数用 err 差分（首拍 0，防 setpoint kick）。
- **限幅 + 梯形限加速**：`clamp(v,±v_max)` → 死区地板 `min_cmd_v` → `rate_limit(a_max·dt)`。
- **到位锁存**：`|err|<tol` **连续 `arrival_cycles` 周期** 且 `|speed|<v_stop/w_stop` 才判到位，
  发零速退出（避免穿越目标点误判）。
- **安全**：`goal_timeout` 超时、`odom_timeout` 失联、cancel/新目标/退出 → 先零速再切换。

---

## 5. 参数整定指引（SPEC §4）

整定顺序（实车）：

1. **先纯 P**：`dist.Kp` / `yaw.Kp` 调到刚好不超调，`Ki=Kd=0`，看 `final_error` 和有无来回。
2. **加 D**：`dist.Kd` / `yaw.Kd` 压住超调/振荡。
3. **Ki 留小 + 限幅**：仅当有稳态残差才加 `Ki`，配合 `I_max` 抗积分饱和。
4. **限速**：`v_max` 取 Nav2 平时安全速度（默认 0.25）；`a_max`/`alpha_max` 控制启停柔和。
5. **死区**：见下方 §6.1（关键）。`min_cmd_v/min_cmd_w` **必须非 0**，起始用报告破启值
   0.05/0.18，实车微调。
6. **方向**：先发 `mode:DISTANCE, distance:0.2`，若 odom 倒退 → `invert_linear:=true`，**不要硬编码翻转**。

POSE 专用(caster-safe stop-and-turn):
- `pose_align_tol`(0.05rad)：进 DRIVE 前的对准容差。
- `pose_realign_tol`(0.12rad)：DRIVE 中航向漂移超此值就停下回 ALIGN 纯转纠正。脚轮蹭得厉害就调小。
- `pose_max_cycles`(4)：末端原地转蹭动落点后的最大复逼近次数。多段 stop-and-turn + 停稳会比较慢,
  必要时把 `goal_timeout` 从 30s 适当加大。

关键参数默认值见 `config/position_controller.yaml`。

---

## 6. 与 Nav2 的 mux / 切换（SPEC §6，关键）

位置环与 Nav2 **不可同时**驱动底盘。两种方式：

### 方式 A：自研 cmd_vel_mux（默认，无外部依赖）
优先级：`/cmd_vel_position` (100) > `/cmd_vel_nav` (10) → 输出 `/cmd_vel`。
- 位置环只在 goal 激活时发 → 空闲时 mux 超时回落到 Nav2。
- 源切换时注入一拍零速（零速过渡，防阶跃）。
- `~/enable` 服务 (`std_srvs/SetBool`)：`data:false` 锁掉位置环（强制 Nav2），`data:true` 恢复。

**接入步骤**：把现有 Nav2 链末端（`cmd_vel_separator`）输出从 `/cmd_vel` 改 remap 到
`/cmd_vel_nav`，让它喂 mux 而不是直接喂底盘；mux 输出 `/cmd_vel` 进 wheeltec。

```bash
ros2 launch david_nav position_controller.launch.py            # 起 controller + mux
ros2 service call /cmd_vel_mux/enable std_srvs/srv/SetBool "{data: false}"  # 强制 Nav2
```

> 注意：位置环输出**不经过** `cmd_vel_separator`（其 binary_output/brake 会破坏 PID）。
> mux 直接选位置环 vs Nav2 链。

### 方式 B：upstream twist_mux
`sudo apt install ros-humble-twist-mux`，用 `config/twist_mux_position.yaml`。

### 6.1 与底盘死区 / david_move 的关系（关键，易踩）

底盘是 70kg 重载差速 + 被动万向轮，低速段严重 stick-slip + 脚轮破启阻力；**平稳低速区物理上
不存在**（见 `a_group_results/PID调参报告_20260608.docx`）。最终 STM32 速度 PI = **Kp300/Ki800**
（从 1200 退回），**破启前馈尚未做**（报告列为治本方向）。

死区补偿现在在 **david_move**（`david_cmd`）上，路径 `cmd_vel_safe → /cmd_vel`，硬 lift
`min_linear_x=0.20 / min_angular_z=0.20 / min_stationary_angular_z=0.35`。

> **本位置环 `/cmd_vel_position → mux → /cmd_vel` 不经过 david_move**，吃不到那套 lift。
> 所以死区**必须在 position_controller 自己处理**（`min_cmd_v/min_cmd_w`，方式 A 已选）。
> 若设 0：临门一脚命令（如最后 2cm 的 `1.2×0.02=0.024 m/s` < 0.04 破启，Ki=0 无积分穿透）
> 落进死区 → 车停在目标前几 cm/几度 → 验收 #1/#2/#3 在实车失败。

报告 §7 死区三层（最终 PI 下，最坏=逆向脚轮）：

| 量 | 裸破启命令 | 该档实速 | david_move 生产 lift |
|---|---|---|---|
| 线速度 | ≈0.04 m/s | 0.023（爬行，stick-slip） | 0.20 m/s |
| 角速度 | ≈0.15 rad/s | 0.037（爬行） | 0.20 / 原地 0.35 rad/s |

整定要点：
- 死区是 **增益 × 脚轮朝向 耦合量，不是定值**（PI=1200 时 0.035/0.12，Kp300/Ki800 时 0.04/0.15）。
  config 默认 `min_cmd_v=0.05 / min_cmd_w=0.18` 只是"略高于裸破启"的起点。
- **ROTATE / POSE 对准·终定向是从静止起转**，脚轮横置，破启更大；最坏需把 `min_cmd_w` 抬到
  ~0.30–0.35（参考 david_move 的 `min_stationary_angular_z`）。但越高，越压不住最后几度精度。
- **裸破启实速本身就是 stick-slip 抖动（0.023/0.037）**，不是干净运动。速度地板只能"治标"让
  位置误差靠抽动逐步闭合；2cm/0.02rad 级精度的"治本"是按 Δθ 自适应的**破启前馈脉冲**（报告 §6
  治本二），本实现暂未做。**若实车最后一档反复抽动到不了位，应上前馈 kick，而不是无限抬地板。**
- 验收容差（2cm / 0.02rad / 3cm）是 SPEC 的目标值；在此 plant 上是否物理可达，需对照报告结论
  实测复核，必要时与下游对齐放宽。

---

## 7. 验收复现（SPEC §7 / §8）

### 单测（纯算法，秒级，无需 ROS 运行）
```bash
cd ~/ros2_ws/src/david_nav
python3 -m pytest test/test_position_control_core.py -q     # 13 passed
```
覆盖：wrap/clamp/rate_limit/距离投影、DISTANCE/ROTATE/POSE 闭环收敛到验收边界
（1m≤2cm、90°≤0.02rad、定点≤3cm/0.03rad）、v_max/a_max 限制、到位锁存连续周期、死区地板。

### 仿真先行（理想差速底盘，验证端到端 + mux）
```bash
export ROS_DOMAIN_ID=77
python3 ~/ros2_ws/src/david_nav/scripts/sim_fake_chassis.py &   # /cmd_vel_position -> /odom
ros2 run david_nav position_controller &
ros2 action send_goal /position_controller/move david_nav_msgs/action/MoveTo "{mode: 0, distance: 1.0}"
ros2 topic echo /odom --once        # x ≈ 1.00
```
已验证（仿真）：DISTANCE 1.0 → x=0.993 (err 0.007m)；ROTATE π/2 → err 0.014rad；
POSE 相对(0.5,0.5) → SUCCEEDED；`/cmd_vel_position` max|vx|=0.25(=v_max)、梯形 0.01/拍、
`linear.y`/`angular.x,y` 恒为 0；mux 优先级 + 零速过渡 + `~/enable` 均正常。

### 实车（3m×3m 场地）—— 推荐用脚本
**机器人放场地正中**，四周留出安全框（脚本启动时把 odom 当前位姿当场地中心；安全框
= ±(field/2 − margin)，默认 3.0/0.45 → ±1.05m），人在急停旁值守：

```bash
# 先看会发哪些动作、确认每步（默认每步回车确认）
python3 scripts/position_controller_field_test.py --tests straight,rotate,pose,repeat,cancel
# 全自动 + 写 CSV
python3 scripts/position_controller_field_test.py --no-confirm --tests all --repeat 5 --csv /tmp/ft.csv
```
脚本自动量化：直线残余/侧偏/超调、转角残余/超调、定点位置·朝向误差、重复散布(均值/标准差)、
cancel→停车时间，并对照各项验收阈值打印 `success`。**越界守护**：运动中 odom 出安全框立即
cancel（位置环收到即零速）。1m 直线在 3m 场地居中起步只剩 ~5cm 越界缓冲，脚本会 ⚠ 提示；
实车可把机器人后移或用 `--distance 0.8`。

手动补充项（脚本不覆盖）：
- **限速核查**：另开 `ros2 topic echo /cmd_vel_position`，确认 `|linear.x|≤v_max`、
  `|angular.z|≤w_max`、`linear.y`/`angular.x,y` 恒 0、启停限加速不阶跃。
- **掉线测试**：运动中拔 odom（停 wheeltec/定位）→ 应 ≤ `odom_timeout`(0.3s) 内零速并放弃。
- **死区复核**：见 §6.1，最后一档若 stick-slip 抽动到不了位，调 `min_cmd_v/min_cmd_w` 或上前馈 kick。
- **回归**：确认开启位置环后 Nav2 常规导航不受影响（mux 空闲回落）。

---

### 附：算法出处
串级思想 + 位置式 PID（P+D 主导、Ki 小 + 积分限幅、输出限速）来自 WHEELTEC STM32 PID
学习套件例程 6（单电机转 N 圈）。本实现闭在整车位姿 (距离, yaw)，输出 (v, ω)，内层速度环
复用 C50C 固件现有增量式 PI。
