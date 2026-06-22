# 双轮前移 16.5cm + 旋转中心解耦 — 设计与实施说明

> 日期：2026-06-22 ｜ 分支：`feature/david_main` ｜ 影响：`david_nav` 的 URDF +
> nav2 footprint。属于「底盘差分系统几何变更」类改动。

## 1. 需求

1. **双轮向前平移 16.5cm**（沿 `base_link` 的 +X / 正前方），**轮距 = 50.5cm**，
   机身其他部分（机架 mesh、雷达、IMU）位置不变。
2. 双轮中心因此**不再与机器人本体中心（base_link）重合**：差分驱动的**旋转中心**
   前移，nav2 旋转时必须绕新的旋转中心。整个差分系统需随之更新。
3. **解耦**：旧 URDF 留备份；把差分几何集中到 URDF，使**下次底盘差分更新尽量只改
   URDF**。

## 2. 关键认知（为什么这样改）

- TF 链（2026-06-18 起）：`map → odom → base_link`（SLAM 直发）→ URDF/RSP 子帧
  （`base_footprint` / `laser_link` / `imu_link` / `chassis_root → 双轮`）。
  `base_link` = SLAM 跟踪的**本体帧**，X 朝前；本次**不动**。
- **整个 nav2 栈用 `base_footprint` 作 `robot_base_frame`**（controller_server、
  local/global costmap、collision_monitor、waypoint_follower、position_controller），
  且 footprint 多边形在 `base_footprint` 帧下表达。
- 差分底盘绕**双轮轴中点**原地旋转。`base_footprint` 就是该中点在地面的投影 =
  **旋转中心**。原来双轮在 base_link X=0，故 base_footprint 在 X=0，二者一致。
- 双轮前移 16.5cm 后，轮轴中点 → base_link **X=+0.165**。要让 nav2「绕
  robot_base_frame 原地转」在物理上成立，**`base_footprint` 必须一起前移到
  X=+0.165**。这就是需求 2 的「旋转中心」修正。

## 3. 几何推导

`chassis_root` 相对 base_link 为 `Rz(-90°)`，所以
**base_link +X(前) = chassis_root +Y**，**base_link +Y(左) = chassis_root -X**。
目标双轮 base_link 坐标 `(X=+0.165, Y=±0.2525)` 映射到 chassis_root：

| 项 | base_link 目标 | chassis_root `xyz` | 旧值 |
|---|---|---|---|
| 左轮 `left_wheel_joint`  | (0.165, +0.2525) | `-0.2525 0.165 -0.084` | `-0.2955 0 -0.084` |
| 右轮 `right_wheel_joint` | (0.165, -0.2525) | ` 0.2525 0.165 -0.084` | ` 0.2969 0 -0.084` |

- 轮距 = 0.2525×2 = **0.505m** ✓；前移量 = chassis_root `y` = **+0.165m** ✓。
- 轮距**对称化**（旧值 L=0.2955 / R=0.2969 有 0.7mm 不对称，本次取消，中点 Y=0）。
- z（轮高 -0.084）与 rpy（`1.5708 0 0`）、axis 不变。

**base_footprint**（旋转中心）：`base_link_to_base_footprint` origin
`0 0 -0.25` → **`0.165 0 -0.4075`**（X 0→0.165；z 见下「3b 竖直 / 轮半径」）。

## 3b. 竖直方向 / 轮半径 22cm（2026-06-22 大轮升级）

- 轮轴 z（base_link）= `chassis_root_z(-0.1035) + wheel_z(-0.084)` = **-0.1875**
  （轮 hub 安装位不变，沿用第 1 节「其他保持不变」）。
- 地面（base_footprint z）= `轮轴 z - 轮半径` = `-0.1875 - 0.22` = **-0.4075**。
- 旧值 -0.25 反推旧半径 = 0.0625（6.25cm），是**旧小轮的残留**；换 22cm 大轮后
  机身抬高 15.75cm，地面在 base_link 系下相应下移到 -0.4075。
- **顺带修正激光滤地**：`2d_david_nav2_bringup.launch.py` 的 scan filter
  `target_frame=base_footprint`、`min_height=0.10` 假设 base_footprint 落在
  **真实地面**。base_footprint 之前在 -0.25（高出真实地面 15.75cm），min/max_height
  其实滤错高度；下移到 -0.4075 后该假设成立，**滤波参数无需改**（相对 base_footprint）。
- `wheel_radius=0.22` 已加入 URDF 顶部 SSOT 块；base_footprint z 由
  `axle_z - wheel_radius` 推导。
- **遗留（外观）**：左右轮 visual/collision 仍是旧 STL mesh，若网格是旧小轮，
  RViz 会看到轮子与地面有间隙（纯显示问题，2D 导航/TF 不受影响）；需换大轮 mesh
  时单独处理。

## 4. nav2 footprint 多边形随动

机身未动、base_footprint 前移 +0.165，故 footprint 多边形（base_footprint 帧）在
X 方向后退 0.165，`config/nav2_params_2d.yaml` 的 **local + global 两处**：

```
旧: [[0.42, 0.30], [0.42, -0.30], [-0.30, -0.30], [-0.30, 0.30]]
新: [[0.255, 0.30], [0.255, -0.30], [-0.465, -0.30], [-0.465, 0.30]]
```

- **inscribed 半径 0.30 → 0.255**（前缘变最近边）。`inflation_radius=0.40` 仍
  ≥ inscribed，**无需改膨胀**；只更新 inscribed 注释。

## 5. 解耦做法（本次选「纯 .urdf + 文档化公式」，不引入 xacro）

- 旧文件备份：`urdf/david.urdf` → `urdf/david.urdf.bak.20260622_pre_wheel_fwd`。
- URDF 顶部加「**差分几何 — 单一真值源**」注释块，列出
  `wheel_x_offset=0.165`、`half_track=0.2525`、`wheel_z`，并写明 3 个 origin
  （左轮 / 右轮 / base_footprint）由这些量推导的公式与 chassis_root 的 Rz(-90°)
  映射，使下次只改 URDF 并保持三者一致。
- **已知限制**：nav2 footprint 多边形在 yaml，**无法从 URDF 读取**。在 URDF 注释
  块和 yaml footprint 注释里**互相交叉引用**：若 `wheel_x_offset` 变化，需把该
  多边形 X 同步平移相同增量。（若将来要彻底单一真值源，可转 xacro property 或加
  运行时从 TF 推导 footprint 的节点；本次 YAGNI 不做。）

## 6. 影响范围 / 不变项

- **改**：`urdf/david.urdf`（双轮 + base_footprint + 注释/头部）、
  `config/nav2_params_2d.yaml`（footprint ×2 + inscribed 注释）、新增 1 个 .bak。
- **不改**：launch（仍 `cat urdf`，无 xacro）、cmd_vel_separator 等节点逻辑、
  base_link / laser_link / imu_link / chassis_root 本体几何。
- **雷达角度滤波**（`base_footprint` 系下 ±π/2 前向半圆）为角度量，base_footprint
  平移 0.165 仅轻微改变到某点的角度，前向半平面语义近似保持，**本次不动**。
- **外部固件**（wheeltec / STM32 里程计的运动学模型）不在本 workspace，未触及；
  本次仅修正 david_nav 侧 robot_base_frame=旋转中心 与物理轮轴中点的一致性。

## 7. 验证

1. `colcon build --packages-select david_nav`。
2. `check_urdf`（或 `xacro`/RSP 解析）确认 URDF 合法、TF 树连通。
3. `python -c "import yaml; yaml.safe_load(...)"` 干跑 nav2_params_2d.yaml。
4. 起一次 nav（`launch_nav_with_log.sh`），RViz 看 base_footprint 落在双轮中点、
   footprint 多边形包住机身、原地旋转时机身绕新中心转。
