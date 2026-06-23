# base → base_link 变换说明（给团队成员）

日期：2026-06-12 ・ 维护：导航组 ・ 关联：`launch/waypoint_nav_bringup.launch.py`

## 一句话

导航栈把 `base` 经 `world` 桥接到 `base_link`，两段都是**零变换（恒等）**：

```
base ──(静态tf, 恒等)──► world ──(静态tf, 恒等)──► base_link
```

`base`、`world`、`base_link` 三个帧**在空间上完全重合**（平移 0、旋转 0）。

## 完整 TF 链

```
map ─► odom ─► base ─► world ─► base_link ─┬─► base_footprint   (导航/底盘 URDF)
 │      │      │                            ├─► laser_link / imu_link
 └Lightning发  └本launch发两条恒等静态tf     └─► Link1~4 → 双臂/相机/手 (抓取栈发)
```

- `map→odom→base`：Lightning 定位发布。
- `base→world`、`world→base_link`：**本 launch 的两个 static_transform_publisher
  发布**（节点名 `base_to_world_tf`、`world_to_base_link_tf`），全 0 参数。
- `base_link` 往下：底盘段由导航的 RSP 发，机械臂段由抓取栈的 RSP 发。

## 为什么要桥过 world（而不是直接 base→base_link）

抓取（上肢）栈的全身 URDF 历史上以 `world` 为根、发 `world→base_link`。
若导航直接发 `base→base_link`，会和它形成 **base_link 双父 → TF 裂成两棵树**，
导航执行器报 `ConnectivityException`、定位/规划失效。

桥过 `world` 后：
- 抓取**新版**（已删 world、根切到 base_link、不再发 base_link 的父）→ 直接
  并入，单父，一棵树。
- 抓取**旧版**（仍发 `world→base_link`）→ 与本 launch 的同名变换**同值重复**，
  无害。

两版都兼容，导航与抓取可同时跑。

## 各帧含义

| 帧 | 来源 | 含义 |
|---|---|---|
| `base` | Lightning | 定位输出的机体根帧 |
| `world` | 本 launch（桥接） | 仅作 base 与 base_link 的中转，无物理意义 |
| `base_link` | URDF 根 | 机器人 URDF 的根；底盘与机械臂子树都挂这里 |
| `base_footprint` | 底盘 URDF | base_link 正下方地面投影（导航 costmap 的 robot_base_frame）|

## 给协作方的约定

- **`base_link` 由导航唯一发布其父**（经 base→world→base_link）。其他栈
  **不要**再发 `base_link` 的父变换（如 `world→base_link`、`odom→base_link`），
  否则双父裂树。
- 机械臂/相机/手等子帧挂在 `base_link` 之下，由抓取栈发，导航不碰。
- 导航的 robot_description 走 `/nav/robot_description`（不占用默认
  `/robot_description`）。

## 验证命令

```bash
# 三帧应全为 0 平移、单位四元数
ros2 run tf2_ros tf2_echo base world
ros2 run tf2_ros tf2_echo world base_link
ros2 run tf2_ros tf2_echo base base_link        # 仍是 0：三帧重合

# 跨栈联通（导航+抓取同时跑时）：应能解析
ros2 run tf2_ros tf2_echo base base_footprint            # 导航帧
ros2 run tf2_ros tf2_echo base camera_color_optical_frame # 抓取帧
```

全 0 输出 = 正常。报 `frame does not exist` / `connectivity` = TF 链有断点或双父。
