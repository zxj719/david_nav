# 工作报告：坐标点导航 v2 落地 + 与抓取栈 TF 统一 + 联调排障

日期：2026-06-11 ~ 06-12
范围：`david_nav` waypoint 导航栈（headless 执行器、TF 拓扑、容差、可视化）
分支：`feature/david_main`

## 0. 一句话

把语义 POI 导航协议升级到"坐标点直达"（nav 协议 v2），做了 headless 一键脚本，
把导航的 TF 树和抓取（上肢）栈统一成一棵，期间排掉若干联调实锅；并为后续
"路径不穿膨胀层"留了设计稿。导航能跑通到点，**剩余主要短板是定位运动期漂移
（~0.9m）和逐段停顿不够丝滑**，均已定位根因。

---

## 1. nav 协议 v2：navigate_to 携带坐标

**背景**：server 侧 2026-06-10 协议 v2，唯一改动是 `navigate_to` 的目标从
`target_id`（POI 字符串，需查 named_poses.json）改为 `target_pose {x,y,theta}`；
其余消息（progress/arrived/error/abort/心跳）不变。

**实现**（commit `b1f421e`）：
- `navigation_protocol.py`：新增 `NavigateToPoseCommand`。payload 里有
  `target_pose` 走 v2 坐标，否则回落 v1 `target_id` —— **新旧并存**，server
  升级期间两种消息都能处理。
- `navigation_executor_core.py`：坐标目标即席构造 `StoredPoi`，跳过 poi_store；
  `theta_in_degrees` 开关控制单位换算。
- `navigation_executor.py`：新增 CLI `--action-name`（waypoint 栈指
  `/waypoint_follower/navigate`，与 bt_navigator 同 NavigateToPose 类型，bridge
  直接复用）、`--theta-degrees`。
- 测试：v2 解析 ×2 + core 坐标流程 ×2。

**悬留**：协议 §四 theta 单位待 server 确认，默认弧度；确认是度则加
`--theta-degrees`。

---

## 2. headless 一键脚本 `start_waypoint_nav_headless.sh`

`start_navigation_headless.sh` 的 waypoint 版。定位演化后**只管导航侧**：

```
jetson_clocks 锁频 → wheeltec 底盘(默认带起,查重杀旧再起)
  → BG=1 launch_waypoint_nav.sh → 等 /waypoint_follower 就绪
  → 前台 navigation_executor(连 ws://172.16.22.205:8101)
```

- SLAM 侧（IMU/雷达/重定位）由 SLAM 流程负责，脚本不碰（曾短暂内置过，按
  用户要求移除）。
- 退出只收导航栈 + executor，底盘留着。
- `--no-chassis` / `--theta-degrees` / `--use-rviz` / `--dry-run` 可选。

---

## 3. TF 树与抓取（上肢）栈统一 ★

**问题**：抓取栈全身 URDF 的 RSP 与导航的 chassis-only RSP 同在 domain 0：
- 抓取曾以 `world` 为根、发 `world→base_link`；导航发 `base→base_link`
  → **base_link 双父 → TF 裂成两棵树** → 执行器 `ConnectivityException`。
- 旧 pre-flight 无差别 `pkill robot_state_publisher/joint_state_publisher`
  → 把抓取的 RSP/jsp 也 kill -9 → `/joint_states` 断 → MoveIt
  `allowed_start_tolerance` 校验失败 → 抓取轨迹 abort（-4），机械臂瘫。

**方案**（commit `7b96308`，导航侧）：

| 项 | 改动 |
|---|---|
| TF 桥接 | 静态 tf 由 `base→base_link` 改为 `base→world` + `world→base_link` 两条恒等。机械臂 RSP 在不在都是一棵树 |
| RSP 隔离 | 导航 RSP 改名 `nav_robot_state_publisher`，`robot_description` 重映射到 `/nav/robot_description`，不抢默认话题（抓取 jsp 订 `/robot_description`，被底盘 URDF 覆盖会判 invalid） |
| pre-flight | 不再无差别杀。只清：① 残留底盘 RSP（URDF 含 `chassis_root` 判定，机械臂 URDF 无此 link）② 旧式 `base→base_link` 静态 tf（与新拓扑冲突）③ 旧导航栈节点。**机械臂 RSP/jsp/相机 tf 一律不碰** |
| 守卫 | headless 脚本每 3s 按同规则常驻清理（修了函数改名后调用点未同步导致 `set -e` 退出的 bug） |
| RViz | `nav2_robot_description` 跟随，Durability 改 Transient Local |

**抓取侧同日 v2 重构**（lrx）：根切到 `base_link`、删 `world`、删底盘段
（轮/lidar_link/imu）。导航的 world 桥**保留不简化**——新版（不发 base_link
父）/ 旧版（发同值 world→base_link）都兼容。

**合成树**：
```
map → odom → base → world → base_link   ← 导航发(base_link 唯一父)
                              ├─ Link1~4 → 双臂/相机/手   ← 抓取发
                              └─ base_footprint/laser/imu  ← 导航发
```

**台架验证**：导航栈 + 新版抓取 RSP + jsp 同跑，
`base→camera_color_optical_frame`、`base→left_hand_base_link`、
`base→base_footprint` 全链可达，单父无冲突。

**遗留问题给抓取组**：`lidar_link` 现在两边都不发（导航只有 `laser_link`，
位置不同）。若抓取感知有消费者，需给外参由导航在底盘 URDF 补 fixed joint。

---

## 4. 其它优化

| 项 | commit / 状态 | 说明 |
|---|---|---|
| 执行器终端防刷屏 | `0e5d7c2` | 空闲只在状态变化时打印；导航中 STATUS 0.3Hz；WS 重连失败第 1 次 + 每 10 次提示 |
| 终点容差 | `a3c1a89` | `goal_dist_tol`/`pose_dist_tol` 0.4→0.3 |
| 巡航参数 | **未提交** | `max_segment` 2.0→3.0、`wp_dist_tol` 0.8→1.0（见 §6 结论：对丝滑无效，待定）|
| 雷达 RViz 可视化 | 已改 lslidar_c16.rviz | 加 WaypointNav Plan / Waypoints+Goal(MarkerArray) / Goal(Pose)；symlink-install 直接生效 |
| 膨胀感知稀疏化 | `db9460a`（设计稿） | 见 §5 |

---

## 5. 设计稿：相邻路径点直线段不穿膨胀层

文档：`docs/reports/2026-06-11-inflation-aware-sparsifier-design.md`

- **现状**：inflation 梯度带仅 0.1m 宽（radius 0.40 − inscribed 0.30），LOS
  阈值 99 对 1~98 的膨胀值不设防，连线可贴 inscribed 切角。
- **方案**：① `inflation_radius` 0.40→0.75 + `cost_scaling` 2.5 铺开梯度
  ② sparsifier 双阈值 LOS（strict 50 优先，99 兜底，窄区自动退化不瘫痪）
  ③ 末段/进场段豁免 strict（贴桌目标在膨胀带内必须可达）
  ④ executor 扩圈判定 cost==0 放宽为 <99。
- **前置**：修 `/scan` 污染、定位动态漂移定量（决定 radius 最终取值）。

---

## 6. 联调实锅与排障结论

| 现象 | 根因 | 状态 |
|---|---|---|
| 执行器刷屏 errno 111 | server 8101 未监听 | server 起来即恢复；已加防刷屏 |
| 到点差 ~1m（停 8.5,-9 vs 目标 9.52,-8.82） | **定位运动期漂移 ~0.9m**：轮 odom 说到点（0.1m），map 说到点（0.21m），停稳后定位才修正回真实位置。日志逐段打 `<<< moveto_ok 但 map 残差>0.5m` | 归 SLAM 侧：180° 点云裁剪后特征、运动畸变、轮速计标定 |
| `/scan` 被污染 | 网内另一台设备 `om1_ros2_driver_node`（domain 0）抢发 `/scan`，时间戳=设备开机秒数 → costmap 障碍层全丢 | 待隔离 om1 / 换专用 topic |
| 规划失败 `failed to create plan` | POI 库是 5月22 旧点，v1 POI_GATE 坐标对应旧地图 | v2 坐标协议绕开 |
| `ConnectivityException` | 见 §3（base_link 双父） | 已修 |
| 脚本启动即退 `kill_foreign_tf_publishers 未找到命令` | 函数改名后主流程调用点未同步 + `set -e` | 已修 |

### "导航不够丝滑"的诊断（重要结论）

用户反馈把 `wp_dist_tol` 放到 1.0m 后仍不丝滑。**根因：`wp_dist_tol` 只控制
DRIVE 阶段的到点距离，不解决卡顿。** position_controller 的 POSE 模式是
**turn-drive-turn 状态机**，卡顿来自 waypoint_follower 逐段离散驱动——每过一个
点都要：① 完全停车 ② 等定位收敛 `_stable_map_pose()`（0.5~6s）③ set_parameters
切容差 ④ 原地 ALIGN 转向 → DRIVE。这四步 `wp_dist_tol` 一个都不碰，放大它只是
把停车点前移 1m，循环原样保留，还放大横向偏移 / 贴障碍风险。

**要真正丝滑的方向**（按性价比，待决策，均未实施）：
1. 段间不等定位收敛（中间点不等、只末点等）——砍掉停顿大头；
2. 省掉中间段原地 ALIGN（caster 底盘有回退风险）；
3. 架构级换连续路径跟踪（Pure Pursuit），治本但要重写驱动层。

附注：`set_parameters` 服务超时/被拒只告警不报错，会退回 launch 默认
`pose_dist_tol 0.30` —— 容差是否真切进去需看日志 `容差未切换` 字样。

---

## 7. 提交记录（feature/david_main）

```
db9460a docs(nav): 膨胀感知稀疏化设计稿
a3c1a89 david_nav: 终点定位容差 0.4→0.3m
0e5d7c2 david_nav: 执行器终端防刷屏
7b96308 david_nav: TF 树与机械臂(抓取栈)统一 —— base→world→base_link 桥接
b1f421e david_nav: nav 协议 v2(坐标点导航)+ waypoint headless 一键脚本
```

未提交：`config/waypoint_follower.yaml`（max_segment 3.0 / wp_dist_tol 1.0，
丝滑参数实验，待实车结论）。

---

## 8. 下一步

1. **定位漂移**（SLAM 侧，最高优先）：同一目标重复导航统计落点散布，区分
   动态漂移 vs 系统性偏差。决定后续 inflation_radius 取值。
2. **/scan 污染**：隔离 om1 或换专用 scan topic，否则膨胀只对静态图生效。
3. **丝滑**：先确认容差是否切进去 + 量化 `_stable_map_pose` 等待耗时，再从
   "段间不等收敛"入手。
4. **膨胀感知稀疏化**：前置项清掉后实施 §5 设计。
5. **theta 单位**：等 server 确认弧度/度。
