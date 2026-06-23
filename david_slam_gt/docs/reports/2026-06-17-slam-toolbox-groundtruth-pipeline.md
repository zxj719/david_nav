# slam_toolbox 真值/定位管路：为 controller 调参提供近似真值

- 日期：2026-06-17
- 范围：用 `.bashrc` alias `1`(wit IMU) + `2`(LSLIDAR C16) 这两个驱动，做一套
  slam_toolbox 启动包，把 3D 雷达压成 2D、滤地面与自身点，出 `/groundtruth/pose`，
  给全流程自主导航测试(主要 controller)做近似真值。
- 代码落点：`ros2_ws/src/david_slam_gt/`
- 关联：[apriltag 真值方案评估](../../../david_nav/docs/reports/2026-06-17-apriltag-truth-oracle-feasibility-and-bugfix.md)、
  P0 真值管路 spec、P1 自动调优 spec

---

## 1. 一句话结论

**可行，已落地并构建通过。** 用户工作流：**先 mapping 建图存图 → 再 localization
锁图跑导航调参**。slam_toolbox 替换 lightning，自己发 `map→odom`，复用 wheeltec
轮速计补 `odom→base_link`，对外发 `/groundtruth/pose`。这是"独立于 cmd 链、独立于
轮速计漂移"的真值，够 grade controller 的路径跟踪/过冲/震荡；但**不是外部独立真值**
(slam_toolbox 同时也是 nav 的定位源)，抓不了"定位本身导致的误差"。详见 §6。

---

## 2. 架构 / 数据流

```
alias 2  LSLIDAR C16 ──► /c16/lslidar_point_cloud (3D, frame=laser_link)
                          │  self_filter (按 URDF 本体盒抠机身)
                          ▼
                       /c16/points_self_filtered
                          │  pointcloud_to_laserscan (360°, 高度过滤)
                          ▼
                       /scan_slam (2D) ──► slam_toolbox ──► map→odom + /map
wheeltec ──► /odom (话题, 无TF) ──► odom_to_base_tf ──► odom→base_link
static: base_link→laser_link(0.352,0,0.3775) / base_link→base_footprint(I)
                          ▼
       groundtruth_pose_publisher: 查 map→base_footprint ──► /groundtruth/pose
alias 1  wit IMU ──► /imu/... (slam_toolbox 不消费，见 §6 注)
```

最终 TF 树（自洽，不依赖 david_nav 的 RSP）：
`map →(slam_toolbox) odom →(轮速计) base_link → {laser_link, base_footprint}`

---

## 3. 关键决策与坑（按踩的顺序）

### 3.1 为什么 slam_toolbox 需要外部 odom，而本机没有现成的
slam_toolbox 是 2D 激光 SLAM，**消费** `odom→base` 作运动先验、**产出** `map→odom`。
它不产 odom。本机 `rf2o/laser_scan_matcher/robot_localization` 全没装。
用户选定**复用 wheeltec 轮速计**作 odom 源(零新依赖、立即可跑)。

### 3.2 lightning 一关，`odom→base` 就断了（核心坑）
实查 `waypoint_nav_bringup.launch.py`(2026-06-09)：链路是
`map →(Lightning) odom →(Lightning) base → base_link`，而 **wheeltec 不发任何 TF**
(源码无 `TransformBroadcaster`)。即 `odom→base` 一直是 lightning 发的。
换成 slam_toolbox(只发 `map→odom`)后这条边消失 → 必须自己补。
`odom_to_base_tf` 节点订阅 `/odom` 话题、广播 `odom→base_link` TF 补上。

### 3.3 90° 坑：怎么绕过
david.urdf 的 `base_link` 是 **IMU 体系**(+X=机身左、+Y=机身后)，非 REP-105。
`base_link→base_footprint` 带 `Rz(-π/2)`。若沿用，把轮速计(base_footprint 朝向)
贴到 base_link 要补 +π/2，错一点航向就偏 90°。

**绕过法（用户建议，已采纳）**：本包不沿用 david 的 base_link 约定，直接把 `base_link`
定义成干净的 REP-105 前向落地系(+X=机身正前)。这样：
- 轮速计 `/odom` 本就是前向系 → `odom→base_link` **原样转发**，无旋转修正。
- `base_footprint→laser_link` 反算出来**恰好是单位旋转**：平移 `(0.352, 0, 0.3775)`
  ——雷达机身正前 0.352m、离地 0.3775m。所以 `base_link→laser_link` 静态也纯平移。

几何来源（david.urdf）：
```
base_link(david) -> laser_link    : xyz="0 -0.352 0.1275"  Rz(-π/2)
base_link(david) -> base_footprint: xyz="0 0 -0.25"        Rz(-π/2)
反算 base_footprint -> laser_link  = (0.352, 0, 0.3775)  旋转=I   ← 本包 base_link 用这个
```

---

## 4. 压 2D + 过滤（数字按 URDF 算定）

**地面/天花板过滤** `config/pointcloud_to_laserscan.yaml`，`target_frame: base_link`(落地,z=0)：
- `min_height: 0.15`(滤地面回波，雷达离地 0.3775m)、`max_height: 1.20`(滤天花板，留墙体)。
- **全向 360°**(`angle_min/max=±π`)，给回环/scan-match；与 david_nav 那个前向 180°
  `/scan`(给 costmap)互不干扰，独立话题 `/scan_slam`。

**自身点过滤 = box，不是径向圈**（`self_filter` 节点 + `config/self_filter.yaml`）：
- 为什么不用 `range_min`：雷达在机身正前 0.352m，机身主要在它**后面**(后 0.65m、
  前仅 0.07m)。以雷达为心的 0.5m 圆会把正前方空地白白挡掉、又够不到机身后部 → 错。
- 按 `david.urdf` + `base_link.STL` **实测包围盒**反算到 base_link(前向落地系)的本体盒：

  | 轴 | 范围(m) | 来源 |
  |---|---|---|
  | X 前 | −0.30 .. +0.42 | chassis CAD Y 范围(后 0.30/前 0.42) |
  | Y 侧 | ±0.30 | chassis CAD X ±0.30(含轮子 ±0.296) |
  | Z 高 | 0 .. 0.45 | chassis CAD Z −0.146..+0.30 反算落地 |

  各边留 ~0.03m 余量(见 self_filter.yaml)。节点字段无关、保留 intensity/ring/time。
- **chassis-only urdf 不含手臂**：若物理上手臂/上装伸进雷达高度层，改 `self_filter.yaml`
  把盒子相应加大或加第二个盒(当前单盒覆盖底盘本体)。

---

## 5. 两段式工作流

### 阶段一 · 建图(mapping)
```bash
# 另起 wheeltec(出 /odom) + 键盘遥控
ros2 run david_slam_gt launch_slam_gt.sh mapping     # 起 IMU+LSLIDAR+p2l+slam_toolbox
#   遥控机器人把场地转一圈(慢、多回环)，RViz 看 /map
ros2 run david_slam_gt save_map.sh ~/ros2_ws/src/david_slam_gt/maps/david_map
#   产出 david_map.posegraph + .data (+ .pgm/.yaml)
```

### 阶段二 · 定位 + 导航调参(localization)
```bash
# 另起 wheeltec + nav2(use_rsp:=false enable_scan_bridge:=false, 不起 lightning)
ros2 run david_slam_gt launch_slam_gt.sh localization ~/ros2_ws/src/david_slam_gt/maps/david_map
#   slam_toolbox 锁图出 map→odom; /groundtruth/pose 实时发(20Hz, map 系下 base_footprint)
ros2 topic echo /groundtruth/pose         # 给 P1 trial_runner 订阅/录
```

`/tmp/slam_gt_latest.log` 是 symlink，直接 `tail -F`。

---

## 6. 真值的诚实边界（务必先看）

1. **IMU(alias 1) slam_toolbox 根本不吃**。2D 激光 SLAM 不用 IMU。按用户要求照常起，
   但它在这条链里只是陪跑/留给以后 EKF。真要用上 IMU(原地旋转更稳)需加
   robot_localization 把 IMU 偏航和激光里程计 EKF 融合 —— 当前未做。
2. **不是外部独立真值**。lightning 不跑、slam_toolbox 既当定位又当真值：
   - ✅ 它**独立于** cmd_vel 链(位姿来自 scan-match，不受 separator/DWB 影响)、
     **独立于**轮速计漂移(scan-match 把轮速累积误差按死) → grade controller 的
     路径跟踪/过冲/震荡**够用**。
   - ❌ 它**不独立于** nav 的定位(就是同一个 slam_toolbox) → 抓不了"定位本身导致的
     误差"。要那种，得用 apriltag 那条完全外部的真值。
3. **localization 锁图 = 跨轮帧一致**(对标 apriltag 固定 tag 的思路)：每轮在同一张图
   上重定位，真值坐标系不漂，A→B 反复调参可比。
4. **轮速计只是先验**：David 脚轮打滑、轮速 3× 存疑，但 slam_toolbox scan-match 会持续
   纠正它；odom 烂只在雷达特征极少(空旷/长走廊)时才会拖累匹配。

---

## 7. 与 nav2 同跑的约定（避免 TF 冲突）

本包**自洽拥有**机器人 TF(`base_link→laser_link/base_footprint`)和 `/scan_slam`。
调参时 nav2 那侧务必：
- `use_rsp:=false`（别再起 robot_state_publisher，否则 `base_link→laser` 双发且按
  david 90° 约定，与本包冲突）
- `enable_scan_bridge:=false`（别和本包 p2l 重复；nav2 costmap 用它自己的 `/scan` 没关系，
  本包用独立的 `/scan_slam`）
- **不起 lightning**（否则 `map→odom` 双发）

---

## 8. 文件清单

```
ros2_ws/src/david_slam_gt/
  package.xml / CMakeLists.txt              # ament_cmake
  config/
    pointcloud_to_laserscan.yaml            # 3D→2D, 高度+自身过滤, 360°
    slam_gt_mapping.yaml                    # slam_toolbox 建图
    slam_gt_localization.yaml               # slam_toolbox 锁图定位
  david_slam_gt/
    self_filter.py                          # 按本体盒抠机身点云(box, 非径向)
    odom_to_base_tf.py                      # /odom 话题 → odom→base_link TF (补 lightning 的洞)
    groundtruth_pose_publisher.py           # map→base_footprint → /groundtruth/pose @20Hz
  launch/
    slam_gt_mapping.launch.py               # statics + odom_tf + p2l + slam(async)
    slam_gt_localization.launch.py          # 上 + slam(localization,锁图) + gt_pose
  scripts/
    launch_slam_gt.sh                       # 一键起(含 alias 1/2 驱动) + 自动 log
    save_map.sh                             # 序列化存图
  maps/                                     # 存 david_map.posegraph/.data
```

构建：`colcon build --packages-select david_slam_gt`
（注意：用系统 cmake 3.22，需把 `~/.local/bin` 的 cmake 4.1 移出 PATH，否则构建失败）

---

## 9. 待办 / 未做（诚实清单）

- [x] 自身点 box 过滤（self_filter，按 URDF 本体盒；已替换径向 range_min）
- [ ] 若物理手臂/上装伸进雷达层，self_filter.yaml 加大盒或加第二个盒（当前单盒=底盘）
- [ ] 真机标定 `base_link→laser_link` 平移（0.352/0.3775 来自旧 IMU↔LiDAR 外参，URDF 注释
      自己标了"应对实物复核"）
- [ ] 若原地旋转处真值发飘，再上 robot_localization 把 IMU 融进来（§6.1）
- [ ] 端到端实跑验证（本报告为离线构建通过 + 几何推导，未上真机）
```
