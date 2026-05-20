# david 抓取相关 ROS 接口速查

> 更新: 2026-05-20 (Stage 1 完成 + Stage 3.a)

## 灵巧手接口 (david_sdk_bridge 提供, 每侧一套 left/right)

### service `/{side}_hand/set_pose10`
- type: `david_hand_msgs/srv/SetHandPose`
- request: `int32[] pose` (长度 10, 每值 0-255; 0=收紧 255=张开)
- response: `bool success`, `string message`

```bash
ros2 service call /left_hand/set_pose10 david_hand_msgs/srv/SetHandPose \
  "{pose: [255,128,255,255,255,255,128,128,128,128]}"
```

### service `/{side}_hand/preset`
- type: `david_hand_msgs/srv/SetHandPreset`
- request: `string name` (open / close / half / grasp)
- response: `bool success`, `string message`

```bash
ros2 service call /left_hand/preset david_hand_msgs/srv/SetHandPreset "{name: 'grasp'}"
```

| preset | pose10 | 形态 |
|--------|--------|------|
| open  | [255,128,255,255,255,255,128,128,128,128] | 完全张开 |
| close | [0]x10 | 握拳 |
| half  | [128]x10 | 半握 |
| grasp | [160,128,160,160,160,160,128,128,128,128] | 矿泉水瓶抓取(现场标定) |

### topic `/{side}_hand/state` (2 Hz)
- type: `sensor_msgs/JointState`
- name: [Thumb_Pitch, Thumb_Yaw, Index_Pitch, Middle_Pitch, Ring_Pitch, Little_Pitch, Index_Yaw, Ring_Yaw, Little_Yaw, Thumb_Roll]
- position: 0-255 当前位置
- effort: 0-255 力反馈 (**力闭环关键信号**)
- velocity: 0-255

```bash
ros2 topic echo /left_hand/state --field effort
```

**effort 实测 (矿泉水瓶)**: 空抓 <=13, 抓物 Index_Pitch/Little_Pitch 飙到 50-80。
力闭环阈值建议: effort > 40, 连续 2 帧, 至少 2 根 pitch 手指触发。

## 机械臂接口

### action `/{side}_arm_controller/follow_joint_trajectory`
- type: `control_msgs/action/FollowJointTrajectory`
- joint_names: `AR5-5_08{L|R}-W4C4A6-ZY2_joint_{1..7}`
- 由 david_sdk_bridge 的 arm_node 提供; lazy power-on (首条轨迹自动上电)

### MoveIt 规划组 (david_moveit_config)
| group | base -> tip | DOF | kinematics |
|-------|-------------|-----|-----------|
| left_arm  | Link4 -> left_tcp  | 7 | KDL |
| right_arm | Link4 -> right_tcp | 7 | KDL |
| both_arms | (subgroup)        | 14 | - |
| waist     | base_link -> Link4 | 4 | KDL (欠驱动) |

### service `/compute_ik`
- type: `moveit_msgs/srv/GetPositionIK`
- move_group 提供; 给末端 pose 解关节角

## Stage 3.a 抓取脚本

```bash
ros2 run david_grasp pick_at_pose left     # 左臂侧面抓取
ros2 run david_grasp pick_at_pose right    # 右臂
```

侧面抓取 5 步: 预抓取位(物体侧向15cm) -> open -> 水平接近 -> grasp -> 后撤 -> 抬升。
默认 grasp_pose 写死在 pick_at_pose.py 的 main(): 左 (0.40,0.10,0.85) / 右 (0.40,-0.10,0.85)。

## 排错

| 现象 | 原因 | 处理 |
|------|------|------|
| service "waiting for service" 卡住 | launch 没起 / arm_node hang | 查 `ros2 node list` |
| IK fail code=-31 | 目标 pose 不可达 | 调近 (x<=0.6, z 0.7-1.1) |
| trajectory rejected | arm_node fallback mock (连不上真机) | 重跑双 NIC 路由 + ping |
| RViz 无规划小球 / 多余手臂 | 某 group 缺 kinematics solver | kinematics.yaml 补该 group |
| hand state position 偶发 [0,0,0,..] | Modbus 偶发错位 | 已有 3 帧中位数滤波, 不影响判抓 |

## 网络 (每次 AGX 重启需重跑)

```bash
sudo ip route add 192.168.160.161/32 dev enxe466e5820705 src 192.168.160.200
sudo ip route add 192.168.160.162/32 dev enxdc326210a989 src 192.168.160.201
sudo sysctl -w net.ipv4.conf.all.arp_filter=1
```
