# david_grasp

David robot vision-based grasping pipeline.
Implements Stage 3-5 of the T2 plan (results/机械臂抓取任务/T2_视觉抓取设计.md).

## Modules (TODO)

| File | Stage | Purpose |
|------|-------|---------|
| `pick_at_pose.py` | 3.a | 手动 pose 开环抓取(arm action + hand service) |
| `force_grasp.py` | 3.b | Q6 力闭环抓取(分步下降+自适应闭合) |
| `vision_pick.py` | 4 | 订阅 B 的 6D pose topic,触发抓取 |
| `grasp_pipeline.py` | 5 | 端到端 + 失败重试 + 触发演示动作 |

## Config

| File | Purpose |
|------|---------|
| `config/grasp_templates.yaml` | 不同物体的 grasp pre-offset + preset |
| `config/handeye.yaml` | T_camera_to_base (B 输出) |
| `config/pick_safety.yaml` | 速度、加速度、reach 边界 |

## Run (TODO)

```bash
ros2 launch david_grasp grasp.launch.py
```

## Dependencies

- david_sdk_bridge (arm action server + hand services)
- david_hand_msgs (SetHandPose.srv)
- david_moveit_config (MoveIt planning)
