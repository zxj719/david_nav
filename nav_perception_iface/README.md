# nav_perception_iface

导航侧定位与底盘接口契约。该包不实现定位、不发送运动指令，也不提供传感器
预处理 pipeline。

## 只读接口检查

`contracts.check_contracts(profile, snapshot, chassis_required=...)` 是纯判断函数。
它区分“等待 RViz 初始位姿”和定位就绪后的 TF/里程计接线故障，并按需检查
底盘命令订阅端与底盘里程计。`chassis=off` 的调用方传
`chassis_required=False`。

独立检查入口：

```sh
ros2 run nav_perception_iface nav_interface_check \
  --profile /path/run/params/inputs/robot.yaml \
  --mode /path/run/params/inputs/mode.yaml --sample-sec 2 --wait-timeout 10
```

检查器等待 map→odom 可查询后才创建 best-effort、volatile、depth=1 的
Odometry 订阅。它不启动定位或底盘，不发布命令、目标或初始位姿。stdout 为
一份 version-one JSON；退出码 0=契约满足、1=契约错误、2=配置/运行错误、
3=等待定位、128+signal=中断。

## 运行时健康观察

`localization_monitor` 在 tour 和 precise 导航图中以 5 Hz 观察 TF 与轮式里程计，
发布 `/diagnostics` 和 `/nav/events`。当前策略仅告警，不取消任务、不停车、
不重定位。帧、话题和阈值全部来自运行目录中冻结的有效档案。

## 验证

```sh
colcon build --symlink-install --packages-select nav_perception_iface
ROS_DOMAIN_ID=99 colcon test --packages-select nav_perception_iface
colcon test-result --verbose
```
