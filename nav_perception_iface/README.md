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
不重定位。帧、话题与原有阈值来自运行目录中冻结的有效档案。

`map→odom` 滞后单独检查，结果是第二个诊断状态 `localization_map_odom_lag`，
与位移一致性的 `localization_health` 互不遮蔽：

- 度量：观测时刻减去 `map→odom` 这一条边最新样本的时间戳，带符号，不取绝对值；
  包含最多一个发布间隔的采样偏移。不再用合成链 `map→base_footprint` 的年龄代替。
- 判定：严格超过 `max_map_odom_lag_sec`（默认 0.30 s）连续 `map_odom_lag_samples`
  次（默认 3 次，约 0.6 s）才报 `map_odom_lag_persistent`；单个尖峰只出现在诊断值里。
  明显未来（早于 `-max_tf_age_sec`）报 `map_odom_future`。
- 事件：进入告警时发一次，之后第一次恢复正常时发 `map_odom_lag_recovered`；
  健康时不发事件。
- 默认值依据现场记录：正常约几十 ms；起过导航栈后 Lightning 可能锁进约 450 ms
  的恒定滞后，只能重启定位恢复。两个参数目前只是节点参数默认值，尚未进入机器人档案。
- 已知未覆盖：Lightning 输出降到 10 Hz、滞后约 156 ms 的劣化在本度量下落在
  0.156–0.256 s，判定它需要发布频率指标，本检查不负责。

## 验证

```sh
colcon build --symlink-install --packages-select nav_perception_iface
ROS_DOMAIN_ID=99 colcon test --packages-select nav_perception_iface
colcon test-result --verbose
```
