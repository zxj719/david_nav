# David Navigation Workspace

ROS 2 Humble 导航重构工作树（`refactor/nav-system`）。`~/ros2_ws_refactor` 用于重构，
`~/ros2_ws` 的生产入口未自动切换。定位由现场启动 `3`；本仓只负责导航，不修改或
启停 Lightning/uslam、雷达或 IMU。

## 文档入口

先读 [协作约定](CLAUDE.md) 和 [运行手册](docs/runbook.md)。

- [架构与所有权](docs/architecture.md)
- [参数目录（自动生成）](docs/parameters.md)
- [换机器人](docs/porting.md)
- [行为与安全边界](docs/behavior_policy.md)
- [事件契约与交付限制](docs/events.md)
- [旧代码隔离提案](docs/legacy_isolation.md)

## 模块

| 包 | 当前职责 |
|---|---|
| `nav_bringup` | navrun/navcfg、配置渲染校验、启动图、记录与诊断 |
| `nav_tasks` | 纯领域/技能、唯一任务管理器、共享Nav2客户端、Dock和站点数据 |
| `nav_executor` | WS协议/传输、ROS入站与同进程组装根；旧executor/Dock入口转发到这里 |
| `nav_perception_iface` | 定位契约、可选点云桥、默认warn的5Hz健康观察 |
| `nav_plugins` | costmap/BT插件与行为树模板 |
| `wheeltec_driver`、`chassis_adapter`、`chassis_teleop` | 驱动、速度适配、遥控；不等于已实机部署 |
| `david_config`、`example_diffdrive_config` | 机器人/模式/站点数据；后者为迁移验证样例 |
| `david_system_env` | 显式加载的共享ROS/DDS数据，旧路径兼容，无自动环境hook |
| `david_nav_msgs`、`nav_sim` | 消息契约、仿真假件；闭环selftest仍待实现/验收 |
| `david_nav`、`david_cmd` | 兼容入口、旧功能与调参工具，不能整包删除 |
| `david_slam_gt`、`nav2_rotation_shim_controller`、`third_party/serial` | 保留依赖/补丁，本次不调整行为 |

18个参与构建的包；`third_party/wheeltec_original` 是忽略构建的原始快照。
探索、waypoint和console的隔离仍待逐项审核。

## 构建与检查

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws_refactor
MAKEFLAGS=-j2 nice -n 19 ionice -c3 colcon build --symlink-install --metas src/colcon.meta --parallel-workers 2
ROS_DOMAIN_ID=99 nice -n 19 ionice -c3 colcon test --parallel-workers 2 --event-handlers console_direct+
colcon test-result --verbose
source install/setup.bash
navrun --help
navcfg --help
```

普通测试不应启动ROS或真实驱动。沙箱/现场检查须遵守运行手册，不将测试绿等同于
实体停车、精度或alias部署验收。运维CLI补齐、闭环selftest及最终门禁仍有待办。
