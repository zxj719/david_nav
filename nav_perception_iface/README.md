# nav_perception_iface

导航侧定位与感知接口（PLAN §6 / P3.3）。不实现定位，不发送运动指令。
默认扫描桥关闭，没有常驻进程增加；开启时只起原有
`pointcloud_to_laserscan_node`，保留原节点名和全部参数。

## 配置与启动

`sensors.scan_from_cloud.*` 和 `frames.scan_target` 来自有效机器人档案。
`navrun` 可传运行目录中的合并档案；旧 David launch 传 David 档案并用
`enabled` 参数保持 `enable_scan_bridge` 的原语义。该包没有默认机器人路径。

```sh
ros2 launch nav_perception_iface pointcloud_to_scan.launch.py profile_file:=/path/effective.yaml
```

`enabled:=true|false` 可覆盖档案；默认 `profile`。此命令可能发布 LaserScan，
只能在现场允许时执行。自动测试只构造 launch 图，不运行传感器或底盘。

## 接口契约

`contracts.check_contracts(profile, snapshot, chassis_required=...)` 是纯判断函数。
外层注入 TF 查询结果、端点数量、类型和采样窗口数据；本包不会自行启动定位。
地图到里程计 TF 未就绪时输出 `localization_waiting`，暂不判断定位 odom 缺失。
就绪后检查里程计→机身→导航基准链和定位 odom 实际消息；按需检查底盘订阅/odom、
点云类型及其目标帧变换。`chassis=off` 的调用方须传 `chassis_required=False`。
这不是运行时健康守护；在线采样/等待提示属于 P4.5，5 Hz 健康节点属于 P6.7。
诊断不会把 Nav2 原 odom 接线改成定位 odom（TODO-O1 保持独立）。

## 验证

```sh
colcon build --symlink-install --packages-select nav_perception_iface david_config david_nav
ROS_DOMAIN_ID=99 colcon test --packages-select nav_perception_iface david_config david_nav
colcon test-result --verbose
```

旧 launch 数值金样固定于 `test/golden/pointcloud_to_scan.yaml`，新映射逐项比较。
另测默认关闭、显式覆盖、错误档案、假图的定位等待/就绪、底盘与点云契约。
新包通过 flake8、Google pep257、AST 文档规范和机器人字面量检查。
