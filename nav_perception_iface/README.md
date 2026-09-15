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
这不是运行时健康守护；5 Hz 健康节点仍属于 P6.7。
诊断不会把 Nav2 原 odom 接线改成定位 odom（TODO-O1 保持独立）。

### 独立只读实时检查（P4.5）

加载共享 ROS/DDS 环境后，可运行已安装的独立入口：

```sh
ros2 run nav_perception_iface nav_interface_check \
  --profile /path/run/params/inputs/robot.yaml \
  --mode /path/run/params/inputs/mode.yaml --sample-sec 2 --wait-timeout 10
```

只读取明确指定的档案与可选模式叠加，不选择默认机器人、不修改源文件，
不启动定位/底盘、不发布命令或初始位姿、不调用 action。
默认等待人工定位且每 5 秒提示；正数 `--wait-timeout` 为独立诊断设置期限。
等到 map→odom 可查后才创建 Odometry/可选 PointCloud2 订阅，
以 best-effort、volatile、depth=1 采集当前窗口数据，不把已有发布端点或
旧缓存当作数据流证据。只保留话题到达标记与最新点云头，不保存点云载荷。
点云使用消息时间戳查目标变换；定位链分别检查 odom→body 与 body→nav_base。
底盘命令只数 Twist 类型的订阅端点，不能把同名错误类型当作驱动接入；
检查器本身绝不订阅命令。命令与数据话题重名、
点云与 odom 话题类型冲突以及依赖观察节点私有命名的配置在 ROS 初始化前拒绝。

stdout 为单份 version-one JSON（状态、结论及窗口证据），等待提示在 stderr。
退出码：0=契约满足，1=契约错误，2=配置/运行错误，3=等待定位，
128+signal=中断。INT/TERM/HUP 仅关闭观察者并恢复自身信号处理，绝不停止提供方。
`--no-chassis` 只用于明确不需要底盘契约的独立诊断，不免除定位或启用的感知契约。
空点云和合成 odom 测试仅证明接口与时间戳/TF 检查，不证明定位质量或感知有效性。

`navcfg check --live` 已通过进程入口接入此采样器，不导入本包运行实现，
遵守 PLAN §5.4 的模块边界。窗口证据进入 run events；缺包、超时或无效报告是运行错误。
`navrun up` 在启动导航图之前等待手动定位，图就绪后执行完整检查。
完整 Nav2 导航启动验收仍待后续，合成接口验证不能替代实机验收。

## 验证

```sh
colcon build --symlink-install --packages-select nav_perception_iface david_config david_nav
ROS_DOMAIN_ID=99 colcon test --packages-select nav_perception_iface david_config david_nav
colcon test-result --verbose
```

旧 launch 数值金样固定于 `test/golden/pointcloud_to_scan.yaml`，新映射逐项比较。
另测默认关闭、显式覆盖、错误档案、假图的定位等待/就绪、底盘与点云契约。
新包通过 flake8、Google pep257、AST 文档规范和机器人字面量检查。
