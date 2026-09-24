# david_config

David 机器人的配置真源，不含运行代码。通用导航包不得写死 David 的帧、话题、
几何或硬件路径。

- `robot/david.yaml`：帧、footprint、速度、底盘、遥控、定位、Nav2、平台和记录。
- `modes/{tour,precise,chassis,teleop}.yaml`：只保存模式差异。
- `modes/overlays/oa.yaml`：把 Lightning 对齐点云直接接到标准 ObstacleLayer。
- `sites/daolan/`：地图、点位和上位机地址。
- `urdf/` 与 `meshes/`：自包含的机器人描述和当前底盘网格。

`base_link`、`base_footprint` 和 POI 采点帧分别由 `frames.body`、
`frames.nav_base`、`frames.poi_capture` 配置，不能合并。

渲染参数：

```bash
share=$(ros2 pkg prefix david_config)/share/david_config
navcfg render --profile $share/robot/david.yaml --mode $share/modes/precise.yaml \
  --output /tmp/nav2_precise.yaml
```

20/24 默认 local/global costmap 均为 `static_layer → inflation_layer`。
21 只在 local costmap 叠加
`static_layer → obstacle_layer(PointCloud2) → inflation_layer`；不存在 LaserScan
转换或自定义扫描层。

验证：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws_refactor
colcon build --symlink-install --packages-up-to david_config
ROS_DOMAIN_ID=99 colcon test --packages-select david_config
colcon test-result --verbose
```
