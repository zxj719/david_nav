# david_system_env

重构导航唯一使用的 ROS domain 与 Fast DDS 传输配置。该包没有节点、环境 hook、
自动启动或安装时系统配置操作；工作区 setup 不会自动加载它。

启动入口显式加载：

```bash
set -o allexport
source /home/agx/ros2_ws_refactor/install/david_system_env/share/david_system_env/config/ros_env.conf
set +o allexport
```

也可 source 同目录的 `load_ros_env.sh`；外部已设置的 `ROS_DOMAIN_ID` 优先。
`FASTRTPS_DEFAULT_PROFILES_FILE` 指向本包安装的 XML，不依赖旧工作区。

当前 XML 只允许 loopback UDP 并保留 SHM。改变 domain、网卡白名单或传输策略
需要单独的现场审核；自动测试不会创建 DDS 参与者。
