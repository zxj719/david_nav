# david_system_env

PLAN §13.5 / P7.1 的共享环境**数据包**。没有节点、环境 hook、自动启动、
常驻进程或安装时系统配置操作。`colcon` 的 setup 不会自动加载此包的域或传输策略。

## 归属与兼容

`config/{ros_env.conf,fastdds_david.xml,load_ros_env.sh}` 从重构分支的
`david_nav/config/` 逐字节迁入；旧三个路径保留相对软链一个版本。
三个文件 SHA-256 在 `test/test_environment_assets.py` 冻结，不是重新生成的金样。
`david_nav` 声明对此数据包的 build/runtime 依赖，CMake 安装完整配置与文档。
旧包安装时解析新包文件真实路径：symlink-install 链接到真源，普通安装保留兼容副本。
不能把source的包间相对软链原样拷到不同的隔离安装前缀。

新代码或工具可以显式引用 `share/david_system_env/config/load_ros_env.sh`。
旧调用方继续用原路径，不需要批量改 `.bashrc`、systemd 或定位脚本。
引用清单及部署边界见 [migration.md](docs/migration.md)。不删除旧路径，
不变更生产工作区，不修改或启停 `3` / Lightning / uslam / 雷达 / IMU。

## 加载契约

调用方必须显式 `source` loader，或以 `set -o allexport` 加载 conf；
conf 仍是 systemd 可读的纯 KEY=value，不能写 shell 展开/引号/export。
外部导出的 `ROS_DOMAIN_ID` 优先，配置默认为 0；loader 原兜底 1 保持。
原 loader 在加载后关闭 allexport 的行为也保持，不在搬迁中夹带修复。
测试只在隔离子 shell 中读配置，不加载 ROS、启动脚本或创建 DDS 参与者。

本次故意保留 conf 内已有的绝对 profiles 路径：
`/home/agx/ros2_ws/src/david_nav/config/fastdds_david.xml`。
**数据已迁移不等于每个运行入口已切到新 XML**；生产路径只读且仍存在，
切换部署/其它主机时需审核该路径。不能因为移动文件而换域、扩展白名单、
取消白名单或偷偷切换到另一份 XML。换机器人应选择自己的环境数据，
而不是把 David 地址、域或旧工作区路径当通用默认值。

当前重构 XML 的有效 UDP 白名单只有 `127.0.0.1`，并显式保留 SHM；
这与文件内历史有线网卡说明不同。本次保持实际内容，不据注释补地址，
也不修改生产 XML 或把 SSH 的 WiFi 网卡加入 DDS。此差异需部署审核。
旧 XML 内的历史“停止 daemon/放开白名单”注释不是本轮操作许可。

## 验证

纯测试覆盖冻结字节、旧相对链接、domain 优先级、变量导出、systemd 格式、
原 DDS 元素与包依赖；样式由 ament_flake8 / Google pep257 检查。
安装后还要核对新旧 share 路径均指向完整资源并加载等价，不启动 ROS。
本包无 robot-profile 参数，schema 为空，配置表由 `navcfg doc` 统一生成。
