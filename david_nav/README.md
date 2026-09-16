# david_nav

导航重构中的兼容与历史功能包。总览见 [workspace README](../README.md)。
生产仍在原工作区，重构改动不等于已切换alias 0/4；只构建本包不足以构建全部新模块。

## 当前归属

| 保留路径/功能 | 实现或用途 |
|---|---|
| `david_nav/navigation_executor.py` | nav_executor.app.cli薄入口，保留路径/执行位/CLI |
| `david_nav/dock_navigator.py` | 同一组装根的Dock入口，完整配置来自david_config |
| `launch/precise_nav_bringup.launch.py` | WS和Dock共享owner，不另起两个manager |
| 领域、POI、地图、Dock旧模块 | 兼容导出/测试，实际归属nav_tasks或nav_executor |
| 新costmap/BT插件 | nav_plugins；本包保留旧描述/资源兼容 |
| `config/{ros_env.conf,load_ros_env.sh,fastdds_david.xml}` | david_system_env的source软链，安装时解析真源 |
| 位置环、controller_bench、dock_bench | 仍在用，不得因旧代码清理删除 |
| frontier、separator、warmup、console、waypoint/航位推算 | 旧可选/调试功能，隔离待审核 |

旧2d launch仍有warmup和collision_monitor条件入口，默认关闭不等于无引用。
探索不是新navrun图的默认功能，逐文件清单见 [legacy_isolation.md](../docs/legacy_isolation.md)。

## 行为边界

精准+WS的Dock在Nav2就绪与完整检查后一起开放；无WS精准模式仍提供本地Dock。
不要独立混启旧Dock和新manager。静态完整footprint预备点检查、恢复后重发跟踪、
0.05rad/s停稳阈值已在重构分支，实体停车/精度/部署仍需现场验收。
详见 [behavior_policy.md](../docs/behavior_policy.md)。
定位由现场3提供，不改uslam/IMU；Nav2 odom按TODO-O1保留，点云桥默认关闭，
不能宣称默认具备有效动态避障输入。配置以david_config和run实际参数为准。

## 开发与历史资料

安全构建/验证见 [runbook.md](../docs/runbook.md)。旧协议/参数金样、Dock日志、
兼容入口、frontier C++与算法测试保留，不删测试或重造金样掩盖差异。
`test/golden/legacy_entries`是固定文本，不是运行入口；测试总数以本次colcon结果为准。

专题：[位置环](docs/position_controller_README.md)、[waypoint](docs/waypoint_follower_README.md)、
[控制器台架](scripts/controller_bench/README.md)、[入位台架](scripts/dock_bench/README.md)、
[历史报告](docs/reports)。历史资料不扩张自动化代理的实机操作授权。
