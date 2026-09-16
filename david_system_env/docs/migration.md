# 共享环境数据迁移与引用清单

基线为重构提交 `9608659`，只移动重构工作树文件；生产和其它团队工作区只读。
改动前已用 `git grep -l` 列出本仓引用方（含文件名检查 headless，未输出其秘密行）。

| 引用方 | 保持方式 |
|---|---|
| `david_nav/start_navigation_headless.sh`、`start_david_auto_explore.sh` | 原 loader 路径继续经软链解析，不修改脚本/权限/进程逻辑 |
| `david_nav/navigate_semantic_pose_with_debug_bag.sh`、`record_david_nav_repro_bag.sh` | 原 source/配置路径兼容 |
| `david_nav/scripts/calibrate_plant_gains.sh`、`launch_waypoint_nav.sh`、`nav_to_goal.sh` | 原 source 路径兼容，不执行这些脚本 |
| `david_nav/scripts/controller_bench/run_bench.sh`、`dock_bench/run_dock_bench.sh` | 不改既有加载与 DDS 规则，不启动 bench |
| `david_nav/operator_console/profiles.yaml`、`systemd/{foxglove-bridge,launch-manager}.service` | 仍指原路径；不安装、重载或启动 systemd 服务 |
| `david_nav/test/golden/capture_nav2_param_golden.sh` | 只保留兼容路径，不运行或重录冻结金样 |
| `nav_bringup/nav_bringup/doctor/checks.py`、`nav_bringup/test/test_doctor_checks.py` | 原 profiles/配置识别路径保持 |
| `{nav_bringup,nav_executor,nav_plugins,wheeltec_driver}/sandbox_tests/run_sandbox_checks.sh` | 原环境路径仍可读取，域99覆盖仍由调用方负责 |
| 根 `CLAUDE.md`、`david_nav/CLAUDE.md` | 旧说明中的路径仍有效，全面文档清理属P7.3 |
| `david_nav/config/{load_ros_env.sh,ros_env.conf,fastdds_david.xml}` | 三条旧相对软链；内部文字与绝对profiles值逐字节不变 |

没有宣称已穷举生产systemd或其它工作区/机械臂/定位组的引用。部署前由现场审核
外部调用方、纯KEY=value的绝对profiles路径和旧软链保留期；不改3来迁就导航。
未来删除旧路径、切换外部环境文件或修改白名单需独立决策，不能夹带在纯搬迁提交里。

安装形态注意：隔离安装包前缀不同，不能直接复制source跨包相对链接。
david_nav显式find_package新数据包，旧config安装排除这三个链接，再以新包的
真实文件路径install(FILES)：symlink-install链接到真源，普通安装复制兼容数据。
实际采用 `--symlink-install`，须验证新旧share路径都可读；不能只看source测试绿。
