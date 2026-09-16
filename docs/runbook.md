# 导航构建、诊断与验收手册

适用重构工作区，不是生产切换授权。交接包RUNBOOK的S1–S10和现场AGENTS优先。
3由现场手动启动；不修改/启停其uslam/Lightning/雷达/IMU链路。
不发导航目标或非零速度，不打开真实串口，不停止非自有进程/daemon，不清共享内存，
不安装软件、不改权限/alias/系统配置、不push。生产与其它团队工作区只读。

## 每轮检查与构建

先核对主机身份、工作树/HEAD、用户栈与生产/.bashrc指纹。用户栈存在时不启动ROS，
只纯测试和低负载构建。不要因为SSH看不到节点就绕过DDS白名单。
不要打印headless或.bashrc中的凭据。下面构建仅source Humble，不叠生产install：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws_refactor
MAKEFLAGS=-j2 nice -n 19 ionice -c3 colcon build --symlink-install --metas src/colcon.meta --parallel-workers 2
ROS_DOMAIN_ID=99 nice -n 19 ionice -c3 colcon test --parallel-workers 2 --event-handlers console_direct+
colcon test-result --verbose
```

受影响包门禁通过才提交；每轮结束fresh全仓并核对生产指纹。普通测试禁止实际ROS
参与者/驱动/动作请求；shell测试替身sudo/nvpmodel/jetson_clocks/pgrep。
真实ROS配方必须叫sandbox_*_checks.py，不被普通pytest自动收集；域99也不是运动授权。
保留失败日志，不根据SSH中断/超时推断进程终止，不盲目重复启动仍活的验证。

## 环境与只读诊断

仅在允许ROS观察的窗口，同一远端shell先加载以下环境，最后覆盖域99：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws_refactor/install/setup.bash
set -o allexport
source ~/ros2_ws_refactor/src/david_system_env/config/ros_env.conf
set +o allexport
export ROS_DOMAIN_ID=99
navrun --help
navcfg --help
```

此顺序也适用于旧软链环境路径。conf仍指向生产XML，迁移不是传输策略切换；
不要放行SSH使用的WiFi。`navrun doctor`为离线检查；`doctor --live`、monitor、
dump-params、`navcfg check --live`会创建ROS观察者或CLI查询，需另守现场条件。
`navcfg check --live`需已准备run-dir；等待人工定位是提示，不是自行给initialpose。
Nav2实际参数、TF和topic须用正确环境复测，不能只查进程名判健康。

## 可用的离线计划入口

加载重构overlay后，可以只查看计划（不运行driver/platform步骤）：

```bash
navrun up --mode precise --robot david --site daolan --no-executor --dry-run \
  --source-root /home/agx/ros2_ws_refactor/src --repo /home/agx/ros2_ws_refactor/src
navcfg doc --source-root /home/agx/ros2_ws_refactor/src --output /tmp/navigation_parameters.md
```

不带dry-run的up会启动进程并可能做profile规定的主机准备，不是本轮可随手执行的检查。
wrap运行其后命令，也不是只读工具。默认0/4仍由现场旧入口控制，未切alias。

## 运行记录

run目录包含manifest.yaml、code.diff、params/inputs/rendered/effective、events.jsonl、
logs、metrics、bag和summary.md；latest指向最新记录，但不单凭链接判断活进程。
查看前核对run身份/路径/完成状态；不要在活动run上自行执行清理或把旧manifest当活owner。
任务/恢复/Dock/健康事件语义与尾部临时发布成本见 [events.md](events.md)。
原console/legacy脚本日志可能含敏感数据，输出或分享前过滤，不把完整环境广播为事件。

## 明确未完成的接口（继续AUTO，不包装成文档已解决）

- P7.A1：`ls/show` 已接入，只读JSON导出，支持 `--params/--diff/--events`；
  `check` 默认重新校验冻结配置，显式 `--live` 才调用共享ROS只读观测；
  `down/status/gc` 仍待实现，不能用底层helper代替CLI验收。
  `--diff` 是冻结robot→robot+mode的差异，另列Nav2/遥控覆盖，不是运行参数自动回写。
  `unfinished` 不证明进程存活；读目录可能与写入并发，输出明确非原子快照。
- P7.A2：PLAN§12.6的 `navcfg validate --robot <robot> --mode tour` 便利选择器未接入，
  当前可用接口是显式profile/mode/site/URDF/BT文件。
- P3.5：selftest闭环CLI/流程仍待；假底盘/假定位组件存在不等于POI与两段Dock闭环完成。
- P7.4：两历史skip和规范最终收尾未结束；现有全仓绿不足以宣称全部AUTO完成。

## 人工验收与回滚

人工负责安全场地、定位3、生产部署/alias、真实目标/停车/入位、轻量化A/B、
驱动超时/固件/udev/sudoers及legacy清单批准。健康stop和Nav2 odom切换不自动启用。
REVIEW_PENDING和交接记录承载这些问题；IMU/uslam问题不阻塞可自主的导航代码任务。
回退仅针对已确认的重构提交与其构建，不重置生产或删除旧入口。
最后按PLAN逐项核对功能/命令/工件/测试范围，所有AUTO完成且最终连续两轮全仓绿才可结束目标。
