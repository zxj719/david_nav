# David 导航工作树：协作与操作约定

先遵守工作区AGENTS.md及交接包AGENTS/HANDOFF/refactor_loop/RUNBOOK，再读PROGRESS。
本文件是当前导航说明，不是旧生产调参记录，不扩张授权或代替现场安全确认。

## 范围与安全

- 仅在 `~/ros2_ws_refactor/src` 的 `refactor/nav-system` 分支修改并本地提交，不push。
- 生产 `~/ros2_ws/{src,build,install}`、`.bashrc`、`/etc` 和其它团队工作区只读。
- **不修改、启动、停止或重启 `3`**：Lightning/uslam及雷达/IMU由现场管理。
  定位侧问题记录待审核，不以修改3作为继续导航工作的前提。
- 不发导航目标/非零速度，不打开真实串口，不清共享内存，不停止他人进程/daemon。
- 不装软件，不改sudoers/权限配置，不输出或复制脚本中的凭据。
- 现场栈运行时仅纯测试和低负载构建；alias切换、固件、实体运动/停车/性能由用户验收。

## 当前架构

见 [architecture.md](docs/architecture.md)。navrun从配置生成图；任务只有一个owner，
共享Nav2Client/ActionSession；Dock与可选WS在同一组装根。旧executor/Dock是薄入口，
不应再加入第二套任务实现。默认链路为Nav2→可选smoother→chassis_adapter→driver，
话题由档案/模式决定，不是旧separator链。差速底盘不能横移，但可同时有线速度和角速度。
旧separator/探索/console仍有消费者，见 [隔离提案](docs/legacy_isolation.md)。

历史报告的域1、0.50m/s、全局清进程或旧STM32控制模式不作为当前事实。
参数以档案、合并模式、run快照和实际参数为准；未知硬件状态不推断。
`frames.body`、`frames.nav_base`、`frames.poi_capture`不合并；轮式父帧与定位odom独立。
David定位 `/lightning/loc/odometry` 与保留的Nav2 `/lightning/odometry` 是不同键，
TODO-O1未批准切换。默认domain0；SSH走WiFi不意味着DDS允许WiFi。
共享环境迁到david_system_env，旧路径兼容，conf仍保留生产XML绝对路径，部署须审核。
临时诊断的同shell环境和domain99顺序见 [runbook.md](docs/runbook.md)。

## 迭代门禁

1. 检查连接、用户栈、工作树与生产/.bashrc指纹，保留已有用户改动。
2. 核对PLAN/SPEC，改行为先红后绿；搬迁不夹带调参。
3. build只加载Humble；普通测试domain99、低并发；shell测试必须替身危险命令。
4. 受影响包build/test/lint通过才提交；每轮结束fresh全仓验证并核对指纹。
5. 同步PROGRESS、LOG、REVIEW_PENDING、交接和简短日报，保留失败证据。
6. 提交说明改动/原因/任务ID，署名 `Co-Authored-By: Codex <noreply@openai.com>`。

```bash
navrun --help
navcfg --help
```

Python公共API需类型注解和Google文档，flake8行宽99；新C++包保留lint。
纯领域/协议不得依赖ROS/YAML/WS；nav_executor只经nav_tasks.api进入任务层。
取消失败不得假释放任务，清理须等自有线程、回调和子进程真正结束。
`[dock]`日志、协议金样和固定参考不可为测试绿而重生成。
行为见 [behavior_policy.md](docs/behavior_policy.md)，事件见 [events.md](docs/events.md)。
运维CLI、selftest、两历史skip和人工验收不能因现有测试绿被标完成；按逐条审计及最终两轮门禁收尾。
