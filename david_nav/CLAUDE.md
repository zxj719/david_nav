# david_nav：兼容包维护约定

先读仓库 [协作约定](../CLAUDE.md)、[架构](../docs/architecture.md) 和
[运行手册](../docs/runbook.md)，并遵守现场AGENTS/HANDOFF/RUNBOOK。
这里不再复制整份workspace指南，避免域号、链路和权限说明再次漂移。

此包保留旧launch、安装入口、历史可选功能、调参工具和固定参考。
`david_nav/navigation_executor.py`、`david_nav/dock_navigator.py` 是薄入口，
实际共享组装根在nav_executor；不要恢复第二套manager或修改冻结参考。
新插件在nav_plugins，配置在david_config，环境资产在david_system_env。

不修改或启停3，不改生产/alias，不发目标/非零速度，不全局清进程，不push。
位置环/controller_bench仍保留；探索/console/waypoint只有
[隔离提案](../docs/legacy_isolation.md)，未批准移动。
[历史报告](docs/reports)只证明对应日期的观察，不是当前固件/参数/链路说明。
构建测试不能只覆盖旧包而漏掉新实现；旧入口执行位、安装路径和副本转发都要验证。
参见 [README](README.md)。
