# example_diffdrive_config

**测试夹具**：为沙箱检查和真实 Nav2 接线测试提供一份虚构机器人配置，作用是让写死在通用代码里的
David 常量暴露出来。它不是受支持的部署目标，也不承诺"换机器人只改配置"（见
[docs/porting.md](../docs/porting.md)）；不连接任何实体底盘。
包内只有机器人档案、两种模式、URDF、自包含地图和示例点位。

相对 David 故意改变所有帧、话题、限速、轮廓、body→nav_base 偏移、底盘和定位类型、地图路径；
关闭 Jetson 操作及清进程。BT 的横向速度为零是此虚构机器人的配置，不改变 David 的既有策略。
公共参数定义复用各模块 `config/schema.yaml`，不复制或覆盖 schema。

`colcon test --packages-select example_diffdrive_config` 自动检查：

- tour/precise 渲染并验证成功；绑定路径与该档案逐项相等，无 David 帧/话题/路径残留。
- 三棵 BT 使用示例配置；URDF 偏移为 0.04 m 而不是 David 的 0.165 m。
- 点云接口可直接读取示例档案，场地地图与点位路径均在包内。
- 安装空间发现与 CLI render/validate；所有输出只写临时目录。
- 公共代码/模板输入前后 SHA256 一致；移植提交只含本包。

这些检查保护的是通用代码不夹带 David 常量；不表示另一台机器人可以部署，也不表示完成了真实 Nav2 闭环或现场验收。
`navrun selftest` 的带目标闭环按 RUNBOOK S1 留待人工安全审核；无人值守测试不发送目标/非零速度。
