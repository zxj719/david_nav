# ADR-0013：诊断历史记录与当前只读检查边界

状态：保留历史，运行策略不变。

## 背景

按 SPEC §5.4，将 doctor/checks.py 中的三段历史叙述移到这里并保留原文。
来源为提交 c6199da 的 nav_bringup/nav_bringup/doctor/checks.py；
这些是既有代码对历史的描述，不是本轮现场复测或根因证明。

### check_ros_environment

```text
A shell without the DDS whitelist profile sees only part of the ROS graph,
which once looked exactly like "the TF tree is empty".
```

### check_dds_shared_memory

```text
Orphan segments accumulate when ROS processes are killed; enough of them
once broke same-host discovery completely.
```

### check_ros_log_dir

```text
Every ros2 command and every ROS process writes a log file there, and
nothing removes them; on David it once held about 40,000 files.
```

## 决策与验证边界

- 运行代码保留只读检查及原有提示文本，说明写实际判断条件，并引用本记录。
- 环境检查仅看变量与文件存在，不解析 XML 或证明 DDS 图可见性。
- 共享内存检查仅报告观察到的未映射候选项，不能凭结果自动删除或信号进程。
- 日志容量是尽力估算，顶层条目数不等于全树文件数，读取错误可能漏计。
- 本次只改说明与归档，非 doc AST 与 c6199da 一致；三段原文逐段核对。

维护提示不能代替现场授权，也不放宽“不启停或修改 3”的边界。
