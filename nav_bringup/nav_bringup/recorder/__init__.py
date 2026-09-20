"""运行记录器：让每一次导航运行都留下自解释的记录.

各模块（方案 §13.2）：

* :mod:`~nav_bringup.recorder.run_dir`：运行目录布局与 ``latest`` 链接。
* :mod:`~nav_bringup.recorder.manifest`：本次运行的 git 状态、版本、环境与
  输入文件指纹。
* :mod:`~nav_bringup.recorder.events`：结构化的 ``events.jsonl`` 时间线。
* :mod:`~nav_bringup.recorder.resources`：逐进程的 CPU 与内存采样。
* :mod:`~nav_bringup.recorder.summary`：给人看的 ``summary.md``。
* :mod:`~nav_bringup.recorder.retention`：有界的磁盘占用，且绝不丢掉任何一次运行的
  清单与摘要。
"""
