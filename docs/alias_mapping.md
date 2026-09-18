# 重构入口的快捷命令编号

用户 2026-09-17 决定：**现有快捷命令一律不动**，重构后的入口一律用"原编号 + 10"。

| 现有编号 | 现在是什么 | 重构后编号 | 重构入口命令 |
|---|---|---|---|
| `0` | 精准入位导航（`precise_nav_bringup.launch.py`，含 8101 上位机接口） | `10` | `navrun up --mode precise` |
| `4` | 无界面导览导航 | `14` | `navrun up --mode tour` |
| `5` | 底盘（驱动 + 速度适配） | `15` | `navrun up --mode chassis` |
| `5k` | 键盘遥控（PLAN §13.6 里的计划编号） | `15k` | `navrun up --mode teleop` |
| `7` / `8` | 控制器调参台架 | `17` | `navrun up --mode bench_slamgt` |

## 编号冲突：`10` 现在已经被占用

`~/.bashrc` 目前定义了 `1 2 3 0 4 5 6 7 8 9 10`，其中 `10` 是入位测评台架（dock_bench）。
把重构后的 `0` 放到 `10` 会盖掉它。需要用户选一种处理方式：把 dock_bench 换个编号、
重构后的 `0` 改用 `20`，或者确认 dock_bench 可以退役。**确认之前不要添加 `10`。**

## 建议的写法（由用户自己加进 `~/.bashrc`；重构期间我们不改这个文件）

```bash
alias 14='source /opt/ros/humble/setup.bash && source ~/ros2_ws_refactor/install/setup.bash && ros2 run nav_bringup navrun up --mode tour'
alias 15='source /opt/ros/humble/setup.bash && source ~/ros2_ws_refactor/install/setup.bash && ros2 run nav_bringup navrun up --mode chassis'
alias 15k='source /opt/ros/humble/setup.bash && source ~/ros2_ws_refactor/install/setup.bash && ros2 run nav_bringup navrun up --mode teleop'
```

`10`（precise）与 `17`（bench_slamgt）等编号冲突定下来之后再按同样格式添加。

## 添加之前必须确认的两件事

1. **DDS 配置来源**：重构工作区里的 `ros_env.conf` 仍写着生产的 XML 绝对路径，正式部署前
   要先审核到底该加载哪一份，否则两套工作区会互相影响。
2. **实机验收**：上面的命令目前只做过 `--dry-run`——三种模式（precise / tour / chassis）都能
   解析出完整进程图、退出码 0，**从没在实机上真正启动过**。第一次实机启动需要用户在场，
   并按 PLAN §16.3 做冒烟。

## 回退

现有 `0` / `4` / `5` 一个字都没改；重构入口用的是新增编号，出问题直接回去用老编号即可。
