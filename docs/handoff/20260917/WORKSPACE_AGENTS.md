# David 测试与诊断约定

本文件是后续 Codex 在本工作区操作 David 时必须先遵守的现场约定。

## 1. David 是什么

- David 是实验室的实体移动机器人；其板载计算机是 Jetson，运行 Ubuntu 和 ROS 2 Humble。
- 当前远程主机为 `172.16.22.51`，SSH 用户为 `agx`。
- SSH 连接经 WiFi 网卡 `wlP1p1s0` 进入。SSH 所走网卡与 ROS 2 DDS 被允许使用的网卡不是一回事。

## 2. 导航代码位置

- 导航工作空间：`~/ros2_ws`
- 导航模块：`~/ros2_ws/src/david_nav`
- Fast DDS 配置：`~/ros2_ws/src/david_nav/config/fastdds_david.xml`
- 正式 ROS 环境配置：`~/ros2_ws/src/david_nav/config/ros_env.conf`

## 3. 正式导航启动方式

- 正式启动组合是先 `3`、后 `0`，即 `3 + 0`。
- `3` 是 Lightning SLAM/定位栈，由现场人员人工手动开启。Codex 在测试中应先检查其状态；除非用户在当前任务中明确要求，否则不要替用户远程自动启动 `3`。
- 用户于 2026-09-15 明确要求：**不修改任何 `3` 相关代码、配置或启动脚本**，包括 Lightning/uslam 及其雷达、IMU 链路。我们只负责导航侧；定位侧问题记录待审核，不通过修改 `3` 解决。未经用户另行明确要求，不启动、停止或重启 `3`。
- `0` 启动 David 导航栈；当前对应 `david_nav/precise_nav_bringup.launch.py`。
- Profiling 必须区分：基线、仅 `3`、`3 + 0`。除非用户明确要求并确认现场安全，不发送导航目标或运动指令。

## 4. Fast DDS 网卡白名单（重要）

`config/fastdds_david.xml` 的 `interfaceWhiteList` 只放行：

- 回环网卡 `lo`
- 有线内网网卡

它故意不放行 WiFi 网卡 `wlP1p1s0`，即使 SSH 正是通过该 WiFi 网卡连接 David，也不能因此删除或绕过白名单。

该隔离是刻意设计的：实验室另一台机器人 `172.16.20.127` 曾把整套导航栈泄露进 ROS domain 0，与 David 争用 `/map`、`/scan`、`/initialpose`。当前方案是在 domain 0 内使用 Fast DDS 2.6 的 `interfaceWhiteList` 隔离，而不是更换 ROS domain。

正式进程通过 `config/ros_env.conf` 加载 Fast DDS XML。该环境文件需要使用 `set -o allexport`，把其中指向 XML 的 Fast DDS profiles 环境变量导出。

## 5. 临时 ROS 诊断命令的必需环境

任何人工或 Codex 临时执行的 `ros2 topic`、`ros2 node`、`ros2 action`、TF、service、lifecycle 等诊断命令，在运行前都必须先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
set -o allexport
source ~/ros2_ws/src/david_nav/config/ros_env.conf
set +o allexport
```

未加载 `ros_env.conf` 的临时命令会使用默认 DDS 传输配置，与白名单参与者不在同一套传输配置上，因而可能看不见正式进程。此时“无节点、无 topic、无 TF”不能直接判定正式进程异常；必须先用正确环境复测。

通过非交互 SSH 执行诊断时，也必须在同一个远端 shell 中完成上述 `source`，然后再运行 ROS 命令。

## 6. Bench 脚本

- `controller_bench/run_bench.sh`
- `dock_bench/run_dock_bench.sh`

这两个脚本已经按正确方式加载 ROS 环境和 `ros_env.conf`，bench 本身不受上述陷阱影响。不要因为临时诊断命令看不到节点，就修改 bench 的 DDS 配置或移除白名单。
