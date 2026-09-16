# 导航架构与所有权

适用重构分支，不表示生产alias已部署。包概览见 [README](../README.md)，
所有参数名见 [自动参数目录](parameters.md)，现场边界见 [运行手册](runbook.md)。

## 配置、进程与控制数据

```text
robot_config + mode + site + schemas/bindings
  → navcfg纯渲染/校验 → navrun冻结run bundle → nav.launch进程图
  → Nav2 controller/behavior → 可选smoother → chassis_adapter → wheeltec_driver

WS协议/ROS Dock入站 → nav_executor同进程组装根
  → nav_tasks.api → 唯一TaskManager → NavigateToPoi / Dock技能
  → 共享Nav2Client / ActionSession → Nav2动作

外部定位/TF + chassis odom → nav_perception_iface健康观察（仅warn）
任务/Dock/健康/Navrun事件 → /nav/events → 既有monitor → events.jsonl/summary
```

定位提供方不由导航启动；`frames.body`、`nav_base`、`poi_capture`分别配置。
`nav_tasks.domain/core/skills`与`nav_executor.protocol`保持纯逻辑；ROS、YAML、
WebSocket和存储由外层端口适配，import不会启动线程或建立网络连接。
nav_executor业务只导入nav_tasks.api，不导入core私有实现；反向依赖禁止。
`nav_site_data`物理上在nav_tasks包内，不据此把存储I/O放进纯技能。

## 模式与启动

tour、precise、bench_slamgt为导航图；chassis/teleop不依赖地图/定位/健康节点。
precise只有一个任务owner：启用WS时兼容WS和ROS Dock，禁WS时只提供ROS Dock。
tour禁WS时不创建manager。模式/站点/robot选择必须和已冻结bundle一致。
precise+WS管理器属于延后阶段，等Nav2就绪和完整检查后再开放，不能另起独立Dock。
ROS-only precise管理器在core阶段。startup拥有两个受管launch阶段，而不是两套图。

默认非组合，显式composition可把受支持Nav2节点及所选C++ adapter放入isolated容器；
profile驱动、任务管理器、健康观察等仍独立。P5性能收益/实体A/B未由纯测试证明。
进程启动前统一解析可执行文件/组件和配置路径；失败不应留下半套图。
有界driver重启预算、最终关键进程退出、shutdown期间不重启均在监管层处理。

## 任务与关闭

每个任务具有身份/代次，技能工作由supervisor持有。Nav2导航与Spin共用动作槽，
迟到handle须补取消/排空；发包期限、结果期限不等于证明后端已停止。
Dock同步流程在可join的受管worker，动作投递回唯一asyncio loop；
ROS入站需要≥2线程的宿主，不能把阻塞Dock流程放到loop或ROS单线程回调中。

关闭顺序是契约：封入站→关闭/等待WS→请求任务清理（ROS回调仍活着）→
排空任务/worker→退出spin并证明SDK工作池/回调尾部静止→关runtime→
销毁入站/宿主/context。清理失败保留handle/ERROR/busy，允许重试，不伪报IDLE。
详见 [任务架构](../nav_tasks/docs/architecture.md)、
[外层运行时](../nav_executor/docs/outer-runtime.md)、[Dock入站](../nav_tasks/docs/dock-ingress.md)。

## 记录和资源

navrun在运行租约下冻结配置、保存manifest/code.diff/实际参数/事件/指标/摘要。
monitor复用原节点订阅与发布；最终run_stopped晚于原监视器清理，故使用一次受管
临时publisher补发尾部，租约保留直到它退出。该过程有临时开销，不承诺零成本。
健康观察是PLAN规定的独立5Hz节点；其它事件生产者不新增常驻观察线程。
游标、DDS ACK和软件所有权事件不是实体停稳或应用持久处理证明，见 [events](events.md)。

## 兼容与未完成

### 协议、决策与日志接口

线协议定义见 [WS v1/v1.1](protocol/nav_ws_v1.md) 与
[AsyncAPI](protocol/nav_ws_v1.asyncapi.yaml)，当前 codec/golden 保持不变。
冻结的 NAV ERROR、STATUS、OUT sent/RESENT、NAV FAILURE 和全部 Dock 时间线
前缀及消费者列于 [日志接口清单](frozen_logs.md)；尤其 dock_bench 的 `_RE_*`
及判词规则仍依赖旧文字。日志器实际仍是兼容 print/注入 sink，不伪称已切标准 logging。

背景与原注释保留于 [ADR-0001 后台任务](adr/0001-navigation-runs-as-background-task.md)、
[ADR-0002 在途帧](adr/0002-reliable-outbox.md)、
[ADR-0004 普通导航重试](adr/0004-navigation-retry.md)、
[ADR-0007 导入真源](adr/0007-named-poses-source.md)。已有
[扩圈决策](../nav_tasks/docs/adr-0003-goal-resolution.md)、
[ROS future](../nav_tasks/docs/adr-0005-ros-futures.md)、
[Dock worker](../nav_tasks/docs/adr-0006-dock-thread.md) 保留原链接和编号。

业务事件/应答经进程级 outbox；JSON 心跳直接写当前连接，是原行为的明确例外。
所有任务必须有 owner：技能/事件 pump 由 supervisor 持有，连接收/发/心跳由连接作用域
强引用并在退出前 join。禁止无归属的 fire-and-forget，不把“每个 asyncio task 都必须
同一个工厂创建”的概括误写成当前实现事实。

旧executor/Dock入口转到共享root，旧模块保留导出身份，参数/协议/日志golden保留。
环境三资产迁入 [david_system_env](../david_system_env/README.md)，原source路径软链兼容。
旧候选文件只有 [隔离提案](legacy_isolation.md)，没有批准移动或删除。
运维CLI已补齐；自测协调器已实现且四组合无运动启动通过，实际POI/Dock闭环、
完整schema工具校验、剩余规范和现场验收仍独立待办；模块存在不等于目标已完成。
