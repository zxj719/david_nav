# 换机器人：只改配置的工作流

目标是新增 `<robot>_config` 而非在通用模块写机器人分支。
[example_diffdrive_config](../example_diffdrive_config/README.md) 提供独立帧/话题的样例，
纯配置迁移测试不证明另一台实体机器人已可用。

## 1. 数据清单

复制配置包结构并修改package/resource/install身份，逐项填写：

- `robot/<robot>.yaml`：body/nav_base/poi_capture/map/odom帧、完整footprint、运动硬限。
- 定位实际odom、Nav2订阅odom、底盘odom话题及header父帧分别配置；不要猜它们相同。
- 驱动package/executable/参数、adapter输入/输出/超时、teleop话题与限速、平台策略。
- executor的地图、可写POI存储、POI采点帧；精准模式完整Dock参数与入站名称。
- localization_health四字段和chassis.odom_frame必须显式提供，当前action仅warn。
- `modes/`叠加、URDF及资源、`sites/<site>/site.yaml`、trinary地图、只读点位文件与server。
- ROS/DDS环境数据须适合该机器；David的绝对profiles路径和网卡不能作为通用默认。

改模式应只写差异；绑定值以profile为真源，不在nav2_overrides中重复改绑定目标。
所有消费键见 [parameters.md](parameters.md)。不要复制派生快照作为新的参数真源。

## 2. 先离线渲染与校验

优先用PLAN的机器人选择器，在已加载重构安装环境后运行：

```bash
navcfg validate --robot example_diffdrive --mode tour
navcfg validate --robot example_diffdrive --mode precise --site offline --source-root ./src
```

只读复用up的配置发现、叠加、Nav2/BT渲染与完整离线校验，不调用up的主机规划。
核机器人/模式/场地身份、schema、帧/限速、URDF、地图与所选点位资产；不启动ROS、
不扫描进程、不写运行记录。默认从安装包找资源，`--source-root`改选源码包。
导航场地只有一个时自动选择，否则必须`--site`；chassis/teleop不依赖场地/地图/URDF。
机器人须提供所选mode文件；样例提供tour/precise/chassis/teleop四种，不含bench_slamgt。
成功不证明可执行文件可用、在线话题或硬件通过；并发编辑时也不是原子配置快照。

原显式文件接口保留，以下在重构工作空间根目录运行，只写指定临时输出：

```bash
navcfg render --profile src/example_diffdrive_config/robot/example_diffdrive.yaml \
  --mode src/example_diffdrive_config/modes/tour.yaml \
  --base src/nav_bringup/config/nav2_base.yaml \
  --bindings src/nav_bringup/config/nav2_bindings.yaml \
  --output /tmp/example_nav2.yaml
navcfg --help
```

显式`--profile` validate仍必须给mode、site、URDF及已渲染BT目录，不能仅校验YAML可读。
使用 `navcfg validate --help` 获取参数；不要混用`--robot`与显式profile/URDF/BT/base/bindings。
新选择器返回0=离线通过、1=配置不合格、2=选择器/资源/schema错误；warning不失败。
`navrun up --dry-run --source-root ...`可验证选定图、绑定与可执行解析，不启动驱动。

## 3. 安装与行为门禁

构建新配置包后核对资源索引、source/installed路径和所有引用的文件；
以两种不同机器人档案测试同一套渲染/纯技能，检查通用代码没有新机器人字面量。
对比 `git diff --stat` 应只改变新配置包（基础通用缺陷须单独提交说明）。
不要修改David冻结金样来迎合新机器人。

PLAN要求的 `navrun selftest --robot ... --mode tour|precise` 闭环尚属P3.5：
仿真假件已存在不代表该CLI/完整闭环已完成。实现后应隔离真实驱动、域和可写状态，
实跑导航目标仍受当前现场规则约束，不因为称为仿真就擅自发布运动请求。

## 4. 现场验收与回滚

定位/底盘提供方由现场确认，导航只做加载正确DDS环境的接口检查；不改3来适配。
现场逐项检查TF、消息新鲜度、参数实际值、制动、转向与Dock，再决定部署。
先验证独立环境文件，再改用户入口；保留原alias与旧配置，不删除旧工作区。
实时检查方法、人工边界及当前未完成接口见 [runbook.md](runbook.md)。
