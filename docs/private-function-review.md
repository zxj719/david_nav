# 短私有函数缺说明审查快照

日期：2026-09-17。依据：执行器 SPEC §5.3。
输入基线：`c34fb5a` 的七个新 Python 源码包。
本记录仅审查**当时没有 docstring、定义跨度不超过十行**的 94 个私有函数；
不覆盖双下划线方法、测试代码、已有说明的内容质量或公共 API 中文覆盖。

## 结论与边界

- 40 个涉及任务/线程边界、几何约定、错误降级、静态分析或格式兼容的函数补充了中文说明，并加入各包按限定名检查说明存在的回归测试。
- 下表 54 个函数逐项阅读后判断为简明取值、转发、条件检查或直接计算，按 SPEC 不因“私有且无说明”本身构成缺口；没有通过删除测试或放宽长函数门禁取得此结论。
- 七包超过十行的私有函数仍由独立门禁检查。短函数的新逻辑或修改仍需评审；本表不是永久豁免名单，也不是新增“所有短函数都必须登记”的工程规则。
- AST 摘要用于精确定位本次判断依据：Python 3.10 下完整函数节点（含装饰器）经 `ast.dump(include_attributes=False)`、UTF-8、SHA-256。行号/注释变化不影响摘要；逻辑、签名或装饰器变化需要重新审查。
- 摘要相同不证明判断必然正确，说明存在不证明文字语义正确。本次不宣称 P7.4、全部 Google 规范或全重构已完成；既有说明的中文、分节与准确性仍待收尾。

## 保留无 docstring 的简单函数

| 源文件 | 限定名 | 本次判断依据 | 函数 AST SHA-256 |
|---|---|---|---|
| [nav_bringup/nav_bringup/doctor/live.py](../nav_bringup/nav_bringup/doctor/live.py) | `_spin` | 仅把已给出的秒数按 g 格式转成命令行文本。 | `55d243f4e5bcd78b0cdabd8a10d325b5301e4f419a88d4dbf31b1bd46d6e7a95` |
| [nav_bringup/nav_bringup/lint/architecture.py](../nav_bringup/nav_bringup/lint/architecture.py) | `_section` | 仅取点分模块名的第二段，缺少该段返回空串。 | `9bcb83008281c8f2a7fce6bdd619c03ff999b279bb78832594b16dffd746d6b5` |
| [nav_bringup/nav_bringup/lint/docstrings.py](../nav_bringup/nav_bringup/lint/docstrings.py) | `_is_public` | 直接按单下划线前缀判断公开命名。 | `b2250d75c94d567e1a455b5b889b1471ca9dd12cb8b59b7651fecb4e738f9561` |
| [nav_bringup/nav_bringup/lint/node_attributes.py](../nav_bringup/nav_bringup/lint/node_attributes.py) | `_ordinary` | 仅排除双下划线名称，已有紧邻注释说明特殊方法与名称改写。 | `657d242b2c738f6113153553ae04933f5bb5f1523d81f7b7bdc039e5d5c30222` |
| [nav_bringup/nav_bringup/navcfg/cli.py](../nav_bringup/nav_bringup/navcfg/cli.py) | `_installed_config` | 显式委托 ament 包查找，再拼接 config 与文件名。 | `02ee191d9bccb09ca800766d9c18226977ea164537709cbbdcc3bb31cbd5cfce` |
| [nav_bringup/nav_bringup/navcfg/render.py](../nav_bringup/nav_bringup/navcfg/render.py) | `_format_value` | 直接委托 deepcopy，没有附加转换或状态。 | `741c670d17100f7e594b313856d7e2d0a6e8ddf0c1c07f0d82d10e7a19c07389` |
| [nav_bringup/nav_bringup/navcfg/validation.py](../nav_bringup/nav_bringup/navcfg/validation.py) | `_identity_rotation` | 直接构造三维单位矩阵常量。 | `e92d6e959679ba06a226e7668c4832beb072afb31c04f7ccdbf27c296954ae40` |
| [nav_bringup/nav_bringup/navcfg/validation.py](../nav_bringup/nav_bringup/navcfg/validation.py) | `_matrix_multiply` | 常见固定三维矩阵乘法，循环维度在表达式中明确。 | `09cbb15c0ce95066f23cf1ec5539ef8ef9ea2c52d21f53f7dc1c0a2753c7f51d` |
| [nav_bringup/nav_bringup/navcfg/validation.py](../nav_bringup/nav_bringup/navcfg/validation.py) | `_matrix_vector` | 常见固定三维矩阵与向量乘法，无状态或 IO。 | `50cd71a7cc11e46f212425f39d7ea90ac19f646ab928c4e8b822482480b9423b` |
| [nav_bringup/nav_bringup/navrun/check.py](../nav_bringup/nav_bringup/navrun/check.py) | `_mapping` | 显式委托 YAML 加载并检查顶层字典，不增加其它检查流程。 | `83ff6dafbb25d2fec76036cb8271413316955b96c6d8909bc000174dbecd2386` |
| [nav_bringup/nav_bringup/navrun/cli.py](../nav_bringup/nav_bringup/navrun/cli.py) | `_as_flag` | 直接为尚无连字符前缀的名字补命令行选项前缀。 | `08ac1bbbfaee6eb728708555989a571d08067556d8016312f8c79bae597d689f` |
| [nav_bringup/nav_bringup/navrun/control_server.py](../nav_bringup/nav_bringup/navrun/control_server.py) | `RunControl._close_client` | 从当前连接表移除给定连接并关闭它，不查找或操作其它连接。 | `944ee575847396bf8ee52a16b1596c4facc4107d3a749bd7e303e32f9889237c` |
| [nav_bringup/nav_bringup/navrun/keyboard_channel.py](../nav_bringup/nav_bringup/navrun/keyboard_channel.py) | `KeyboardChannel._opened` | 直接检查自有 socket 是否已打开，未打开抛明确错误。 | `9fd8f653a0371b3f6b3c17a7c36fdd17f07072f302be5596293e4154acc2ada0` |
| [nav_bringup/nav_bringup/navrun/preflight.py](../nav_bringup/nav_bringup/navrun/preflight.py) | `_matches` | 非空命令对已提供正则逐个 search，结果为布尔值。 | `3ca8c42c8c73546f627e93dbd6eaf0f59851d97f613f8cb14f673fd7765f2235` |
| [nav_bringup/nav_bringup/navrun/preflight.py](../nav_bringup/nav_bringup/navrun/preflight.py) | `_identity` | 直接提取进程身份比较使用的四个字段，无查询或信号。 | `896fdfaf0021666ea40522a76c763b3634d37c123c0a7ec11ef9b11969325853` |
| [nav_bringup/nav_bringup/navrun/preparation.py](../nav_bringup/nav_bringup/navrun/preparation.py) | `_identity` | 从配置取值后显式检查非空字符串，返回原值。 | `f68583a34be42fea8ae301f5d78a4c928d62fced911629a08b9e704d69ecf0f1` |
| [nav_bringup/nav_bringup/navrun/preparation.py](../nav_bringup/nav_bringup/navrun/preparation.py) | `_version` | 直接核 schema_version 的严格整数类型及固定值 1。 | `a5023663cd577cd99cca884a4a8f09e784aa79be5e0534c5452f7550277ac373` |
| [nav_bringup/nav_bringup/navrun/recorded_startup.py](../nav_bringup/nav_bringup/navrun/recorded_startup.py) | `_retry_pause` | 仅等待固定一秒，没有重试状态或资源操作。 | `f478dd668357217ff2fc9ad88c963d762efe4f6f7395b9d3d313982ceee02339` |
| [nav_bringup/nav_bringup/navrun/runtime_host.py](../nav_bringup/nav_bringup/navrun/runtime_host.py) | `_interrupt` | 读取一次停止标记并在非空时抛启动中断。 | `cd77ceba484d637514a763e0819302350bd8c12f1fc3030454650877f5e82d20` |
| [nav_bringup/nav_bringup/navrun/startup_lifecycle.py](../nav_bringup/nav_bringup/navrun/startup_lifecycle.py) | `_timeout` | 显式检查内置数值、有限性与非负条件，错误信息直接说明限制。 | `ee9c39451b0b1ef658a48644f9df2b6a4d5cc619cc7c8bfb9161fa385f786f78` |
| [nav_bringup/nav_bringup/navrun/up.py](../nav_bringup/nav_bringup/navrun/up.py) | `_processes` | 延迟导入并直接委托已有的进程快照函数。 | `416fbcff2cdb4315af5ce171d1ea11ee31781bf95b0e6f43b6628dba3a943e92` |
| [nav_bringup/nav_bringup/recorder/event_flush.py](../nav_bringup/nav_bringup/recorder/event_flush.py) | `_notice` | 直接向 stderr 打印并刷新，无错误恢复策略。 | `5473791095acfe7fa043c552d33516c5b8a40ea5e59865feacb0815f26ae08a6` |
| [nav_bringup/nav_bringup/recorder/linked_process.py](../nav_bringup/nav_bringup/recorder/linked_process.py) | `_working_directory` | 读取指定进程 cwd 链接，任何 OSError 直接转换为 None。 | `2c837f0a4de10d68c611afd92d7243ae30b211b8359103eb1cda1e73ead09b4e` |
| [nav_bringup/nav_bringup/recorder/resources.py](../nav_bringup/nav_bringup/recorder/resources.py) | `ResourceSampler._open_process` | 用已提供工厂打开进程观察对象，指定错误集合转换为 None。 | `3950d05a96f7204d6a764d987461bcc00c7949a5536b4b7bf157e0806dae69e5` |
| [nav_bringup/nav_bringup/recorder/safe_retention.py](../nav_bringup/nav_bringup/recorder/safe_retention.py) | `_identity` | 直接取 stat 的设备号与 inode 二元组。 | `7f467da0a3822f5e08568c05da42aa01feb51513d9b307e0b9b13f125942ca23` |
| [nav_bringup/nav_bringup/recorder/summary.py](../nav_bringup/nav_bringup/recorder/summary.py) | `_mapping` | 保留映射对象，否则返回空字典。 | `1372e7fe36f8622477b7bbc86576e6db1c3a695eba80e1160420fc04285a6297` |
| [nav_bringup/nav_bringup/recorder/summary.py](../nav_bringup/nav_bringup/recorder/summary.py) | `_text` | None 或空文本显示占位符，其余直接 str。 | `587114e7d7f54de3f46b7c7adb171b900d42e57b76c753a5a7b873d99b195313` |
| [nav_bringup/nav_bringup/recorder/summary.py](../nav_bringup/nav_bringup/recorder/summary.py) | `_milliseconds` | 缺失数值显示占位符，其余使用一位小数格式。 | `cd46886a6e36ded939d4bce99f291cdbe4d483022605147b8add2e4c97a6a3cd` |
| [nav_executor/nav_executor/adapters/transport/outbox.py](../nav_executor/nav_executor/adapters/transport/outbox.py) | `ReliableOutbox._check_loop` | 直接比对当前与创建时的事件循环，不同即抛明确错误。 | `c64127aa1374c003ee1b44c416c7d4960ef7ce3f4bc0705c9bdef10b2cbaadec` |
| [nav_executor/nav_executor/app/dock_cli.py](../nav_executor/nav_executor/app/dock_cli.py) | `_default_policy` | 延迟查询默认配置包并拼接唯一 standalone 策略路径。 | `d78147ce2a11437569ad0e1598fdbf861a16888a5fad79614ae88b114068ed33` |
| [nav_tasks/nav_tasks/adapters/ros/action_session.py](../nav_tasks/nav_tasks/adapters/ros/action_session.py) | `ActionSession._check_loop` | 直接比对事件循环身份，不同即抛明确错误。 | `e4711028244fb642a448c79e8161d66ec4627835bb7d57020464bc440697ec68` |
| [nav_tasks/nav_tasks/adapters/ros/action_session.py](../nav_tasks/nav_tasks/adapters/ros/action_session.py) | `ActionSession._on_crash` | 单次错误日志调用，不执行清理或状态转换。 | `a352bef11c4e5f19f8520b35a4e51976e7471af457c30b29723f59f848b0191f` |
| [nav_tasks/nav_tasks/adapters/ros/dock_ingress.py](../nav_tasks/nav_tasks/adapters/ros/dock_ingress.py) | `DockIngress._check_loop` | 直接比对事件循环身份，不同即抛明确错误。 | `b621930d8bc9cbcc5c5326e8b2d95d7307165225485e8c59df7f92e5b196417b` |
| [nav_tasks/nav_tasks/adapters/ros/dock_ingress.py](../nav_tasks/nav_tasks/adapters/ros/dock_ingress.py) | `DockIngress._on_crash` | 单次错误日志调用，无入口状态写入。 | `74b6397ac4f0abca022c56048c680ca597997b09f488839689122b224d84cc4f` |
| [nav_tasks/nav_tasks/adapters/ros/dock_worker.py](../nav_tasks/nav_tasks/adapters/ros/dock_worker.py) | `DockWorker._check_loop` | 直接比对事件循环身份，不同即抛明确错误。 | `06d2a4677978447c159ddae81dfce69b6f110eb4f65e4f2b1a17e979bbbeedbf` |
| [nav_tasks/nav_tasks/adapters/ros/dock_worker.py](../nav_tasks/nav_tasks/adapters/ros/dock_worker.py) | `DockWorker._on_crash` | 单次错误日志调用，无线程取消或资源释放。 | `63749918cb68a55c79e8467fa36d241be5e2498630529c8c7ece4c09c1aa31ee` |
| [nav_tasks/nav_tasks/adapters/ros/grid_source.py](../nav_tasks/nav_tasks/adapters/ros/grid_source.py) | `RosGridSource._retry_close` | 异步签名适配器，直接委托同步 close，关闭细节在该公共方法。 | `a82cf69f0b67ce6e4b942b1a97c5d688f09e85e6e0bfe0e6f15aea4b070ca0d0` |
| [nav_tasks/nav_tasks/adapters/ros/tf_pose_source.py](../nav_tasks/nav_tasks/adapters/ros/tf_pose_source.py) | `_yaw` | 按字段转 float 后委托已命名的 legacy 四元数转换助手。 | `031ea28f9e62c334543542a91d09bd783fca0bf258b13954c7d92d5bad436bbd` |
| [nav_tasks/nav_tasks/adapters/ros/tf_pose_source.py](../nav_tasks/nav_tasks/adapters/ros/tf_pose_source.py) | `_stamp` | 直接将秒和纳秒合成浮点秒。 | `a096dec786595e371ab641c2c71b3f9e5424bb5cdc448afcf1dd720c2d5c8b1b` |
| [nav_tasks/nav_tasks/adapters/ros/tf_pose_source.py](../nav_tasks/nav_tasks/adapters/ros/tf_pose_source.py) | `TfPoseSource._check_open` | 锁内单条件检查 closed，并抛明确关闭错误。 | `3b8a7e0bdf5f294c2a78de6c44e07db6cdbfe6d995720ff3e3feb09019641309` |
| [nav_tasks/nav_tasks/api.py](../nav_tasks/nav_tasks/api.py) | `_runtime_log` | 直接打印固定兼容前缀并刷新，无额外状态。 | `2038f77bf3bf3f25a9317d68b559b29ff2ea3dc4903be77bc533515c959f0615` |
| [nav_tasks/nav_tasks/app/logging.py](../nav_tasks/nav_tasks/app/logging.py) | `_print_line` | 直接打印调用方给定行并刷新。 | `603ab985b61d5a815f427e7e1e81c9bcb147b1dfcb3ac4e134c62b9304e828b6` |
| [nav_tasks/nav_tasks/app/runtime.py](../nav_tasks/nav_tasks/app/runtime.py) | `TaskRuntime._check_loop` | 直接比对事件循环身份，不同即抛明确错误。 | `290fc04160e7534ed2d6fd0f56586f611d8acfbb9cec2c009a5e198ed950a793` |
| [nav_tasks/nav_tasks/app/runtime.py](../nav_tasks/nav_tasks/app/runtime.py) | `TaskRuntime._on_crash` | 单次错误日志调用，无清理或状态转移。 | `5f632c91de130c266e713d6ddd16ce02b823669fa57ec58ff2f08a4fea5120a1` |
| [nav_tasks/nav_tasks/core/goal_resolution.py](../nav_tasks/nav_tasks/core/goal_resolution.py) | `_distance_xy` | 直接计算两个二维点的欧氏距离。 | `bae51af9028d1a9a5b8ee962b8bc4a547c2013c60213757ee85c1da0ea016d2a` |
| [nav_tasks/nav_tasks/core/state.py](../nav_tasks/nav_tasks/core/state.py) | `TaskManager._check_loop` | 直接比对事件循环身份，不同即抛明确错误。 | `791e97b250402d6ec9ea4a24c039bd1e2988a82875bbd3a48a201a3699697fe7` |
| [nav_tasks/nav_tasks/core/state.py](../nav_tasks/nav_tasks/core/state.py) | `TaskManager._check_open` | 单条件检查 closing，并抛明确关闭错误。 | `f14cd9d5aca84c4a565a338a81485dce2fd9b0bf7f3bb2736815621f0712ed2d` |
| [nav_tasks/nav_tasks/nav2_client.py](../nav_tasks/nav_tasks/nav2_client.py) | `Nav2Client._describe_goal` | 仅按现有模板与两位小数格式化目标日志，不解算或提交目标。 | `7546707aaf9640bd2f374ca0b1b3af319824d2a5fc1ec1bc57ca192b968c42b4` |
| [nav_tasks/nav_tasks/nav2_client.py](../nav_tasks/nav_tasks/nav2_client.py) | `Nav2Client._build_goal` | 直接将 POI 的地图位姿委托给已有 _build_pose。 | `da59ac247e4b36c60809d6d30a0485384ba1ee11fcf6321020c5a7fb467a721d` |
| [nav_sim/nav_sim/selftest_ros.py](../nav_sim/nav_sim/selftest_ros.py) | `Observer._seconds` | 直接将消息秒和纳秒换算为秒。 | `6fcb712ecf98a2d2dffef45a432b5a4e4260ba79675e0fec154e7f220d6b677d` |
| [nav_perception_iface/nav_perception_iface/health_core.py](../nav_perception_iface/nav_perception_iface/health_core.py) | `_finite` | 直接检查严格 int/float 类型与有限性，没有隐藏回退。 | `744c47569f4c5db7d5bad33e18d91cdeabd49701724d8db0805a473ff8c5dc52` |
| [nav_perception_iface/nav_perception_iface/health_node.py](../nav_perception_iface/nav_perception_iface/health_node.py) | `_stamp` | 直接把秒和纳秒字段转为浮点秒。 | `a5b0fad92bacee4e57e054ca97f3bc61e7e55f4ef4083ad110d1242cafead0e9` |
| [chassis_adapter/chassis_adapter/core.py](../chassis_adapter/chassis_adapter/core.py) | `_clamp` | 取限值绝对值后直接作对称裁剪，无死区、计时或状态。 | `88698948074c7d8de2a1e19d0466e76ab4164a54b048a2e939b69722b7c19e22` |
| [chassis_adapter/chassis_adapter/node.py](../chassis_adapter/chassis_adapter/node.py) | `AdapterNode._now` | 直接读取节点当前 ROS 时钟纳秒值。 | `03aad028c9ef5efadb288e06a4847e0d0fe715936865e8be7705f0133fe7603e` |

## 后续维护

修改下表函数时，以新实现重新判断是否出现非显而易见逻辑；需要时补说明。
审查者也可直接补有价值的说明，不需为了维持数量而保留空白。
本轮运行语句、参数、冻结日志和 golden 均不改；没有启停定位、发送运动指令或执行真实记录清理。
