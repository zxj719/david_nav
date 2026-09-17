# 已有私有函数分节审查（2026-09-17）

基线为 `601c402`。本次审查针对已有说明被结构规则标记的31个函数，
不是全仓语义审查完成证明；既有中文/英文说明仍需按 SPEC §5.3 继续检查。

## 实际修订

- nav_bringup 的20个函数：lint AST 两个遍历器；BT 标量；navcfg CLI 五个入口帮助函数；live_cli 的一次观察闭包；validation 十一个规则/变换函数。
- nav_tasks 的七个函数：footprint 六个几何帮助函数以及 Dock 跨线程派发。
- 逐项补充参数、单位、坐标系、返回状态、原地修改与异常边界。地图局部坐标仍为米，不是栅格下标；底盘最小原地旋转速度不是 Dock 停稳阈值。
- Dock 的 BusyError 经 answer Future 转给工作线程；真正可能从回调抛出的 InvalidStateError 是写回已结束/取消 Future 失败。二者不可混为一谈。
- 未更改运行语句、AST 规则或既有测试断言。模块说明不变，包括被 CLI 直接用作 help 的说明。
- 两个 test_reviewed_doc_sections.py 共27项按限定名保护已审函数的中文说明、必要分节和参数名。它们只是结构回归，不证明自然语言语义。

## 仍被粗略 Raises 规则标记的五处

_check_function 只看自身语法作用域有没有显式 raise，不分析捕获路径。
以下均已阅读完整函数及相关边界；不为使这份清单归零而虚构对外异常接口，
也不新增永久豁免或削弱公共 API 门禁。实现变化后应重新审查。

| 文件 / 函数 | 实际边界 | 为什么不增加对应的 Raises |
|---|---|---|
| nav_bringup/nav_bringup/navcfg/cli.py · _validate_command | 显式 RenderError 在同一 try 的 except 中打印，返回2；规则问题返回0或1。 | 正常配置错误不是由此函数向调用者抛出；说明已写明处理方式。 |
| nav_executor/nav_executor/app/idle_profile.py · _collect | 采样循环的 ValueError、OSError 转成 reason；KeyboardInterrupt 也转换。 | 既有说明已写清不完整结果及其它注入函数异常透传；不能把捕获的 ValueError 声称为对外异常。循环外计算仍假定合法时长、间隔和 tick_hz。 |
| nav_tasks/nav_tasks/adapters/ros/event_publisher.py · NavigationEvents._publish | 构造阶段显式 ValueError 在本地捕获并告警；两路 publish 各自捕获 Exception。 | 不声明数据或投递失败一定向外抛出；不承诺日志设施、锁或 BaseException 永远不失败。 |
| nav_sim/nav_sim/selftest_ros.py · Observer._event | payload 限额等 Exception 被转成 self.error。 | 后续 step 根据 error 抛 RuntimeError，不能把 step 的失败归为回调直接抛出。 |
| nav_perception_iface/nav_perception_iface/health_node.py · LocalizationMonitor._odometry | 消息转换和显式帧检查的 Exception 被捕获，清空旧轮式样本。 | 这里是导航侧只读观察回调；不宣称其直接抛出帧错误，也不改变定位链路。 |

## 尚未完成

这五条只是对本次启发式结果的解释，不代表所有异常路径、所有私有函数或所有语言规范都已满足。
后续继续检查已有说明的中文与语义质量。翻译模块说明前须识别 __doc__ 消费者，
保留现行 CLI help 兼容；仅比较移除说明后的 AST 不足以证明 help 行为不变。
