# wheeltec_driver

P3.7 / PLAN §8.3 D1：独立的纯 C++ 协议库 `wheeltec_protocol`。
P3.8 / D2 已增加驱动核心与独立节点；未切换生产驱动、未修改正式启动入口。

P3.8 / D2 传输层：`ByteStream` 可注入内存假串口；`SerialStream` 连接仓内 vendored serial，
构造时不打开设备，必须显式调用 open。`StatusReader` 保持旧驱动分包/丢帧规则，一次排空
进入函数时已缓冲的字节、返回最后一帧有效数据。短读保留半帧；I/O 异常丢弃半帧并关闭端口；
若有效帧后发生 I/O 异常，本次不返回样本，与旧实现一致。实例间不共享旧 static 半帧状态。
统计量是累计值，供后续 diagnostics 使用；本层不改变重连时序、写帧或停车策略。

`DriverCore` 保留旧 float 位姿积分、100 次断连 tick 重试、每 12 个有效样本发送电压；
节点的 odom/imu/PowerVoltage 深度分别是 50/20/10，并非频率。消息保留旧 covariance
及 `position.z = yaw`，后者的修正仍需人工审核。9 个旧参数默认值在
`config/wheeltec_driver.yaml`，可用普通 ROS 参数覆盖；话题可 remap。
当前可执行文件仍叫 `wheeltec_robot_node`，节点名仍是 `wheeltec_robot`，后续 D9 独立改名。
构造节点会连接配置的串口，故自动测试禁止运行该硬件入口。析构执行一次尽力零速并关闭；
诊断和发布开关尚属后续 D7/D8，不在此步宣称完成。

D4：硬件入口构造节点后注册弱引用 pre-shutdown 回调，在 ROS context 关闭前停车并记录日志。
rclcpp 的 SIGINT/SIGTERM 退出与节点析构
共用带锁幂等 stop，尝试零速、关闭串口并记录是否确认完整写帧；信号处理器内不执行串口 I/O。
写失败只能报告，不能保证真实硬件已经停稳，固件失联保护仍需现场审核。
独立假串口进程分别向自身发 INT/TERM，检查析构前回调已零速/关口且析构不重复写。

D3：`cmd_timeout_sec` 默认 `0.0`（关闭），保持现有行为。显式设为正值后，启动或最后一次
成功下发有效指令超过该时间，按 20 Hz 尽力发送零速直到新有效指令成功写出；非法指令不续租。
使用 steady clock，不受 ROS / 仿真时钟暂停影响，也不依赖状态帧到来。写失败不忙重试、
executor 延迟后不补发突发帧、串口断开时不由看门狗开设备。默认改为 0.5 s 留待独立现场审核。

`sandbox_tests/run_sandbox_checks.sh` 是单独显式运行的 ROS 集成检查，不注册进 CTest。
检查现场无运行中的用户栈后，加载正式 DDS 白名单并限定 domain 99；节点注入内存串口，
只发布零 Twist，非零协议金样仅直接调用内存回调。无需 IMU/定位启动，不访问真实设备。

`ImuAttitude` 提取原 20 Hz 姿态算法，并修复 Jetson 上已复现的 long/float 越界读取；
数值参考仅修正旧函数的内存访问，详细范围与 sanitizer 验证见 `docs/imu_safety.md`。
不宣称与旧未定义行为逐位等价；生产仍未切换。

下行 11 字节：`7B 00 00 vx_hi vx_lo vy_hi vy_lo wz_hi wz_lo xor 7D`。
三个轴以 1000 倍转换为有符号 int16、大端、向零截断；X/Y 单位 m/s，Z 单位 rad/s。
`encodeCommand()` 只返回数组，不向任何设备写入。

上行 24 字节：帧头、Flag_Stop、三轴速度、三轴加速度、三轴陀螺、电压、前 22 字节 XOR、帧尾。
`decodeStatus()` 先核对帧头/尾/校验和，失败返回 nullopt。速度/电压保留旧驱动的
整数商余数公式及 float 舍入；IMU 保留 float 比例 1671.84f / 0.00026644f 再提升到 double。
电压仍按旧驱动有符号 int16 解释，不擅自改为无符号或修正实测约 4.338 V。

定义旧代码中未定义的非法输入：NaN/Inf、截断后不能装入 int16 的下行值抛 invalid_argument，
不生成部分帧；有效范围内字节和数值保持等价。节点消费该 API 时必须处理拒绝，不能崩溃或
继续发送此前的非零帧。此库不包含速度限制策略。

测试包含观测到的 0.55 rad/s 下行金样、3 帧 P0.8 实录上行、全部 65536 个有符号标量、
单 bit 损坏拒绝、负数/截断边界及 IMU 精度。`colcon test --packages-select wheeltec_driver`
仅运行 gtest、cpplint、uncrustify，不创建 ROS 上下文，不打开串口。

源头许可说明见 NOTICE：旧 Wheeltec 快照 license 仍是 TODO；新协议实现不能替上游确定许可。
