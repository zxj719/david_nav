# wheeltec_driver

P3.7 / PLAN §8.3 D1：独立的纯 C++ 协议库 `wheeltec_protocol`。当前尚无串口节点或 launch，
不改生产驱动；后续 ByteStream/节点迁移归 P3.8。

P3.8 / D2 传输层：`ByteStream` 可注入内存假串口；`SerialStream` 连接仓内 vendored serial，
构造时不打开设备，必须显式调用 open。`StatusReader` 保持旧驱动分包/丢帧规则，一次排空
进入函数时已缓冲的字节、返回最后一帧有效数据。短读保留半帧；I/O 异常丢弃半帧并关闭端口；
若有效帧后发生 I/O 异常，本次不返回样本，与旧实现一致。实例间不共享旧 static 半帧状态。
统计量是累计值，供后续 diagnostics 使用；本层不改变重连时序、写帧或停车策略。

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
