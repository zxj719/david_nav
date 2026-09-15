# IMU 迁移的内存安全例外

2026-09-15，在 David 的重构 staging 目录，以 GCC AddressSanitizer 运行只含旧 InvSqrt 的
独立程序（没有 ROS 或串口）。观察到 `sizeof(long)=8 sizeof(float)=4`，旧
`i = *((long *)&y)` 报 `stack-buffer-overflow / READ of size 8`，位置在 4 字节局部变量 y。
重现源码留在工作空间 `staging/p38_codex/imu_long_probe.cpp`，不加入正常测试或运行入口。

旧实现既有越界读取也违反严格别名规则，因此无法把该未定义行为输出当作可信数值金样。
新 `ImuAttitude` 用 uint32_t/memcpy 做同一位变换，保留常数 0x5f375a86、一次 Newton 迭代、
20 Hz 固定步长及原 Kp*2=1/Ki*2=0。实例不共享旧全局四元数；无效输入不污染上一个有效姿态。
不改成精确 sqrt，不偷偷校正旧算法的近似归一化误差。

测试参考由 CMake 从不可变 vendor 快照生成：仅替换 InvSqrt 内存访问，保留后续算法原文和
volatile 全局状态；两者都禁用浮点乘加融合，并对参考及新实现启用 AddressSanitizer/UBSan。
测试比较实录传感器重复序列及运动/零加速度混合序列各 3000 步的每个四元数分量。
这是与“修正内存访问后的旧算法”的等价证据，不是对线上未定义行为的等价宣称。

生产驱动没有修改或重启。新解算仍需随驱动在后续人工现场验收中核对 IMU 输出。
源头许可未明确，vendor 派生文件在 package.xml 中明确保留 LicenseRef-Wheeltec-Unspecified，
不能因新包装代码使用 Apache-2.0 就认为 vendor 已被重新授权。
