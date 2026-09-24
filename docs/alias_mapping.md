# 导航快捷入口

重构工作区只保留一套启动管线：

```text
20 / 24 / 21
  → navrun up
  → navcfg 冻结并校验配置
  → nav.launch.py
  → Nav2
  → chassis_adapter
  → wheeltec_driver
```

| 编号 | 模式 | 命令 |
|---|---|---|
| `20` | 精准入位 | `navrun up --mode precise --takeover` |
| `24` | 普通导览 | `navrun up --mode tour --takeover` |
| `21` | 导览 + 动态避障 | `navrun up --mode tour --OA --no-executor --takeover` |

三条入口都从重构安装目录加载 `david_system_env/config/ros_env.conf`；该文件的
Fast DDS XML 也由同一安装包提供，不依赖其它工作区。

原 6/7/8/9 的 SLAM-GT、控制器台架、标定和配套测试已经退役，不再提供别名、
mode、launch 或兼容实现。核心导航控制器仍是 RotationShim→DWB；删除的是独立
DWB bench/标定管线，不是 20/24 的局部控制器。

20/24 的 local/global costmap 默认只有 `static_layer → inflation_layer`。
21 的 OA overlay 只把 local costmap 改为：

```text
/lightning/loc/aligned_scan (PointCloud2)
  → nav2_costmap_2d::ObstacleLayer
  → nav2_costmap_2d::InflationLayer
```

OA 不改变进程图、monitor 或行为树；不存在 PointCloud2→LaserScan 转换和自定义
扫描层。生产工作区与原 0/4 回退入口不属于本仓修改范围。
