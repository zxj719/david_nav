"""保存显式运行时配置值，不依赖 CLI、档案解析器或 ROS 参数 API.

冻结数据类没有构造后校验；类型注解不是运行时验证。加载器、组装工厂和
策略对象分别负责其边界检查，不应把成功构造这些值对象当作配置已通过验收。
"""

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class DockRuntimeConfig:
    """保存显式 Dock 接线和机器人策略，构造器不自动选择 David 调参或验证数值.

    此处 robot_base_frame 是导航旋转中心帧，与 RuntimeConfig.robot_base_frame
    保留的 POI 捕获帧分别配置。核心几何与停稳策略需显式提供；个别字段保留
    声明的默认值。停稳阈值描述采样角速度判断，不等于整个车身的物理静止证明。

    Attributes:
        approach_dist: 目标朝向后方的预备点距离（米）。
        footprint: 导航基准坐标系下的车身多边形顶点（米）。
        global_frame: Dock 目标与位姿比较的全局坐标系。
        max_yaw_retries: 最终偏航补偿的最大次数。
        nav_timeout_sec: 每段导航的等待期限（秒）。
        pose_timeout_sec: 等待所需目标/机器人位姿变换的期限（秒）。
        robot_base_frame: Dock 导航及旋转使用的机器人基准坐标系，不等同于 POI 捕获坐标系。
        settle_max_rate_rad_s: 停稳窗口内相邻样本允许的角速度上限（弧度/秒）。
        settle_timeout_sec: 每次停稳等待的最大时长（秒）。
        settle_window_sec: 连续停稳样本必须覆盖的窗口（秒）。
        spin_action_name: 偏航补偿使用的 Spin 动作名称。
        spin_timeout_sec: 单次 Spin 的时间预算（秒）。
        static_map_topic: 预备点车身轮廓校验使用的静态地图话题。
        static_occupied_threshold: 非负格值达到该阈值即占用；未知格沿用原策略。
        xy_ok: 最终平面位置容差（米）。
        yaw_ok: 最终偏航容差（弧度）。
    """

    robot_base_frame: str
    global_frame: str
    static_map_topic: str
    spin_action_name: str
    footprint: tuple[tuple[float, float], ...]
    approach_dist: float
    xy_ok: float
    yaw_ok: float
    max_yaw_retries: int
    nav_timeout_sec: float
    spin_timeout_sec: float
    settle_max_rate_rad_s: float
    settle_window_sec: float
    settle_timeout_sec: float
    static_occupied_threshold: int = 50
    pose_timeout_sec: float = 2.0


@dataclass(frozen=True)
class DockNodeConfig:
    """保存旧 ROS 入站名称及共享 Dock 运行时配置，不自行执行验证.

    Attributes:
        action_name: 对外提供的 Dock 动作名称。
        goal_topic: 接收 Dock 目标位姿的话题名称。
        runtime: 共享 Dock 技能的完整配置。
    """

    goal_topic: str
    action_name: str
    runtime: DockRuntimeConfig


@dataclass(frozen=True)
class RuntimeConfig:
    """保存由了解机器人配置的外层提供的任务服务组装参数.

    帧、话题、动作和存储路径需显式传入；不猜测机器人档案或修改定位接线。
    可选数值默认值保留旧执行器策略。值对象本身不校验 Dock IDs 与 dock 的组合：
    组装工厂拒绝缺少显式 Dock 配置的 Dock IDs，避免退回普通吸附/重试路径；
    dock 为 None 时工厂不创建 Dock 资源。

    Attributes:
        action_name: 普通导航后端的动作名称。
        cancel_timeout_sec: 取消清理等待期限（秒）；超时不证明资源已释放。
        dock: 显式 Dock 接线及策略；None 不创建 Dock 资源。
        dock_poi_ids: 显式路由到 Dock 技能的 POI 标识集合。
        global_costmap_topic: 用于目标可行性查询的全局代价地图话题。
        grid_map_topic: 用于目标解析的栅格地图话题。
        loop_closure_poll_sec: 闭环重投影检查间隔（秒）。
        loop_closure_translation_m: 触发点位重投影的平移变化阈值（米）。
        loop_closure_yaw_rad: 触发点位重投影的偏航变化阈值（弧度）。
        map_frame: 全局地图坐标系名称。
        nav_goal_clearance_radius: 目标周边净空检查半径（米）。
        nav_goal_fallback_snap_radius: 后备目标搜索半径（米）。
        nav_goal_max_snap_radius: 扩圈搜索半径上限（米）。
        nav_goal_snap_radius: 初始目标搜索半径（米）。
        nav_goal_snap_step: 扩圈搜索半径增量（米）。
        nav_retry_count: FAILED 后额外重试次数，不包含首次尝试。
        nav_retry_delay_sec: 相邻失败重试之间的等待时间（秒）。
        odom_frame: 定位里程计坐标系名称。
        odom_topic: 导航使用的里程计话题；不在此处重接定位端。
        poi_store_file: POI 缓存文件路径；不是 named_poses 的替代真源。
        pose_file: 可选的 named_poses JSON 真源路径；None 不执行该文件导入。
        pose_stale_after_sec: 在线位姿允许的最大年龄（秒）。
        progress_min_interval_sec: 普通导航进度报告的最小间隔（秒）。
        robot_base_frame: POI 位姿捕获的机器人坐标系；与 Dock 导航基准分别配置。
        server_timeout_sec: 等待动作服务就绪的期限（秒）。
        slam_session_id: 标记和筛选点位的定位会话标识。
        source_poi_file: 可选的只读初始 POI YAML 文件路径。
    """

    poi_store_file: Path
    robot_base_frame: str
    odom_topic: str
    map_frame: str
    odom_frame: str
    grid_map_topic: str
    global_costmap_topic: str
    action_name: str
    slam_session_id: str
    pose_file: Path | None = None
    source_poi_file: Path | None = None
    dock_poi_ids: tuple[str, ...] = ()
    server_timeout_sec: float = 10.0
    cancel_timeout_sec: float = 5.0
    nav_retry_count: int = 1
    nav_retry_delay_sec: float = 1.0
    nav_goal_snap_radius: float = 0.5
    nav_goal_fallback_snap_radius: float = 1.0
    nav_goal_clearance_radius: float = 0.1
    nav_goal_max_snap_radius: float = 3.0
    nav_goal_snap_step: float = 0.25
    loop_closure_poll_sec: float = 1.0
    loop_closure_translation_m: float = 0.05
    loop_closure_yaw_rad: float = 0.03
    progress_min_interval_sec: float = 1.0
    pose_stale_after_sec: float = 0.5
    dock: DockRuntimeConfig | None = None
