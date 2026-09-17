"""定义任务技能的后端、位姿、存储、目标解析和时钟端口.

Protocol 声明调用约定，不在接口层执行验证、I/O 或资源清理。实现必须
保持单一运动所有权；观测数据、动作终态和清理完成是不同的保证。
"""

from typing import Callable, Iterable, Protocol

from nav_tasks.domain.tasks import DockGoal, NavGoal, NavOutcome, ResolvedGoal, SpinGoal
from nav_tasks.domain.types import CapturedPose, PoseRecord, StoredPoi, Transform2D

FeedbackCallback = Callable[[float], None]


class DockBackend(Protocol):
    """定义共享唯一运动后端的完整 Dock 执行端口，不把预备点到达当作最终成功."""

    async def dock(self, goal: DockGoal, on_feedback: FeedbackCallback) -> NavOutcome:
        """等待本次 Dock 流程完成，不返回中间预备点的到达事件.

        Args:
            goal: Dock 目标及用于关联反馈的任务身份。
            on_feedback: 任务循环中的剩余距离回调，包含后续阶段距离偏移。

        Returns:
            完整流程结果；可能在静态地图拒绝等前置阶段失败，并非总有最终位姿测量。
            按实现契约，返回前应处理自有工作线程及动作收尾。

        Raises:
            asyncio.CancelledError: 显式取消在实现执行其清理约定后传播。
            Exception: 工作线程或动作清理失败，不能当作可继续的普通阶段失败。
        """
        ...


class NavigationBackend(Protocol):
    """定义单次导航尝试，失败重试由外层任务技能决定."""

    async def navigate(self, goal: NavGoal, on_feedback: FeedbackCallback) -> NavOutcome:
        """等待单次导航终态，并在所有者循环交付距离反馈.

        Args:
            goal: 目标 POI 和日志/事件关联身份。
            on_feedback: 剩余距离回调，不应直接从 ROS 回调线程调用。

        Returns:
            后端成功、外部取消或单次失败；后端成功不是独立实测到点证明。

        Raises:
            asyncio.CancelledError: 调用取消后，按实现约定安排在途或迟到目标的清理。
            Exception: 后端执行或清理无法确认；调用方须保留所有者，不能直接开始新目标。
        """
        ...

    async def cancel(self) -> None:
        """取消当前自有目标并确认收尾，涵盖尚未返回的接纳句柄.

        没有自有目标时无操作。正常返回才表示共享动作槽可重新接纳；
        取消应答或期限届满不能代替迟到句柄和终态清理证明。

        Raises:
            asyncio.CancelledError: 等待被取消，不表示底层目标已结束。
            Exception: 清理失败或未能在等待期限内确认，所有者必须继续保留。
        """
        ...


class SpinBackend(Protocol):
    """定义共享导航动作槽的单次相对旋转，不授予第二个运动所有权."""

    async def spin(self, goal: SpinGoal, on_feedback: FeedbackCallback) -> NavOutcome:
        """等待一次 Spin 终态，不套用 Dock 补偿重试或停稳判定.

        Args:
            goal: 有符号相对角度和发给服务端的执行预算。
            on_feedback: 所有者循环中的已转过角度反馈，单位弧度，不是剩余米数。

        Returns:
            后端成功、外部取消或单次失败；成功不独立证明 Dock 最终朝向或停稳。

        Raises:
            asyncio.CancelledError: 调用取消后按实现约定安排清理。
            Exception: 执行或共享动作所有权清理失败。
        """
        ...

    async def cancel(self) -> None:
        """确认共享运动槽的取消收尾，包含迟到的已接纳目标句柄.

        Raises:
            asyncio.CancelledError: 等待被取消，不能据此释放底层所有权。
            Exception: 清理未确认，调用方须保留共享后端并重试。
        """
        ...


class PoseSource(Protocol):
    """定义导航侧定位观测端口，不拥有或控制定位栈."""

    def current_map_pose(self) -> PoseRecord:
        """读取全局坐标系中的机器人位姿，具体时效由观察源和调用方负责.

        Returns:
            最新可取得的全局位姿；取得数据不等于已通过新鲜度或有限性检查。

        Raises:
            PoseUnavailableError: 所需定位数据不可取得。
            ClosedError: 观察源已关闭。
            Exception: 消息字段转换等适配处理失败。
        """
        ...

    async def capture_for_poi(self) -> CapturedPose:
        """取得地图/里程计两种位姿表示及会话标识.

        当前 TF 实现取各自最新样本，不提供时间同步保证；点位管理层另查地图位姿年龄。

        Returns:
            两份位姿观察和会话标识，不应把它解释为同一时刻的同步测量。

        Raises:
            PoseUnavailableError: 任一必需位姿不可取得。
            ClosedError: 观察源已关闭。
            Exception: 观察或会话适配处理失败。
        """
        ...

    def current_map_from_odom(self) -> Transform2D:
        """读取把里程计坐标映射到全局坐标的平面变换.

        Returns:
            用于 POI 重投影的 map-from-odom 变换，不修改定位状态。

        Raises:
            PoseUnavailableError: 所需变换不可取得。
            ClosedError: 观察源已关闭。
            Exception: 变换字段转换失败。
        """
        ...

    def current_slam_session_id(self) -> str:
        """读取定位适配器提供的会话标识，不新建定位会话.

        Returns:
            当前提供的会话字符串，接口本身不验证其唯一性。

        Raises:
            ClosedError: 观察源已关闭。
            Exception: 会话提供者失败。
        """
        ...


class PoiRepository(Protocol):
    """定义点位存储端口，文件实现使用原子替换而非原地覆盖.

    原子文件替换不等于内存/磁盘事务回滚或 fsync 持久化。当前单写入者 YAML
    实现先修改内存再写文件，失败可能留下内存领先磁盘；调用方不能假定回滚。
    """

    def load(self) -> None:
        """加载仓库数据，不修改外部来源地图或自动修复原文件.

        当前 YAML 实现先清空内存，再合并只读来源和本地记录；失败不回滚已合并部分。

        Raises:
            Exception: 读取、解析或记录转换失败。
        """
        ...

    def get(self, poi_id: str) -> StoredPoi | None:
        """按标识查询点位，不创建缺失记录.

        Args:
            poi_id: 点位标识。

        Returns:
            已存点位，缺失时为 None。
        """
        ...

    def upsert(self, poi: StoredPoi) -> StoredPoi:
        """插入或替换点位，并按实现的写入契约保存.

        Args:
            poi: 待存储的领域值。

        Returns:
            仓库返回的已存值，不增加跨进程事务或断电持久性保证。

        Raises:
            Exception: 序列化或写入失败，内存是否已修改由实现决定。
        """
        ...

    def rename_many(self, id_to_name: Iterable[tuple[str, str]]) -> None:
        """更新已有点位的显示名称，不创建未测量的点位.

        Args:
            id_to_name: 按输入顺序处理的点位标识和新名称对。

        Raises:
            Exception: 输入迭代或写入失败；已执行的内存修改不保证回滚。
        """
        ...

    def prune_session(self, session_id: str, keep_ids: set[str]) -> list[str]:
        """只移除指定会话中不在保留集合的点位.

        Args:
            session_id: 本次目录具有管理权的会话标识。
            keep_ids: 该会话中必须保留的点位标识集合。

        Returns:
            已移除标识；当前 YAML 实现保持仓库原有顺序。

        Raises:
            Exception: 持久化失败；当前实现可能已完成内存删除。
        """
        ...

    def reproject_session(
        self, session_id: str, map_from_odom: Transform2D,
        translation_threshold_m: float, yaw_threshold_rad: float,
    ) -> list[StoredPoi]:
        """按指定会话的里程计锚点重投影，不修改其它会话点位.

        当前 YAML 实现保留未折返的 yaw 和原始 yaw 差值阈值，不能擅自改用
        归一化角度比较；只有位置或角度变化超出相应阈值时才更新。

        Args:
            session_id: 要更新的会话标识。
            map_from_odom: 修正后的里程计系到全局系变换，即 map-from-odom。
            translation_threshold_m: 无变化容许的平移阈值，单位米，包含边界。
            yaw_threshold_rad: 无变化容许的原始角度差阈值，单位弧度，包含边界。

        Returns:
            更新后的 POI 列表。

        Raises:
            Exception: 几何运算或持久化失败，已完成的内存修改不保证回滚。
        """
        ...


class GoalResolver(Protocol):
    """定义不移动机器人的锚点/替代点解析端口，不独立证明规划路径可达."""

    def resolve(self, poi: StoredPoi) -> ResolvedGoal | None:
        """按实现策略选取目标，不发送导航动作.

        Args:
            poi: 请求的原始锚点。

        Returns:
            选定的原始或吸附目标；策略找不到合格点时返回 None。
            直接解析器不查占用，旧栅格解析器缺任一地图时允许锚点。
        """
        ...

    def is_admissible(self, poi: StoredPoi) -> bool:
        """按实现策略复查锚点是否可供后续导航，不执行该导航.

        Args:
            poi: 原始请求锚点。

        Returns:
            策略是否允许继续向锚点导航；不是完整路径或车身轮廓安全证明。
        """
        ...


class Clock(Protocol):
    """定义可注入的单调间隔时钟，便于测试避免真实墙钟等待."""

    def now(self) -> float:
        """读取单调时间，不作为来源位姿的时间戳.

        Returns:
            单调时钟秒数。
        """
        ...

    async def sleep(self, seconds: float) -> None:
        """异步让出执行，按间隔时钟等待.

        Args:
            seconds: 等待秒数，调用方应提供合法非负间隔。

        Raises:
            asyncio.CancelledError: 等待被取消。
        """
        ...
