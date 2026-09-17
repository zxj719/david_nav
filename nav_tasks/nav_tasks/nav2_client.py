"""共享单次导航与可选旋转后端，用同一会话持有动作所有权.

模块导入不启动 ROS；构造未注入的真实客户端时才导入 ROS 类型。
运行时供多个技能共享一个实例；重试与反馈节流由核心技能负责。
"""

import math
from typing import Callable

from nav_tasks.adapters.ros.action_session import (
    ActionClientLike, ActionOutcome, ActionSession, ActionState)
from nav_tasks.core.ports import Clock, FeedbackCallback
from nav_tasks.core.retry import MonotonicClock
from nav_tasks.domain.errors import ConfigError, RuntimeAssemblyError
from nav_tasks.domain.tasks import NavGoal, NavOutcome, NavStatus, SpinGoal
from nav_tasks.domain.types import PoseRecord

_STATUS_NAMES = {
    0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
    4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED',
}


class Nav2Client:
    """把导航和相对旋转目标转换成共享单槽位中的单次动作尝试."""

    def __init__(
        self, node: object, action_name: str, server_timeout_sec: float,
        *, client: ActionClientLike | None = None,
        goal_factory: Callable[[], object] | None = None,
        session: ActionSession | None = None, clock: Clock | None = None,
        cancel_timeout_sec: float = 5.0,
        spin_action_name: str | None = None, spin_server_timeout_sec: float = 5.0,
        spin_client: ActionClientLike | None = None,
        spin_goal_factory: Callable[[], object] | None = None,
    ) -> None:
        """注入现有节点及可选测试客户端，并记录本实例创建的资源.

        节点与 ROS executor 由外层运行时持有。客户端和消息工厂必须成对提供；
        仅导航注入而 Spin 未注入时仍会创建真实旋转客户端。构造失败只回滚自建
        客户端，不关闭借用的会话；回滚失败保留可重试清理句柄。

        Args:
            node: 外层持有并驱动回调的既有节点。
            action_name: 导航端点，要求非空；不在这里进行严格字符串校验。
            server_timeout_sec: 导航服务就绪预算，有限且非负。
            client: 可选注入客户端，其销毁仍由注入方负责。
            goal_factory: 与注入客户端配对的消息工厂。
            session: 可选共享会话；省略时在当前事件循环创建。
            clock: 单调时钟，用于服务就绪与尝试耗时。
            cancel_timeout_sec: 仅新建会话时采用的清理等待预算。
            spin_action_name: 可选旋转端点；None 表示禁用。
            spin_server_timeout_sec: 独立旋转就绪预算，即使禁用也校验。
            spin_client: 可选旋转客户端，需显式配置旋转端点。
            spin_goal_factory: 与注入旋转客户端配对的工厂。

        Raises:
            ConfigError: 已检查的配置或注入配对不满足要求。
            RuntimeAssemblyError: 构造回滚失败，需保留并重试附带清理。
            Exception: 客户端构造或未归一化的类型校验异常。
        """
        if not action_name:
            raise ConfigError('action_name must be nonempty')
        if not math.isfinite(server_timeout_sec) or server_timeout_sec < 0:
            raise ConfigError('server_timeout_sec must be finite and nonnegative')
        if (client is None) != (goal_factory is None):
            raise ConfigError('client and goal_factory must be injected together')
        if (spin_client is None) != (spin_goal_factory is None):
            raise ConfigError('spin_client and spin_goal_factory must be injected together')
        if spin_action_name is not None and not spin_action_name:
            raise ConfigError('spin_action_name must be nonempty when configured')
        if spin_client is not None and spin_action_name is None:
            raise ConfigError('an injected spin client requires spin_action_name')
        if not math.isfinite(spin_server_timeout_sec) or spin_server_timeout_sec < 0:
            raise ConfigError('spin_server_timeout_sec must be finite and nonnegative')
        self._node = node
        self._action_name = action_name
        self._server_timeout = server_timeout_sec
        self._clock = clock if clock is not None else MonotonicClock()
        self._session = session if session is not None else ActionSession(
            cancel_timeout_sec=cancel_timeout_sec, clock=self._clock)
        self._owned_clients: list[object] = []
        self._spin_action_name = spin_action_name
        self._spin_server_timeout = spin_server_timeout_sec
        try:
            if client is None:
                from nav2_msgs.action import NavigateToPose
                from rclpy.action import ActionClient

                client = ActionClient(node, NavigateToPose, action_name)
                self._owned_clients.append(client)
                goal_factory = NavigateToPose.Goal
            if spin_action_name is not None and spin_client is None:
                from nav2_msgs.action import Spin
                from rclpy.action import ActionClient

                spin_client = ActionClient(node, Spin, spin_action_name)
                self._owned_clients.append(spin_client)
                spin_goal_factory = Spin.Goal
        except Exception as error:
            try:
                self._destroy_owned_clients()
            except Exception:
                raise RuntimeAssemblyError(error, self._rollback_clients) from error
            raise
        self._client = client
        self._goal_factory = goal_factory
        self._spin_client = spin_client
        self._spin_goal_factory = spin_goal_factory

    async def navigate(self, goal: NavGoal, on_feedback: FeedbackCallback) -> NavOutcome:
        """执行一次导航并保留兼容的详细失败说明.

        状态 4/5/6 映射为成功、取消、失败；软件终态不证明实车到位。
        这里只设置就绪预算，不隐含发送或结果期限，也不重试或执行停稳校验。

        Args:
            goal: 已解析的领域目标与关联字段；此处不做 TF 转换。
            on_feedback: 事件循环线程回调，距离单位为米；节流由技能负责。

        Returns:
            单次成功、外部取消或可读失败结果。

        Raises:
            asyncio.CancelledError: 请求取消并尝试清理后传播；清理异常可能替代它。
            Exception: 源动作或所有权清理失败，不可据此继续下发目标。
        """
        started = self._clock.now()
        remaining_distance: float | None = None

        def feedback(raw: object) -> None:
            nonlocal remaining_distance
            remaining_distance = float(getattr(raw.feedback, 'distance_remaining', 0.0))
            on_feedback(remaining_distance)

        result = await self._session.execute(
            self._client, lambda: self._build_goal(goal), feedback,
            server_timeout_sec=self._server_timeout)
        description = self._describe_goal(goal)
        if result.state == ActionState.UNAVAILABLE:
            return NavOutcome(NavStatus.FAILED, (
                f"Nav2 action server '{self._action_name}' 在 "
                f'{self._server_timeout:.1f}s 内不可用(未起来或 lifecycle 未 Activate), '
                f'{description}'))
        if result.state == ActionState.REJECTED:
            return NavOutcome(NavStatus.FAILED, (
                'Nav2 拒绝 goal(bt_navigator rejected;常见于 TF 未就绪/上一个 '
                f'goal 未清), {description}'))
        status = result.response.status
        if status == 4:
            return NavOutcome(NavStatus.SUCCEEDED)
        if status == 5:
            return NavOutcome(NavStatus.CANCELED)
        elapsed = self._clock.now() - started
        remaining = (
            f'{remaining_distance:.2f}m' if remaining_distance is not None
            else 'n/a(无 feedback,可能 planner 直接失败)')
        return NavOutcome(NavStatus.FAILED, (
            f'Nav2 终态 status={status}({_STATUS_NAMES.get(status, "?")}), '
            f'{description}, 剩余距离={remaining}, 本次耗时={elapsed:.1f}s'))

    async def spin(self, goal: SpinGoal, on_feedback: FeedbackCallback) -> NavOutcome:
        """在导航共享会话内执行一次有符号的相对旋转.

        角度不归一化或裁剪；发送服务端 allowance，但不附加本地结果期限、
        重试或停稳检查。这些策略由 Dock 显式提供。

        Args:
            goal: 弧度角与非负、可由 ROS 表示的时间额度。
            on_feedback: 循环线程上的已转角反馈，单位为弧度而非剩余米数。

        Returns:
            单次终态或可读的接收失败。

        Raises:
            ConfigError: 旋转禁用，或已检查的目标字段不可表示。
            asyncio.CancelledError: 调用方取消；会话仍持有未完成清理。
            Exception: 数值转换、动作或清理异常；超大整数也可能触发溢出。
        """
        def feedback(raw):
            on_feedback(float(raw.feedback.angular_distance_traveled))

        build = self._spin_factory(goal)
        result = await self._session.execute(
            self._spin_client, build, feedback,
            server_timeout_sec=self._spin_server_timeout)
        if result.state == ActionState.UNAVAILABLE:
            return NavOutcome(NavStatus.FAILED, (
                f"Spin action server '{self._spin_action_name}' 在 "
                f'{self._spin_server_timeout:.1f}s 内不可用'))
        if result.state == ActionState.REJECTED:
            return NavOutcome(NavStatus.FAILED, 'Spin 目标被拒绝')
        status = result.response.status
        if status == 4:
            return NavOutcome(NavStatus.SUCCEEDED)
        if status == 5:
            return NavOutcome(NavStatus.CANCELED)
        return NavOutcome(NavStatus.FAILED, (
            f'Spin 终态 status={status}({_STATUS_NAMES.get(status, "?")})'))

    async def navigate_pose(
        self, pose: PoseRecord, on_feedback: Callable[[object], None], *,
        server_timeout_sec: float, send_timeout_sec: float, result_timeout_sec: float,
        on_timeout: Callable[[ActionState], None] | None = None,
    ) -> ActionOutcome:
        """通过共享客户端执行 Dock 位姿阶段，并保留完整原始反馈.

        三个阶段预算与清理等待预算独立。超时结果只在软件动作所有权清理
        确认后返回，不代表物理停稳；清理异常不能作为可继续的阶段失败。

        Args:
            pose: 已解析目标，不在这里查 POI 或吸附；消息采用节点当前时间。
            on_feedback: 循环线程上接收完整 ROS 反馈包装的回调。
            server_timeout_sec: 服务就绪预算。
            send_timeout_sec: 发送和接收预算。
            result_timeout_sec: 接收后的结果预算。
            on_timeout: 清理前同步调用的期限审计钩子。

        Returns:
            原始接收、终态或已确认清理的超时分类，供 worker 审计。

        Raises:
            Exception: 参数、源动作或所有权清理失败。
            asyncio.CancelledError: 显式取消并尝试清理后传播。
        """
        return await self._session.execute(
            self._client, lambda: self._build_pose(pose), on_feedback,
            server_timeout_sec=server_timeout_sec, send_timeout_sec=send_timeout_sec,
            result_timeout_sec=result_timeout_sec, on_timeout=on_timeout)

    async def spin_action(
        self, goal: SpinGoal, *, server_timeout_sec: float, send_timeout_sec: float,
        result_timeout_sec: float, on_timeout: Callable[[ActionState], None] | None = None,
    ) -> ActionOutcome:
        """在共享槽位执行带独立阶段期限的 Dock 旋转修正.

        使用与普通 Spin 相同的目标工厂，不另建动作所有者，也不提供距离反馈。

        Args:
            goal: 有符号旋转角与服务端时间额度。
            server_timeout_sec: 服务就绪预算。
            send_timeout_sec: 发送和接收预算。
            result_timeout_sec: 结果预算，与服务端额度分离。
            on_timeout: 期限到达后、所有权清理前的同步审计钩子。

        Returns:
            原始动作结果；本地超时仅在确认软件清理后返回。

        Raises:
            Exception: 无效目标、动作错误或未解决的所有权。
            asyncio.CancelledError: 显式取消并尝试清理后传播。
        """
        build = self._spin_factory(goal)
        return await self._session.execute(
            self._spin_client, build, lambda raw: None,
            server_timeout_sec=server_timeout_sec, send_timeout_sec=send_timeout_sec,
            result_timeout_sec=result_timeout_sec, on_timeout=on_timeout)

    async def cancel(self) -> None:
        """取消当前动作并等待会话清理，不封闭后续接收也不销毁客户端.

        Raises:
            TimeoutError: 调用方预算到期，后台清理和忙碌槽位仍被持有。
            Exception: 清理失败；不据此宣布空闲或物理停稳。
        """
        await self._session.cancel()

    async def settle(self) -> None:
        """封闭新动作并清理会话，同时保留全部 ROS 客户端.

        成功前必须保持 ROS 回调驱动；成功后外层才可静止回调线程，
        再调用 close 销毁自建客户端。失败不会重新开放接收。

        Raises:
            Exception: 清理尚未完成，所有权继续保留。
        """
        await self._session.close()

    async def close(self) -> None:
        """先完成会话清理，再逆序销毁本实例创建的客户端.

        注入客户端不由本实例销毁。遇普通销毁异常仍尝试其余自建客户端，
        成功项移除、失败项保留供重试；不承诺吞掉任意 BaseException。

        Raises:
            Exception: 会话未清理或客户端销毁失败；保留相应未完成阶段。
        """
        await self.settle()
        self._destroy_owned_clients()

    def _destroy_owned_clients(self) -> None:
        """逆创建顺序销毁自建客户端，成功项移除，失败项保留供重试.

        外层必须先确认动作会话已收尾；外部注入的客户端不在此列表中。

        Raises:
            Exception: 全部自建客户端均尝试销毁后，重抛最后一个销毁异常。
        """
        failure = None
        for client in tuple(reversed(self._owned_clients)):
            try:
                client.destroy()
            except Exception as error:
                failure = error
            else:
                self._owned_clients.remove(client)
        if failure is not None:
            raise failure

    async def _rollback_clients(self) -> None:
        # Construction never submitted work. Do not seal a caller-provided session.
        """构建回滚时只销毁本对象持有的动作客户端，不关闭借入的会话.

        构建尚未提交动作，不能因为构建失败就封闭调用方提供的共享 session。

        Raises:
            Exception: 自有客户端销毁失败，调用方应保留回滚所有权并重试。
        """
        self._destroy_owned_clients()

    @staticmethod
    def _describe_goal(goal: NavGoal) -> str:
        poi = goal.poi
        pose = poi.map_pose
        return (
            f'target={poi.poi_id}({poi.name}) '
            f'goal={pose.frame_id}(x={pose.x:.2f}, y={pose.y:.2f}, yaw={pose.yaw:.2f}rad)')

    def _build_goal(self, goal: NavGoal) -> object:
        return self._build_pose(goal.poi.map_pose)

    def _build_pose(self, pose: PoseRecord) -> object:
        """用节点当前时钟构造导航目标消息，不转换坐标系或发送目标.

        Args:
            pose: 目标帧下的位置（米）与 yaw（弧度）；原采集时间不写入目标。

        Returns:
            填好帧、当前时间、位置及 yaw 四元数的导航目标消息。

        Raises:
            Exception: 消息工厂、节点时钟或位姿转换失败。
        """
        message = self._goal_factory()
        message.pose.header.frame_id = pose.frame_id
        message.pose.header.stamp = self._node.get_clock().now().to_msg()
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        message.pose.pose.position.z = pose.z
        quaternion = pose.orientation()
        message.pose.pose.orientation.x = quaternion['x']
        message.pose.pose.orientation.y = quaternion['y']
        message.pose.pose.orientation.z = quaternion['z']
        message.pose.pose.orientation.w = quaternion['w']
        return message

    def _spin_factory(self, goal: SpinGoal) -> Callable[[], object]:
        """验证旋转参数并返回延迟构造消息的工厂，不在这里发送动作.

        Args:
            goal: 有符号旋转角（弧度）与非负服务端时间预算（秒）。

        Returns:
            零参数消息工厂；预算截断为整数纳秒，再拆为 ROS 秒与纳秒字段。

        Raises:
            ConfigError: 未配置旋转客户端，角度非有限或超出 float32，
                时间预算非有限、为负或不能放入 ROS Duration。
        """
        if self._spin_client is None:
            raise ConfigError('Spin is not configured on this shared backend')
        if (type(goal.target_yaw) not in (int, float) or
                not math.isfinite(goal.target_yaw) or
                abs(goal.target_yaw) > 3.4028234663852886e38):
            raise ConfigError('target_yaw must be a finite float32 value')
        seconds = goal.time_allowance_sec
        if (type(seconds) not in (int, float) or not math.isfinite(seconds) or
                seconds < 0 or seconds >= 2**31):
            raise ConfigError('time_allowance_sec must be nonnegative and fit ROS Duration')
        nanoseconds = int(seconds * 1000000000)

        def build():
            message = self._spin_goal_factory()
            message.target_yaw = float(goal.target_yaw)
            message.time_allowance.sec = nanoseconds // 1000000000
            message.time_allowance.nanosec = nanoseconds % 1000000000
            return message

        return build
