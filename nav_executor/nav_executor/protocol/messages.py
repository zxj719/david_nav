"""定义既有 WS v1 命令数据和携带请求身份的解析错误，不执行 I/O."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Union


class NavExecutorError(Exception):
    """作为执行器协议错误基类，与公共任务 API 的错误体系分开."""


class ProtocolError(NavExecutorError, ValueError):
    """保留 ValueError 兼容性，并携带尽力恢复的协议请求身份.

    Attributes:
        request_id: 可恢复的请求标识；无法恢复时为 None。
        sub_id: 尽力恢复的子任务编号，解码失败时沿用零值策略。
    """

    def __init__(self, message: str, request_id: str | None = None, sub_id: int = 0) -> None:
        """保存原始错误文本和已恢复请求身份，不在异常对象中重新校验.

        Args:
            message: 可读的协议错误说明。
            request_id: 已恢复请求标识或 None。
            sub_id: 已恢复子任务编号，解码时可退回默认零。
        """
        super().__init__(message)
        self.request_id = request_id
        self.sub_id = sub_id


class CommandParseError(ProtocolError):
    """兼容旧导入路径的命令解析错误基类，可携带恢复出的请求身份."""


class MalformedCommandError(CommandParseError):
    """表示字段缺失、形状或转换错误，上层仅在身份可恢复时构造应答."""


class UnsupportedActionError(ProtocolError):
    """表示未知动作，供上层记录并忽略，不自行生成协议应答."""


@dataclass(frozen=True)
class NavigateToCommand:
    """保存只引用 POI 标识的 v1 导航请求，构造器不重复执行解析校验.

    Attributes:
        action: 命令动作名 navigate_to。
        request_id: 上游请求标识，用于关联事件。
        sub_id: 上游子任务编号。
        target_id: 已有 POI 标识，不携带直接导航坐标。
    """

    action: str
    request_id: str
    sub_id: int
    target_id: str


@dataclass(frozen=True)
class MarkCurrentPoiCommand:
    """保存将当前位姿标记为命名 POI 的请求，不在构造时采集或存储.

    Attributes:
        action: 命令动作名 mark_current_poi。
        poi_id: 要标记的 POI 标识。
        poi_name: POI 显示名称，可为空字符串。
        request_id: 上游请求标识，用于关联事件。
        sub_id: 上游子任务编号。
    """

    action: str
    request_id: str
    sub_id: int
    poi_id: str
    poi_name: str


@dataclass(frozen=True)
class AbortNavigationCommand:
    """保存按请求标识取消导航的命令，不代表取消已被接受或完成.

    Attributes:
        action: 命令动作名 abort_navigation。
        request_id: 待取消的匹配请求标识。
    """

    action: str
    request_id: str


@dataclass(frozen=True)
class UpdatePoiListCommand:
    """保存带版本号的目录更新，字段冻结不等于列表和内部字典深冻结.

    Attributes:
        action: 命令动作名 update_poi_list。
        poi_list: 保留原输入值的目录项列表，业务层只更新已有 POI 名称；
            解码只浅拷贝列表，不复制各项字典。
        version: 已解码的目录版本号，当前不参与版本冲突裁决。
    """

    action: str
    version: int
    poi_list: list[dict]


Command = Union[
    NavigateToCommand,
    MarkCurrentPoiCommand,
    AbortNavigationCommand,
    UpdatePoiListCommand,
]
