"""解码既有命令并构造有序事件字典，不执行网络或仓库 I/O."""

from __future__ import annotations

from nav_tasks.api import StoredPoi

from nav_executor.protocol.messages import (
    AbortNavigationCommand, Command, MalformedCommandError, MarkCurrentPoiCommand,
    NavigateToCommand, ProtocolError, UnsupportedActionError, UpdatePoiListCommand,
)


def _extract_request_context(payload: dict) -> tuple[str | None, int]:
    """尽力提取错误应答所需的请求身份，不要求整帧命令有效.

    request_id 仅接受标量并转为去空白文本；空白或非标量变为 None。
    sub_id 转整数失败时退回零，保留既有线协议的宽松错误关联方式。

    Args:
        payload: 已解析的顶层消息字典。

    Returns:
        可用请求 ID 或 None，以及转换后的子任务整数 ID。
    """
    request_id = payload.get("request_id")
    request_id = (str(request_id).strip()
                  if isinstance(request_id, (str, int, float, bool)) else None)
    try:
        sub_id = int(payload.get("sub_id", 0))
    except (TypeError, ValueError, OverflowError):
        sub_id = 0
    return request_id or None, sub_id


def parse_command(payload: object) -> Command:
    """把已解码 JSON 字典转换为 v1 命令，在调用业务 API 前归一化格式错误.

    沿用旧协议的标量文本及整数转换策略，不是严格 JSON schema 验证。
    导航只使用 target_id，残留 target_pose/label 不转为直接坐标目标。

    Args:
        payload: 已由 JSON 解码器得到的值，命令顶层必须是 dict。

    Returns:
        对应动作的命令数据对象，不在此启动导航或修改 POI。

    Raises:
        MalformedCommandError: 顶层类型、字段或转换不合法，附带尽力恢复的请求身份。
        UnsupportedActionError: 动作未知，交给上层按既有策略记录并忽略。
    """
    if not isinstance(payload, dict):
        raise MalformedCommandError('command must be a JSON object')
    try:
        return _decode(payload)
    except ProtocolError:
        raise
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        request_id, sub_id = _extract_request_context(payload)
        raise MalformedCommandError(f'malformed command: {error}', request_id, sub_id) from error


def _text(value, field, allow_empty=False):
    """按既有协议把标量转成去空白文本，不额外校验业务标识语义.

    Args:
        value: 字符串、整数、浮点数或布尔值。
        field: 类型和空值错误信息中的字段名。
        allow_empty: 是否允许转换后为空字符串。

    Returns:
        str 转换并 strip 后的文本；不检查浮点有限性。

    Raises:
        TypeError: 输入不是支持的标量类型。
        ValueError: 转换后为空且未允许空文本。
    """
    if not isinstance(value, (str, int, float, bool)):
        raise TypeError(f'{field} must be a string or scalar value')
    result = str(value).strip()
    if not result and not allow_empty:
        raise ValueError(f'{field} must not be empty')
    return result


def _decode(payload):
    """按既有 v1 规则构造命令对象，不执行导航或写入 POI 仓库.

    Args:
        payload: 已由外层确认类型的 JSON 字典；导航只使用 target_id。

    Returns:
        与 action 对应的命令对象，POI 批量更新保留原始字段值。

    Raises:
        MalformedCommandError: 导航请求缺少 request_id 或有效 target_id。
        UnsupportedActionError: action 未受支持。
        KeyError: 缺少其它命令的必需字段，外层会转换为协议错误。
        TypeError: 字段结构或类型错误，外层会转换为协议错误。
        ValueError: 字段为空或整数转换失败，外层会转换为协议错误。
        OverflowError: 整数转换溢出，外层会转换为协议错误。
    """
    action = _text(payload.get("action", ""), 'action', allow_empty=True)
    if action == "navigate_to":
        # v1 仅按 target_id 查 POI；残留的 target_pose/label 不参与导航。
        # 兼容性依据见 nav_executor/docs/adr-0012-v1-poi-targets.md。
        request_id, sub_id = _extract_request_context(payload)
        if request_id is None:
            raise MalformedCommandError("navigate_to 缺少 request_id", None, sub_id)
        target_id = payload.get("target_id")
        if not isinstance(target_id, str) or not target_id.strip():
            raise MalformedCommandError(
                "navigate_to 缺少有效 target_id(v1 只认 POI id,target_pose 已废弃)",
                request_id,
                sub_id,
            )
        return NavigateToCommand(
            action="navigate_to",
            request_id=request_id,
            sub_id=sub_id,
            target_id=target_id.strip(),
        )
    if action == "mark_current_poi":
        poi = payload["poi"]
        if not isinstance(poi, dict):
            raise TypeError('poi must be an object')
        return MarkCurrentPoiCommand(
            action="mark_current_poi",
            request_id=_text(payload["request_id"], 'request_id'),
            sub_id=int(payload["sub_id"]),
            poi_id=_text(poi["id"], 'poi.id'),
            poi_name=_text(poi["name"], 'poi.name', allow_empty=True),
        )
    if action == "abort_navigation":
        return AbortNavigationCommand(
            action="abort_navigation",
            request_id=_text(payload["request_id"], 'request_id'),
        )
    if action == "update_poi_list":
        items = payload.get("poi_list", [])
        if not isinstance(items, list):
            raise TypeError('poi_list must be a list')
        for item in items:
            if not isinstance(item, dict):
                raise TypeError('poi_list entries must be objects')
            # Validate the whole batch, retaining the original values/whitespace
            # for legacy directory conversion. No repository mutation happens here.
            _text(item['id'], 'poi.id')
            _text(item['name'], 'poi.name', allow_empty=True)
        return UpdatePoiListCommand(
            action="update_poi_list",
            version=int(payload.get("version", 0)),
            poi_list=list(items),
        )
    raise UnsupportedActionError(f"unsupported action: {action}")


def build_progress_event(
    request_id: str,
    sub_id: int,
    remaining_distance: float,
    status: str,
) -> dict:
    """构造有序进度字典，沿用距离转 float 后 round 到三位小数的规则.

    Args:
        request_id: 请求关联标识，原样保留。
        sub_id: 子任务关联编号，原样保留。
        remaining_distance: 剩余距离，单位米；不在此额外检查有限性。
        status: 上层提供的既有 running/paused 状态文本。

    Returns:
        待后续 JSON 序列化的有序 on_progress 字典，不在此发送。

    Raises:
        TypeError: 距离不支持浮点转换。
        ValueError: 距离文本不能转换为浮点数。
        OverflowError: 距离数值转换超出浮点范围。
    """
    return {
        "event_type": "on_progress",
        "request_id": request_id,
        "sub_id": sub_id,
        "remaining_distance": round(float(remaining_distance), 3),
        "status": status,
    }


def build_mark_poi_success_event(
    request_id: str,
    sub_id: int,
    poi: StoredPoi,
) -> dict:
    """使用已存 POI 的身份构造标记成功事件，不在此执行存储.

    Args:
        request_id: 请求关联标识。
        sub_id: 子任务关联编号。
        poi: 提供 poi_id 与 name 的已存 POI 对象。

    Returns:
        保持既有字段顺序的 on_mark_poi_success 字典。
    """
    return {
        "event_type": "on_mark_poi_success",
        "request_id": request_id,
        "sub_id": sub_id,
        "poi": {
            "id": poi.poi_id,
            "name": poi.name,
        },
    }


def build_arrived_event(request_id: str, sub_id: int) -> dict:
    """构造导航成功终态字典，不自行判断到点、脱困或停稳.

    Args:
        request_id: 请求关联标识。
        sub_id: 子任务关联编号。

    Returns:
        保持既有字段顺序的 on_arrived 字典，由上层负责成功判定及发送。
    """
    return {
        "event_type": "on_arrived",
        "request_id": request_id,
        "sub_id": sub_id,
    }


def build_error_event(request_id: str, sub_id: int, error_message: str) -> dict:
    """构造导航失败字典，错误详情只做既有 str 转换.

    Args:
        request_id: 请求关联标识。
        sub_id: 子任务关联编号。
        error_message: 转换成字符串后写入的错误详情，不额外改写或清洗。

    Returns:
        保持既有字段顺序的 on_error 字典。
    """
    return {
        "event_type": "on_error",
        "request_id": request_id,
        "sub_id": sub_id,
        "error_message": str(error_message),
    }


def build_mark_poi_ack(request_id: str, sub_id: int) -> dict:
    """构造 accepted 标记应答，不决定它何时发送或证明 POI 已存储.

    适配器可先构造本字典再等待 mark，但本函数没有任何发送副作用；
    当前适配器在 mark 返回后才将 ACK 与结果一同返回。

    Args:
        request_id: 请求关联标识。
        sub_id: 子任务关联编号。

    Returns:
        保持既有字段顺序、status 为 accepted 的 on_mark_poi_ack 字典。
    """
    return {
        "event_type": "on_mark_poi_ack",
        "request_id": request_id,
        "sub_id": sub_id,
        "status": "accepted",
    }


def build_mark_poi_error(request_id: str, sub_id: int, error_message: str) -> dict:
    """按既有字段顺序构造位姿采集或 POI 存储失败事件.

    Args:
        request_id: 请求关联标识。
        sub_id: 子任务关联编号。
        error_message: 只做既有 str 转换的错误详情。

    Returns:
        有序 on_mark_poi_error 字典，不在此发送。
    """
    return {
        "event_type": "on_mark_poi_error",
        "request_id": request_id,
        "sub_id": sub_id,
        "error_message": str(error_message),
    }
