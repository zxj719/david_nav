# ADR-0012: keep v1 target identity as a POI identifier

Status: existing compatibility decision, documented without a protocol change.

The v1 `navigate_to` decoder accepts a nonempty string `target_id`, not an inline
target pose. Residual `target_pose` and `label` fields do not activate an alternate
navigation route. The task adapter resolves the POI before admission; documenting
this rule does not introduce `dock_to`, new fields or a handshake.

Current [codec](../nav_executor/protocol/codec.py) normalizes malformed commands
through the [R1 boundary](adr-0007-message-isolation.md). Recoverable request
context permits the existing error response; missing identity cannot be invented.
The original comment's core/error wording below is historical, not a replacement
for the current parser/adapter ownership described in the
[WS contract](../../docs/protocol/nav_ws_v1.md).

Verification retains [wire goldens](../test/test_task_wire_golden.py) and
[malformed-command tests](../test/test_malformed_messages.py); their expectations
are not generated from this ADR. The source edit changes comments only.

## 背景原文（逐字保留）

下列片段取自标明的不可变提交，保留原标点、缩进和时间。它们不是本轮新的现场测量，
也不是实体停车/到达验收；存在与现状不同的历史描述时，以本 ADR 上方当前决定为准。

### wire-v1-target

来源：`ca42041:nav_executor/nav_executor/protocol/codec.py`。
SHA256：`139074b0a0e5be880c0171ac9cd989e5f9641e6bd2e40a453c4faf2dd1b2d780`。

<!-- history: wire-v1-target -->

```python
        # nav 协议 v1(2026-07-22 回退定稿):只认 target_id(POI id 字符串)。
        # v2 的 target_pose/label 字段按协议要求直接忽略——即使还带着,也
        # 不再走坐标直达路径。target_id 缺失/类型错 → CommandParseError,
        # core 回 on_error,不启车。
```
