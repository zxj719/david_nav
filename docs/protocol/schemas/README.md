# 归档的 AsyncAPI 元 schema

`nav_ws_v1.asyncapi.yaml` 的整份文档要能在没有网络的机器上重复校验，所以把官方元 schema
归档在这里。测试 `nav_executor/test/test_protocol_documentation.py` 会先核对下表的 sha256，
再用它校验文档；文件被换掉、升级或损坏都会失败。

| 项 | 值 |
|---|---|
| 文件 | `asyncapi-2.6.0.json` |
| 上游 | `asyncapi/spec-json-schemas` 仓库的 `schemas/2.6.0-without-$id.json` |
| 固定提交 | `23bb8b0c9d1cbe45edbccb3308b956de94c11a0c` |
| 取回时间 | 2026-09-18 |
| sha256 | `be227ff668d64da62932c8409daca58fef1f7749f5d386175111f17314595137` |
| 许可证 | Apache-2.0（上游仓库） |
| JSON Schema 草案 | draft-07 |

为什么用 `-without-$id` 这个变体：官方的 `schemas/2.6.0.json` 里有 234 处指向
`http://asyncapi.com/definitions/` 的外部引用，离线解析不了；这个变体把定义内联在同一份文件里，
没有外部引用，语义与官方 2.6.0 相同。

升级步骤：换文件后同步更新上表的提交号、时间与 sha256，并更新测试里的 `META_SCHEMA_SHA256`；
文档若因此校验失败，按 SPEC 的协议章节评估是文档要改还是版本不该升。
