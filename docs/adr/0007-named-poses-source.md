# ADR-0007：named_poses 导入源与会话清理

状态：兼容行为已迁入存储适配器。此文件保留 SPEC 的 ADR-0007 主题；
与 [nav_executor 的 R1 ADR-0007](../../nav_executor/docs/adr-0007-message-isolation.md)
位于不同目录，不改名或覆盖既有链接。

## 背景原文（保留）

来源 `93676a5:david_nav/david_nav/poi_store.py` 的 `prune_session` 文档：

```text
删掉某会话中不在 keep_ids 里的点位,返回被删的 id 列表。

named_poses.json 是点位映射的唯一真源(nav WS v1 §四.1):启动导入
时用它清理上次导入残留的旧 id,否则 json 里已删/已改的点位会一直
躺在持久化 store 里,agent 发旧 id 时会导航到过期坐标而不是报错。
```

来源为提交 `93676a5` 中归档的旧导航入口：

```python
# named_poses.json 是映射唯一真源:清掉上次导入残留、这次 json 里
# 已经不存在的点位,避免 agent 发旧 id 时导航到过期坐标。
```

## 决定与当前落点

[import_named_poses](../../nav_tasks/nav_tasks/nav_site_data/named_poses_importer.py)
只读取人工维护 JSON，不回写。逐条写入 `named_poses` 会话，再仅清理该会话不在本次集合的
旧 ID；运行期标记的其它会话不因这次清理被删。入口必须显式提供 pose_file 才导入，
不是每次重连自动导入。目录 update_poi_list 仅重命名，不成为坐标真源。

原样把手工坐标复制为 map/odom 两份表示，frame 标签按配置保留；本次不纠正点位坐标语义、
不把 body/nav_base/poi_capture 合并成一个 frame。导入保留逐项写入与既有异常，
不是新的全批次事务；中途坏记录可能留下前面已写入内容，清理在成功迭代后才发生。

## 证据与边界

`nav_tasks/test/test_site_data.py`
覆盖格式兼容、重命名及按会话裁剪。这里说明程序契约，未修改用户真实 JSON/POI store。
“唯一真源”限定人工导入的该会话，不抹掉运行期标记能力，也不证明手工坐标 frame 正确。
