# 2026-04-07 Frontier Costmap Search 日志

## 背景

本周在现场观察到一类新的 frontier 选点异常：

- `grid_map` 中未知边界附近的障碍点比较稀碎
- `frontier_search` 仍然只按 `grid_map` 的 free/unknown 连通关系聚类
- `global_costmap` 只在最终 snapped goal 准入时才参与

这会导致两个直接后果：

- frontier cluster 会穿插进高代价膨胀区
- frontier 质心会被被污染的 cluster 拉偏，即使最终 snapped goal 还能被二次修正

## 现场现象

在 RViz 里可以看到 frontier 沿着未知边界连续生成，但其中夹杂着高代价障碍碎片。由于这些碎片没有在 frontier 生成阶段被排除，系统仍把它们两侧的 frontier cells 视为同一簇，最终导致质心偏移。

```mermaid
flowchart LR
    A["grid_map frontier cells<br/>沿未知边界连续生成"] --> B["高代价 costmap 碎片<br/>未参与 frontier 聚类过滤"]
    B --> C["frontier cluster 被错误连通"]
    C --> D["frontier centroid 偏移"]
    D --> E["后续 snap 只能修 nav goal<br/>不能修 cluster 本身"]
```

## 本周决策

本周采用“方案一”：

- 继续保留 `grid_map` 作为 frontier 的基础语义来源
- 在 frontier 生成阶段额外引入 `global_costmap` 过滤
- 过滤方式不是把所有非零膨胀代价都当障碍，而是使用独立阈值

这意味着：

- 高代价膨胀碎片会切断 frontier 的错误连通
- 低代价边缘 cells 仍可保留，避免探索过度保守

## 实现落点

本周代码改动集中在以下位置：

- `frontier_search` 现在同时接收 `grid_map` 和 `global_costmap`
- 新增 `costmap_search_threshold` 参数，用于控制 frontier 搜索阶段允许的最大 costmap 代价
- BFS 扩张、frontier seed 查找、frontier cell 判定都会共同参考：
  - `grid_map.search_free_threshold`
  - `global_costmap.costmap_search_threshold`

当前默认值：

- `search_free_threshold: 0`
- `costmap_search_threshold: 20`

## 预期效果

- frontier 不再轻易穿过高代价膨胀区
- frontier centroid 更接近真实可探索边界
- snapped goal 不再承担“修复错误 cluster 质心”的主要责任
- 仍然保留低代价边缘区域的探索机会，避免把门边、墙边 frontier 一刀切掉

## 风险与后续观察点

- 如果 `costmap_search_threshold` 过低，门口和贴墙区域的 frontier 可能被压掉
- 如果 `costmap_search_threshold` 过高，高代价碎片仍可能把 frontier 串起来
- 后续需要现场结合 RViz 观察：
  - frontier cells 是否仍穿插在明显膨胀障碍中
  - frontier centroid 是否仍明显偏向障碍一侧
  - 狭窄通道入口的 frontier 是否被过度抑制
