"""提供目标解析与路径稀疏化共用的旧轴对齐栅格访问逻辑.

该模型不处理地图原点旋转，也不代表 Dock 的完整车身轮廓校验。
冻结字段不等于深冻结 cells；查询方法不修改数据，但调用方仍可修改列表。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any


@dataclass(frozen=True)
class GridMapView:
    """保存旧轴对齐栅格的字段与访问规则，不作完整地图合法性校验.

    源自旧目标解析器，供路径稀疏化复用同一坐标计算。目标准入严格要求格值
    为零；视线阻挡检查采用调用方的阈值和未知格策略，二者不能互相替代。
    预期消息格值为 0..100 或未知值 -1，但构造时不验证此范围。

    Attributes:
        cells: 按行排列的格值列表；冻结对象不会深冻结该列表。
        height: 栅格行数。
        origin_x: 旧轴对齐栅格模型的原点 x（米）。
        origin_y: 旧轴对齐栅格模型的原点 y（米）。
        resolution: 每格边长（米）。
        width: 栅格列数。
    """

    cells: list
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float

    def in_bounds(self, x: int, y: int) -> bool:
        """只按宽高范围判断格坐标是否在图内.

        Args:
            x: 列下标，应由调用方保证为整数；此处只有范围比较。
            y: 行下标，应由调用方保证为整数。

        Returns:
            两个下标均在对应半开范围内时为 True。
        """
        return 0 <= x < self.width and 0 <= y < self.height

    def value(self, x: int, y: int) -> int:
        """直接按行优先下标读取格值，不添加越界检查或占用策略.

        必须先分别检查 x/y；单一列表下标合法不代表二维坐标合法，
        负下标也会沿用 Python 列表语义。

        Args:
            x: 已由调用方检查的列下标。
            y: 已由调用方检查的行下标。

        Returns:
            原始占用或代价值，包括负的未知值。

        Raises:
            IndexError: 计算得到的列表下标超出 cells 可索引范围。
        """
        return self.cells[y * self.width + x]

    def is_goal_free(self, x: int, y: int) -> bool:
        """按既有严格零代价规则判断目标格准入.

        Args:
            x: 已由调用方检查的列下标。
            y: 已由调用方检查的行下标。

        Returns:
            原始格值恰好等于零时为 True；未知格不满足该条件。

        Raises:
            IndexError: 底层格值读取的列表下标越界。
        """
        return self.value(x, y) == 0

    def is_blocked(self, x: int, y: int, threshold: int,
                   block_unknown: bool) -> bool:
        """按调用方阈值和未知格策略判断阻挡，不验证阈值本身.

        Args:
            x: 已由调用方检查的列下标。
            y: 已由调用方检查的行下标。
            threshold: 非负格值达到或超过该值即阻挡。
            block_unknown: 负格值是否阻挡视线。

        Returns:
            负格值采用 block_unknown，其余格值采用包含边界的阈值比较。

        Raises:
            IndexError: 底层格值读取的列表下标越界。
        """
        v = self.value(x, y)
        if v < 0:
            return block_unknown
        return v >= threshold

    def world_to_cell(self, wx: float, wy: float) -> tuple[int, int] | None:
        """按旧轴对齐原点和分辨率向下取整，把世界坐标转换为格坐标.

        不处理地图原点旋转；调用方负责提供有效分辨率和有限坐标。
        无效数值的计算错误不会统一转换为 None。

        Args:
            wx: 世界系 x 坐标，单位米。
            wy: 世界系 y 坐标，单位米。

        Returns:
            列/行下标；计算得到的格坐标越出宽高范围时返回 None。

        Raises:
            ZeroDivisionError: 分辨率为零。
            ValueError: 取整遇到 NaN。
            OverflowError: 取整遇到无穷值。
        """
        x = int(math.floor((wx - self.origin_x) / self.resolution))
        y = int(math.floor((wy - self.origin_y) / self.resolution))
        if not self.in_bounds(x, y):
            return None
        return (x, y)

    def cell_center_world(self, x: int, y: int) -> tuple[float, float]:
        """按旧轴对齐模型计算格中心，不检查下标是否在图内.

        Args:
            x: 列下标，调用方应保证整数类型和所需范围。
            y: 行下标，调用方应保证整数类型和所需范围。

        Returns:
            对应格中心的世界系 x/y，单位米；不应用原点旋转。
        """
        return (
            (float(x) + 0.5) * self.resolution + self.origin_x,
            (float(y) + 0.5) * self.resolution + self.origin_y,
        )

    @classmethod
    def from_occupancy_grid(cls, msg: Any) -> "GridMapView | None":
        """从兼容消息复制旧栅格字段，不引入 ROS 或完整验证器.

        只拒绝消息缺失、宽高非正或数据为空；不检查数据长度、格值范围、
        分辨率合法性或原点旋转。复制后的列表与消息独立，但仍可修改。

        Args:
            msg: 具有 info/data 字段的对象，或 None；字段应能完成相应数值转换。

        Returns:
            字段快照，或在上述缺失/空/非正宽高情形下返回 None。
            字段访问和类型转换异常向调用方传播，不作为空地图处理。
        """
        if msg is None:
            return None
        info = msg.info
        if info.width <= 0 or info.height <= 0 or not msg.data:
            return None
        return cls(
            cells=list(msg.data),
            width=int(info.width),
            height=int(info.height),
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
        )
