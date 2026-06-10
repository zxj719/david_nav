import math

from david_nav.grid_map_view import GridMapView


def _grid(cells, w, h, res=1.0, ox=0.0, oy=0.0):
    return GridMapView(cells=cells, width=w, height=h, resolution=res,
                       origin_x=ox, origin_y=oy)


def test_in_bounds_and_value():
    g = _grid([0, 1, 2, 3], 2, 2)
    assert g.in_bounds(0, 0)
    assert g.in_bounds(1, 1)
    assert not g.in_bounds(2, 0)
    assert not g.in_bounds(-1, 0)
    assert g.value(1, 1) == 3  # y*width + x = 1*2 + 1


def test_world_to_cell_and_center():
    g = _grid([0] * 16, 4, 4, res=0.5, ox=-1.0, oy=-1.0)
    # world (0.0, 0.0) -> floor((0 - -1)/0.5) = 2
    assert g.world_to_cell(0.0, 0.0) == (2, 2)
    # out of bounds returns None
    assert g.world_to_cell(100.0, 0.0) is None
    cx, cy = g.cell_center_world(2, 2)
    assert math.isclose(cx, 0.25) and math.isclose(cy, 0.25)


def test_is_blocked_threshold_and_unknown():
    # values on the OccupancyGrid 0..100 scale, -1 = unknown
    g = _grid([0, 50, 99, -1], 2, 2)
    assert not g.is_blocked(0, 0, threshold=99, block_unknown=True)   # free
    assert not g.is_blocked(1, 0, threshold=99, block_unknown=True)   # 50 < 99
    assert g.is_blocked(0, 1, threshold=99, block_unknown=True)       # 99 >= 99
    assert g.is_blocked(1, 1, threshold=99, block_unknown=True)       # unknown
    assert not g.is_blocked(1, 1, threshold=99, block_unknown=False)  # unknown allowed


def test_from_occupancy_grid_factory():
    class _Info:
        width = 2
        height = 2
        resolution = 0.5

        class origin:
            class position:
                x = -1.0
                y = -2.0

    class _Msg:
        info = _Info()
        data = [0, 1, 2, 3]

    g = GridMapView.from_occupancy_grid(_Msg())
    assert g is not None
    assert g.width == 2 and g.height == 2
    assert math.isclose(g.resolution, 0.5)
    assert math.isclose(g.origin_x, -1.0) and math.isclose(g.origin_y, -2.0)

    class _Empty:
        class info:
            width = 0
            height = 0
            resolution = 0.0

            class origin:
                class position:
                    x = 0.0
                    y = 0.0
        data = []

    assert GridMapView.from_occupancy_grid(_Empty()) is None
    assert GridMapView.from_occupancy_grid(None) is None
