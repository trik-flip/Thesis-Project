# %%
import logging
from math import cos, pi, sin
from typing import Callable, Iterable

from utils.math import quadratic_from_lagrange_polynomial_interpolation

_logger = logging.getLogger("ModelGeneratorLogger")
_logger.setLevel(logging.INFO)

__all__ = [
    "gen_cylinder_part",
    "gen_hollow_cylinder_points",
    "gen_arc_end_effector",
    "gen_arc_rect",
    "gen_cylinder",
    "gen_hole_pyramid_part",
]


class Point:
    x: float
    y: float
    z: "float|None"

    def __init__(
        self, x: float, y: float, z: "float|None" = None, precision: int = 2
    ) -> None:
        self.x = round(x, precision)
        self.y = round(y, precision)
        if z:
            self.z = round(z, precision)
        else:
            self.z = None

    def __mul__(self, val: "float|int") -> "Point":
        return Point(self.x * val, self.y * val, self.z * val if self.z else None)

    def __iter__(self):
        return iter((self.x, self.y, self.z)) if self.z else iter((self.x, self.y))


def gen_cylinder_part(n: int, outer_r: float, h: float, inner_r: float = 0):
    part_size = 2 * pi / n
    points = Point(cos(part_size), sin(part_size))

    inner = points * inner_r
    outer = points * outer_r

    inner_bottom = (*inner, 0)
    inner_top = (*inner, h)

    outer_bottom = (*outer, 0)
    outer_top = (*outer, h)

    return (
        (inner_r, 0, 0),
        (inner_r, 0, h),
        (outer_r, 0, 0),
        (outer_r, 0, h),
        inner_bottom,
        inner_top,
        outer_bottom,
        outer_top,
    )


def gen_hollow_cylinder_points(
    n: int, outer_r: float, h: float, inner_r: float
) -> "Iterable[tuple[float, float, float]]":
    """
    return points are inner bottom, inner top, outer bottom, outer top
    to reverse engineer `n` do `len(points) // 4`
    """
    equal_part = 2 * pi / n
    points = [
        (round(cos(equal_part * i), 2), round(sin(equal_part * i), 2)) for i in range(n)
    ]
    outer_points = [(outer_r * x, outer_r * y) for x, y in points]
    outer_bottom = [(x, y, 0) for x, y in outer_points]
    outer_top = [(x, y, h) for x, y in outer_points]

    inner_points = [(inner_r * x, inner_r * y) for x, y in points]
    inner_bottom = [(x, y, 0) for x, y in inner_points]
    inner_top = [(x, y, h) for x, y in inner_points]
    return [*inner_bottom, *inner_top, *outer_bottom, *outer_top]


# source: https://stackoverflow.com/questions/19175037/determine-a-b-c-of-quadratic-equation-using-data-points
# Lagrange Interpolation
def _handle_offset(
    points: "Iterable[tuple[float,float,float]]", x_offset: float, y_offset: float
):
    for x, y, z in points:
        yield x - x_offset, y - y_offset, z


def _handle_offset2(
    *points: "Iterable[tuple[float,float,float]]", x_offset: float, y_offset: float
):
    for obj in points:
        for x, y, z in obj:
            yield x - x_offset, y - y_offset, z


def gen_arc_end_effector(
    n: int, r: float, d: float, _h: float, _w: float, l: float
) -> "Iterable[Iterable[tuple[float,float,float]]]":
    assert n > 1
    assert r > 0
    assert d > 0
    assert l > 0
    poly = quadratic_from_lagrange_polynomial_interpolation((r / 2, 0, r), (d, 0, 0))
    inner_pieces = (_gen_sub_part(poly, i, r / n, l) for i in range(n))
    return (_handle_offset(piece, r / 2, l / 2) for piece in inner_pieces)


def gen_arc_rect(
    n: int, r: float, d: float, h: float, w: float, l: float
) -> "Iterable[Iterable[tuple[float,float,float]]]":
    assert w > r
    assert h > d
    assert n < 200, "don't make the objects to complicated"

    margin = (w - r) / 2

    x_offset = w / 2
    y_offset = l / 2

    mid = w / 2
    x, z = mid, h - d

    poly = quadratic_from_lagrange_polynomial_interpolation(
        (x, margin, w - margin), (z, h, h)
    )

    front_piece = (
        (0, 0, 0),
        (0, 0, h),
        (margin, 0, 0),
        (margin, 0, h),
        (0, l, 0),
        (0, l, h),
        (margin, l, 0),
        (margin, l, h),
    )

    last_piece = (
        (w - margin, 0, 0),
        (w - margin, 0, h),
        (w, 0, 0),
        (w, 0, h),
        (w - margin, l, 0),
        (w - margin, l, h),
        (w, l, 0),
        (w, l, h),
    )

    inner_pieces = (_gen_sub_part(poly, i, r / n, l, margin) for i in range(n))
    return _handle_offset2(
        front_piece, *inner_pieces, last_piece, x_offset=x_offset, y_offset=y_offset
    )


def _gen_sub_part(
    poly: Callable[[float], float],
    i: int,
    part_size: float,
    l: float,
    start: float = 0,
) -> "Iterable[tuple[float,float,float]]":
    x_1 = start + part_size * i
    x_2 = x_1 + part_size

    y_1 = poly(x_1)
    y_2 = poly(x_2)

    points = [
        (x_1, 0, 0),
        (x_2, 0, 0),
        (x_1, 0, y_1),
        (x_2, 0, y_2),
        (x_1, l, 0),
        (x_2, l, 0),
        (x_1, l, y_1),
        (x_2, l, y_2),
    ]

    return points


def gen_arc_round(n: int, radius: float, depth: float, h: float):
    pass


def gen_cylinder(n: int, r: float, h: float) -> "Iterable[tuple[float, float, float]]":
    """
    return points are bottom, top
    to reverse engineer `n` do `len(points) // 2`
    """
    equal_part = 2 * pi / n
    points = (
        (round(cos(equal_part * i), 2), round(sin(equal_part * i), 2)) for i in range(n)
    )
    points = [(r * x, r * y) for x, y in points]

    bottom = ((x, y, 0) for x, y in points)
    top = ((x, y, h) for x, y in points)
    return [*bottom, *top]


def gen_hole_pyramid_part(
    l: float, w: float, d: float
) -> "Iterable[tuple[float,float,float]]":
    front = ((0, 0, 0), (0, 0, d), (l, 0, d), (l, 0, 0), (l / 2, w / 2, 0))
    return _handle_offset(front, l / 2, w / 2)


def gen_hole_pyramid(
    l: float, w: float, d: float
) -> "Iterable[Iterable[tuple[float,float,float]]]":
    front = ((0, 0, 0), (0, 0, d), (l, 0, d), (l, 0, 0), (l / 2, w / 2, 0))
    back = ((0, w, 0), (0, w, d), (l, w, d), (l, w, 0), (l / 2, w / 2, 0))
    left = ((0, 0, 0), (0, w, 0), (0, w, d), (0, 0, d), (l / 2, w / 2, 0))
    right = ((l, 0, 0), (l, 0, d), (l, w, d), (l, w, 0), (l / 2, w / 2, 0))
    return _handle_offset2(
        front,
        back,
        left,
        right,
        x_offset=l / 2,
        y_offset=w / 2,
    )
