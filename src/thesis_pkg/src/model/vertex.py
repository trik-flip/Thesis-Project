from typing import overload
from .face import Face
from .edge import Edge


class Vertex:
    _x: float
    _y: float
    _z: float

    @property
    def position(self) -> "tuple[float, float,float]":
        return self._x, self._y, self._z

    @property
    def is_convex(self) -> bool:
        # ? Is this logical
        raise NotImplementedError()

    def __sub__(self, other: "Vertex") -> "Vertex":
        if not isinstance(other, Vertex):
            raise TypeError("")
        raise NotImplementedError()

    @overload
    def lies_on(self, other: "Face") -> bool: ...
    @overload
    def lies_on(self, other: "Edge") -> bool: ...
    def lies_on(self, other: "Face | Edge") -> bool:
        if isinstance(other, Face):
            raise NotImplementedError()
        elif isinstance(other, Edge):
            raise NotImplementedError()
