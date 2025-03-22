from typing import overload


class ConfigurationState:
    pass


class Edge:
    _start: "Vertex"
    _end: "Vertex"

    def intersects(self, other: "Edge") -> bool:
        raise NotImplementedError()

    @property
    def vertices(self) -> "list[Vertex]":
        raise NotImplementedError()

    @property
    def is_convex(self) -> bool:
        raise NotImplementedError()

    @property
    def normal(self) -> "Edge":
        raise NotImplementedError()

    @property
    def faces(self) -> "list[Face]":
        raise NotImplementedError()

    def other_face(self, face: "Face") -> "Face":
        raise NotImplementedError()


class Face:
    @property
    def edges(self) -> "list[Edge]":
        raise NotImplementedError()

    @property
    def surface(self) -> "None":
        raise NotImplementedError()


class Facet:
    pass


class Vertex:
    _x: float
    _y: float
    _z: float

    @property
    def position(self) -> "tuple[float, float, float]":
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
