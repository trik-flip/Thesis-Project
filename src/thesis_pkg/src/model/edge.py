from .face import Face
from .vertex import Vertex


class Edge:
    _start: Vertex
    _end: Vertex

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
