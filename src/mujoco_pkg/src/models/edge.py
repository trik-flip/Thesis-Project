class Edge:
    def intersects(self, other:"Edge") -> bool:
        raise NotImplementedError()
    @property
    def vertices(self) -> "list[Vertex]":
        raise NotImplementedError()
    @property
    def is_convex(self) -> bool:
        raise NotImplementedError()

    @property
    def faces(self) -> "list[Face]":
        raise NotImplementedError()

    def other_face(self, face: "Face") -> "Face":
        raise NotImplementedError()
