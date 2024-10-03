from .edge import Edge


class Face:
    @property
    def edges(self) -> "list[Edge]":
        raise NotImplementedError()

    @property
    def surface(self) -> "None":
        raise NotImplementedError()
