class Face:
    @property
    def edges(self) -> "list[Edge]":
        raise NotImplementedError()
    @property
    def surface(self) -> "":
        raise NotImplementedError()