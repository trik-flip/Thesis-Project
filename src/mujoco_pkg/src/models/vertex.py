from src.mujoco_pkg.src.models.Face import Face
from src.mujoco_pkg.src.models.Edge import Edge


class Vertex:
    @property
    def is_convex(self) -> bool:
        raise NotImplementedError()

    def lies_on(self, other: "Face | Edge") -> bool:
        if isinstance(other, Face):
            raise NotImplementedError()
        elif isinstance(other, Edge):
            raise NotImplementedError()
        raise Exception("Not possible")