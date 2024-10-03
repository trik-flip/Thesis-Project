import random
from math import log2 as log
from typing import Iterable, Literal, overload

from my_types.axis import Axis


def create_random_directions(axes: Axis) -> "Iterable[Literal[-1] | Literal[1]]":
    directions = []
    for _axis in range(len(axes)):
        direction = 1 if random.random() < 0.5 else -1
        directions.append(direction)
    return directions


class RobotController:
    # region: Interfaces
    @overload
    def move(self, axes: Literal[Axis.X], *directions: int) -> None:
        """If `direction` is positive it means front, otherwise it means back"""
        ...

    @overload
    def move(self, axes: Literal[Axis.Y], *directions: int) -> None:
        """If `direction` is positive it means left, otherwise it means right"""
        ...

    @overload
    def move(self, axes: Literal[Axis.Z], *directions: int) -> None:
        """if `direction` is positive it means up, otherwise it means down"""
        ...

    @overload
    def move(self, axes: Axis, directions: Literal["random"]) -> None:
        """the robot will move up and down in a random fashion in the specified Axes"""
        ...

    @overload
    def rotate(self, axes: Literal[Axis.X], *directions: int) -> None:
        """If `direction` is positive it means front, otherwise it means back"""
        ...

    @overload
    def rotate(self, axes: Literal[Axis.Y], *directions: int) -> None:
        """If `direction` is positive it means left, otherwise it means right"""
        ...

    @overload
    def rotate(self, axes: Literal[Axis.Z], *directions: int) -> None:
        """if `direction` is positive it means up, otherwise it means down"""
        ...

    @overload
    def rotate(self, axes: Axis, directions: Literal["random"]) -> None:
        """the robot will move up and down in a random fashion in the specified Axes"""
        ...

    # endregion
    @property
    def base_forces(self): ...
    @property
    def current_forces(self): ...
    def is_feeling_external_force(self, axis: Axis) -> bool:
        current_forces = self.current_forces
        base_forces = self.base_forces
        return current_forces.get(int(log(axis.value) - 1), 0) > base_forces.get(
            int(log(axis.value) - 1), 0
        )

    def rotate(self, axes: Axis, *directions: int) -> None:
        """
        Case Axis.X, then positive = Clockwise,   negative = Counterclockwise
        Case Axis.Y, then positive = Front,    negative = Back
        Case Axis.Z, then positive = Left,      negative = Right
        """
        ...

    def move(self, axes: Axis, *directions: 'int | Literal["random"]') -> None:
        """
        Case Axis.X, then positive = Front,     negative = Back\\
        Case Axis.Y, then positive = Left,      negative = Right\\
        Case Axis.Z, then positive = Up,        negative = Down
        """
        if directions[0] == "random":
            directions = create_random_directions(axes)  # type: ignore

        seperate_axis = axes.get_base_axis()  # type: ignore
        # TODO: move in the direction along the axis

    def get_forces(self, *axes: Axis) -> "dict[Axis, float]":
        # return {x: self.get_force(x) for x in axes}
        ...

    def get_force(self, axis: Axis) -> float: ...
