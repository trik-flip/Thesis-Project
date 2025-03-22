import random
from typing import Dict, Iterable, Literal, Union

from my_types import Axis
from ros_controller import RobotController

base_forces: Iterable[int] = {}
rc = RobotController()
constraints: Dict[Axis, float] = {}
object_is_non_cylindrical: bool


def insert(robot_controller: RobotController):
    # 1
    _1reach_the_hole_plane(robot_controller)
    # 2
    _2search_the_hole(robot_controller)
    # 3
    _3wedge(robot_controller)
    # 4
    if object_is_non_cylindrical:
        _4alignment_rotation_of_peg_and_hole(robot_controller)
    # 5
    _5correct_upward_tilt(robot_controller)
    # 6
    _6insert_peg(robot_controller)
    # 7
    _7detach_hand_grip(robot_controller)
    _8retract_arm(robot_controller)


def _7detach_hand_grip(robot_controller: RobotController):
    raise NotImplementedError()


def _8retract_arm(robot_controller: RobotController):
    raise NotImplementedError()


def set_base_forces() -> None:
    global base_forces
    base_forces = rc.get_forces(Axis.X, Axis.Y, Axis.Z)


def setup() -> None:
    set_base_forces()


def get_current_forces() -> Iterable[float]:
    return rc.get_forces(Axis.X, Axis.Y, Axis.Z)


def get_base_forces() -> Iterable[float]:
    bf = base_forces
    return bf


def create_random_directions(axes: Axis) -> Iterable[Union[Literal[-1], Literal[1]]]:
    directions = []
    for _axis in range(len(axes)):
        direction = 1 if random.random() < 0.5 else -1
        directions.append(direction)
    return directions


def rotate(axes: Axis, *direction: Union[int, Literal["random"]]):
    if direction[0] == "random":
        directions = create_random_directions(axes)  # type: ignore
    rc.rotate(axes, directions)


def _1reach_the_hole_plane(robot_controller: RobotController) -> None:
    "f(z)"
    while not robot_controller.is_feeling_external_force(Axis.Z):
        robot_controller.move(Axis.Z, -1)
    # If we are feeling external forces in the Z-Axis then we've probably hit the hole plane


def _2search_the_hole(robot_controller: RobotController) -> None:
    "f(x)"
    while not robot_controller.is_feeling_external_force(Axis.Y):
        robot_controller.move(Axis.X, "random")


def _3wedge(robot_controller: RobotController) -> None:
    "f(y)"
    while not robot_controller.is_feeling_external_force(Axis.Y):
        robot_controller.move(Axis.Y, "random")


def _4alignment_rotation_of_peg_and_hole(robot_controller: RobotController) -> None:
    "t(z)"
    while not robot_controller.is_feeling_external_force(Axis.Z):
        robot_controller.rotate(Axis.Z, "random")


def _5correct_upward_tilt(robot_controller: RobotController) -> None:
    "t(x)"
    while not robot_controller.is_feeling_external_force(Axis.X):
        robot_controller.rotate(Axis.X, "random")


def _6insert_peg(robot_controller: RobotController) -> None:
    "t(y)"
    while not robot_controller.is_feeling_external_force(Axis.Y):
        robot_controller.rotate(Axis.Y, "random")
