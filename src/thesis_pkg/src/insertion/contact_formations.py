"""
# Insertion Pseudocode
1) Reaching the hole plane
Current constraints: None
Target constraint: (+fz , 1.5N)
Implicit CF targets: (CF1) point or edge on face
Additional motions:
- Lateral exploration within the workspace area, ran-
domly selecting x/y direction within workspace limits.
- In-hand manipulation to tilt the object toward the
direction of the lateral motion, to ensure an edge/face
contact, and avoid a face/face contact.
2) Searching for the hole
Current constraints: (+fz , 1.5N)
Target constraint: (+fx, 0.7N)
Implicit CF target: (CF2) 3-point contact with hole
Additional motions: Lateral exploration as previous step.
3) Wedging
Current constraints: (+fz , 1.5N), (+fx, 0.7N)
Target constraint: (+fy , 0.7N)
Implicit CF target: (CF3) 4-point contact with hole
4) Rotational alignment of peg and hole
Current constraints: (+fz , 1.5N), (+fx, 0.7N), (+fy ,
0.7N)
Target constraint: (+τz , 0.1N/m)
Implicit CF target: (CF4) hinge-type contact
Note: This step applies only to non-cylindrical objects.
5) Correcting upward tilt
Current constraints: (+fz , 1.5N)
Target constraint: (+fx, 0N), (+fy , 0N)
Implicit CF target: (CF5) Prismatic joint-type contact
Additional motion: Rotation around x-and y-axes
to minimize the accumulated angle between the object
and the hand due to fingertip slip during the previous
steps. This rotation is performed both with in-hand
manipulation and arm motions.
Note: The lateral forces are now minimized to avoid
jamming the object. This also helps centering the object
in the hole if the peg rotation is not perfectly centered
on the object's center of mass. The angle information is
provided by extracting the 3D pose of a marker placed on
the surface of the object as seen from the palm camera.
6) Inserting peg
Current constraints: (+fz , 1.5N), (+fx, 0N), (+fy , 0N)
Exit condition: When the fingertips start touching the
hole surface or the object hits the hole bottom, we
switch to disengagement. This can be detected as a
sharp increase in fz .
Note: We have already reached the CF state that enables
peg insertion, thus the system maintains it while the final
free DOF, i.e., the z-axis translation, is controlled to
perform the final insertion motion.
7) Detaching hand grip and retracting arm
Action: The hand opens and the arm returns to its origin
position in an open loop motion
"""

# region:Imports
import random
from typing import Dict, Iterable, Literal, Union, overload

from my_ros.controller import RobotController
from my_types.axis import Axis

# endregion
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


# region: Implemented
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


# endregion
