from typing import Collection, Dict, Iterable, Optional

import mujoco
import mujoco.viewer
from my_ros.controller import RobotController
from my_types.axis import Axis

__all__ = ["mend", "calc", "next_command", "examine", "get_flags", "simulators"]


# region: Internal fuctions
def mend(model: mujoco.MjModel, data: Optional[mujoco.MjData]) -> None: ...
def calc(model: mujoco.MjModel, data: Optional[mujoco.MjData]) -> None: ...
def view(model: mujoco.MjModel, data: Optional[mujoco.MjData]) -> None: ...


# endregion


# region: Communicational functions
def examine(model: mujoco.MjModel, data: Optional[mujoco.MjData], new_data) -> None: ...
def next_command(model: mujoco.MjModel, data: Optional[mujoco.MjData]) -> "Command": ...


# endregion
# region: utils
from ._mujoco.config import get_flags

# endregion
from ._mujoco.util import create_model_iiwa

# endregion
# region: enduser functions
from ._mujoco.viewer import *


class SimController(RobotController):
    model: mujoco.MjModel
    data: mujoco.MjData

    def __init__(
        self,
    ) -> None:
        """Link a simulation model to the controller"""
        self.model, self.data = create_model_iiwa()

    def rotate(self, axes: Axis, directions: Iterable[int]) -> None:
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                self.data.ctrl[7] += directions[0]
                viewer.sync()

    def move(self, axes: Axis, directions: Iterable[int]) -> None: ...
    def get_force(self, axis: Axis) -> float: ...
    def get_forces(self, *axes: Axis) -> Dict[Axis, float]:
        return {x: self.get_force(x) for x in axes}
