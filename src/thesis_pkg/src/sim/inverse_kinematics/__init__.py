# %% import dependendcys
from enum import Enum
from typing import Optional

import mujoco
import mujoco.renderer
import numpy as np

from .GaussNewton import GaussNewton
from .GradientDescent import GradientDescent
from .InverseKinematics import InverseKinematics
from .LevenbegMarquardt import LevenbegMarquardt

pi = np.pi


# %% init camera
MODEL = "kuka_iiwa_14"
JOINT = "link7"
STARTING_POS = [0, 0, 0, 0, 0, 0, 0]

GOAL = [0, 0.5, 0.5]

xml = f"/home/philip/Code/OtherProjects/mujoco_menagerie/{MODEL}/scene2.xml"

model = mujoco.MjModel.from_xml_path(xml)


# %%


def get_IK(
    Model: "type[InverseKinematics] | Models" = LevenbegMarquardt,
    m: Optional[mujoco.MjModel] = None,
    d: Optional[mujoco.MjData] = None,
    joint: str = JOINT,
) -> InverseKinematics:
    if m is None:
        m = model
    if d is None:
        d = mujoco.MjData(m)

    body_id = m.body(joint).id
    step_size = 0.5
    tol = 0.01
    alpha = 0.5
    if d is not None:
        mujoco.mj_resetData(m, d)
    if isinstance(Model, Models):
        Model = Model.value
    ik = Model(m, d, step_size, tol, alpha, body_id, frame_rate=60)  # type: ignore
    # reset qpos to initial value
    if d is not None:
        mujoco.mj_resetDataKeyframe(m, d, 1)
    return ik


# %%
class Models(Enum):
    """Enumation of all available models"""

    GD = GradientDescent
    """GradientDescent"""
    GN = GaussNewton
    """GaussNewton"""
    LM = LevenbegMarquardt
    """LevenbegMarquardt"""
