from typing import Optional, overload

import mujoco

from .files import *

__all__ = ["copy_model", "create_model_iiwa"]


@overload
def copy_model(
    model: mujoco.MjModel, data: mujoco.MjData
) -> "tuple[mujoco.MjModel, mujoco.MjData]": ...
@overload
def copy_model(
    model: mujoco.MjModel,
) -> "tuple[mujoco.MjModel, mujoco.MjData]": ...


def copy_model(
    model: mujoco.MjModel, data: Optional[mujoco.MjData] = None
) -> "tuple[mujoco.MjModel, mujoco.MjData]":
    m = model.__copy__()
    d: mujoco.MjData
    if data is not None:
        d = data.__copy__()
    else:
        d = mujoco.MjData(m)
    return m, d


def create_model_iiwa() -> "tuple[mujoco.MjModel, mujoco.MjData]":
    MODEL_IIWA = mujoco.MjModel.from_xml_path(PATH_MODEL_IIWA)
    MODEL_DATA_IIWA = mujoco.MjData(MODEL_IIWA)
    return MODEL_IIWA, MODEL_DATA_IIWA
