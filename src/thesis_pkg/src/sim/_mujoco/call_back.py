from typing import Dict

import mujoco
from utils.decorators import create_collection

from .communication import Modes, transform_values

# from thesis_pkg.msg import JointPosition


__all__ = [
    "callback",
    "key_callback",
]
_key_registery = {}


def _preprocess(key: "str|int") -> int:
    if isinstance(key, str):
        key = ord(key)
    return key


def key_callback(state: "dict[str, dict[int,int]|bool]", keycode: int) -> None:
    if isinstance(state["acc"], bool):
        return
    if keycode not in _key_registery:
        print(f"{keycode}: {chr(keycode)}")
    else:
        _key_registery[keycode](state)


def _register_toggle(key: str, toggle: str) -> None:
    def inner_func(state: "dict[str, bool]") -> None:
        state[toggle] = not state[toggle]

    _key_registery[ord(key)] = inner_func


_register_toggle(" ", "paused")
_register_toggle("Q", "close")
_register_toggle("C", "copy")
register_key = create_collection(
    _key_registery, _preprocess, check=lambda key: isinstance(key, int)
)


STATE_DICT_TYPE = Dict[str, Dict[int, int]]


@register_key(265)
def _move_up_action(state: STATE_DICT_TYPE) -> None:
    print("Move up")
    state["acc"][0] = True
    state["acc"][1] += 1


@register_key(262)
def _move_right_action(state: STATE_DICT_TYPE) -> None:
    print("Move right")
    state["acc"][0] = True
    state["acc"][2] += 1


@register_key(264)
def _move_down_action(state: STATE_DICT_TYPE) -> None:
    print("Move down")
    state["acc"][0] = True
    state["acc"][1] -= 1


@register_key(263)
def _move_left_action(state: STATE_DICT_TYPE) -> None:
    print("Move left")
    state["acc"][0] = True
    state["acc"][2] -= 1


def callback(d: mujoco.MjData, jp: "JointPosition") -> None:
    # Step 1: Retrieve Data
    jp = jp.position
    # Step 2: Pass the sensor data into the simulation
    d.ctrl[0] = transform_values(jp.a1, 1, Modes.R2S)
    d.ctrl[1] = transform_values(jp.a2, 2, Modes.R2S)
    d.ctrl[2] = transform_values(jp.a3, 3, Modes.R2S)
    d.ctrl[3] = transform_values(jp.a4, 4, Modes.R2S)
    d.ctrl[4] = transform_values(jp.a5, 5, Modes.R2S)
    d.ctrl[5] = transform_values(jp.a6, 6, Modes.R2S)
    d.ctrl[6] = transform_values(jp.a7, 7, Modes.R2S)
