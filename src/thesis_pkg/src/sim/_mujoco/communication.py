from enum import Enum, auto

# from thesis_pkg.msg import JointPosition

__all__ = ["transform_values", "translate_forces", "translate_positions"]


class Modes(Enum):
    R2S = auto()
    S2R = auto()


def transform_values(x: float, i: int, mode: Modes) -> float:
    if i not in range(1, 8):
        raise NotImplementedError()
    if mode == Modes.R2S:
        y = _R2S_transform(x, i)
    elif mode == Modes.S2R:
        y = _S2R_transform(x, i)
    return y


def translate_positions(pos, mode: Modes) -> "JointPosition":
    if mode == Modes.S2R:
        res = _S2B_position(pos)
    else:
        res = _B2S_position(pos)
    return res


def translate_forces(frc: "list[float]", mode: Modes) -> "JointPosition":
    if mode == Modes.R2S:
        res = _R2S_forces(frc)
    elif mode == Modes.S2R:
        res = _S2R_forces(frc)
    return res


def _S2B_position(pos) -> "JointPosition":
    res = JointPosition()
    return res


def _B2S_position(pos) -> "JointPosition":
    raise NotImplementedError()


def _S2R_transform(x: float, i: int) -> float:
    if i == 1:
        # Should be in the domain of [-2.97, 2.97]
        y = x
    if i == 2:
        # Should be in the domain of [-2.09, 2.09]
        y = x
    if i == 3:
        # Should be in the domain of [-2.97, 2.97]
        y = x
    if i == 4:
        # Should be in the domain of [-2.09, 2.09]
        y = x
    if i == 5:
        # Should be in the domain of [-2.97, 2.97]
        y = x
    if i == 6:
        # Should be in the domain of [-2.09, 2.09]
        y = x
    if i == 7:
        # Should be in the domain of [-3.05, 3.05]
        y = x
    return y


def _R2S_transform(x: float, i: int) -> float:
    if i == 1:
        # Should be in the domain of [-2.97, 2.97]
        y = x
    if i == 2:
        # Should be in the domain of [-2.09, 2.09]
        y = x
    if i == 3:
        # Should be in the domain of [-2.97, 2.97]
        y = x
    if i == 4:
        # Should be in the domain of [-2.09, 2.09]
        y = x
    if i == 5:
        # Should be in the domain of [-2.97, 2.97]
        y = x
    if i == 6:
        # Should be in the domain of [-2.09, 2.09]
        y = x
    if i == 7:
        # Should be in the domain of [-3.05, 3.05]
        y = x
    return y


def _S2R_forces(frc: "list[float]") -> "JointPosition":
    raise NotImplementedError()


def _R2S_forces(frc: "list[float]") -> "JointPosition":
    jp = JointPosition()
    jp.position.a1 = frc[0]
    jp.position.a2 = frc[1]
    jp.position.a3 = frc[2]
    jp.position.a4 = frc[3]
    jp.position.a5 = frc[4]
    jp.position.a6 = frc[5]
    jp.position.a7 = frc[6]
    return jp
