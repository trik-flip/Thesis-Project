import numpy as np
import numpy.typing as npt
from numpy import cos, sin


def _rotation_matrix_x(theta: float) -> npt.NDArray[np.float64]:
    """Rotation matrix around X-axis"""
    return np.array(
        [[1, 0, 0], [0, cos(theta), -sin(theta)], [0, sin(theta), cos(theta)]]
    )


def _rotation_matrix_y(theta: float) -> npt.NDArray[np.float64]:
    """Rotation matrix around Y-axis"""
    return np.array(
        [[cos(theta), 0, sin(theta)], [0, 1, 0], [-sin(theta), 0, cos(theta)]]
    )


def _rotation_matrix_z(theta: float) -> npt.NDArray[np.float64]:
    """Rotation matrix around Z-axis"""
    return np.array(
        [[cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0], [0, 0, 1]]
    )
