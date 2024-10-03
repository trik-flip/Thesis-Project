from typing import Callable

import numpy as np
import numpy.typing as npt
from sim.inverse_kinematics.rotation_matrix import (
    _rotation_matrix_y,
    _rotation_matrix_z,
)
from utils.log import logger


def _solve_position_ik(
    postion_end_effector: npt.NDArray[np.float64],
    a1: float,
    a2: float,
    d6: float,
) -> "tuple[float,float,float]":
    """
    Solves for the first three joint angles (position IK)

    p_e: Desired end-effector position (3x1) [x_e, y_e, z_e]
    a1: Length of the first link
    a2: Length of the second link
    d6: Distance from the wrist center to the end-effector
    """
    # Wrist center calculation
    # For simplicity, aligned along z-axis, Assume the z-axis of wrist is pointing downward
    # Assume: z-axis of end-effector is aligned with global z
    orientation_wrist = np.array([0, 0, -1])
    position_wrist_center = postion_end_effector - d6 * orientation_wrist
    (x_pos_wrist, y_pos_wrist, z_pos_wrist) = position_wrist_center
    # Note: Solve for theta_1, Rotation about the base
    theta_1: float = np.arctan2(y_pos_wrist, x_pos_wrist)

    # r is the projection of wrist center onto the xy-plane, AKA distance from base to wrist in xy-plane
    r = np.sqrt(x_pos_wrist**2 + y_pos_wrist**2)
    # Distance along z-axis to wrist
    s = z_pos_wrist

    # Solve for theta_2 and theta_3 using geometric methods
    # Solve for theta_3 using the law of cosines
    cos_theta_3 = (r**2 + s**2 - a1**2 - a2**2) / (2 * a1 * a2)

    # Elbow up configuration
    sin_theta_3 = np.sqrt(1 - cos_theta_3**2)

    theta_3: float = np.arctan2(sin_theta_3, cos_theta_3)
    theta_2: float = np.arctan2(s, r) - np.arctan2(
        (a2 * sin_theta_3), (a1 + a2 * cos_theta_3)
    )

    return theta_1, theta_2, theta_3


def _solve_orientation_ik(
    orientation_end_effector: npt.NDArray[np.float64],
    orientation_arm: npt.NDArray[np.float64],
) -> "tuple[float,float,float]":
    """
    Solves for the wrist angles (orientation IK) theta_4, theta_5, theta_6

    orientation_end_effector: Desired end-effector rotation matrix (3x3)
    orientation_arm: Rotation matrix from base to wrist (3x3) due to the first three joints
    """
    # Note: Calculate the wrist orientation by removing arm rotation
    # Question: Is this in global space or with respect to the arm?
    orientation_wrist = np.linalg.inv(orientation_arm).dot(orientation_end_effector)

    # Extract theta_5 (pitch) from orientation_wrist
    theta_5: float = np.arctan2(
        (np.sqrt(orientation_wrist[0, 2] ** 2 + orientation_wrist[1, 2] ** 2)),
        orientation_wrist[2, 2],
    )

    # Extract theta_4 (yaw)
    theta_4: float = np.arctan2(orientation_wrist[1, 2], orientation_wrist[0, 2])
    # Extract theta_6 (roll)
    theta_6: float = np.arctan2(orientation_wrist[2, 1], -orientation_wrist[2, 0])

    return theta_4, theta_5, theta_6


def inverse_kinematics(
    end_effector_position: npt.NDArray[np.float64],
    end_effector_rotation: npt.NDArray[np.float64],
    a1: float,
    a2: float,
    d6: float,
) -> "list[float]":
    """
    Full inverse kinematics solver for a manipulator with a spherical wrist.

    end_effector_position: Desired end-effector position [x_e, y_e, z_e]
    end_effector_rotation: Desired end-effector rotation matrix (3x3)
    a1, a2: Link lengths
    d6: Distance from wrist center to end-effector
    """
    logger.debug("inverse_kinematics(input):")
    logger.debug(f"end_effector_position: {end_effector_position}")
    logger.debug(f"end_effector_rotation: {end_effector_rotation}")
    logger.debug(f"a1: {a1}")
    logger.debug(f"a2: {a2}")
    logger.debug(f"d6: {d6}")

    theta_1, theta_2, theta_3 = _solve_position_ik(end_effector_position, a1, a2, d6)

    # Calculate the arm rotation matrix R_arm
    arm_rotation = _calc_arm_orientation(theta_1, theta_2, theta_3)

    # Solve for the wrist angles (orientation IK)
    theta_4, theta_5, theta_6 = _solve_orientation_ik(
        end_effector_rotation, arm_rotation
    )

    # Return all joint angles
    logger.debug("inverse_kinematics(output):")
    logger.debug([theta_1, theta_2, theta_3, theta_4, theta_5, theta_6])
    return [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]


def _calc_arm_orientation(
    theta_1: float, theta_2: float, theta_3: float
) -> npt.NDArray[np.float64]:
    return (
        _rotation_matrix_z(theta_1)
        .dot(_rotation_matrix_y(theta_2))
        .dot(_rotation_matrix_y(theta_3))
    )


def create_inverse_kinematics_function(
    a1: float, a2: float, d6: float
) -> Callable[[npt.NDArray[np.float64], npt.NDArray[np.float64]], "list[float]"]:
    """Create a inverse kinematics function with fixed parameters"""

    def inner_func(
        end_effector_position: npt.NDArray[np.float64],
        end_effector_rotation: npt.NDArray[np.float64],
    ) -> "list[float]":
        """
        Full inverse kinematics solver for a manipulator with a spherical wrist.
        p_e: Desired end-effector position [x_e, y_e, z_e]
        R_e: Desired end-effector rotation matrix (3x3)
        """

        return inverse_kinematics(
            end_effector_position, end_effector_rotation, a1, a2, d6
        )

    return inner_func
