# %% define abstract InverseKinematics class
import logging
from abc import ABCMeta, abstractmethod
from typing import Optional

import mediapy as media
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R
from sim.inverse_kinematics.decoupled_position_and_orientation import (
    create_inverse_kinematics_function,
)

a1 = 0.42  # Length of first link
a2 = 0.4  # Length of second link
d6 = 0.181  # Distance from wrist to end-effector
_ikf = create_inverse_kinematics_function(a1, a2, d6)


class InverseKinematics(metaclass=ABCMeta):
    """Any implementation should implement the `_step` method"""

    logger = logging.getLogger("InverseKinematicsLogger")

    def __call__(
        self,
        goal: "tuple[float,float,float] | list[float]",
        goalr: "tuple[float,float,float] | list[float] | None" = None,
        init_q: "Optional[list[mujoco.mjtNum]]" = None,
    ) -> "InverseKinematics":
        """This is just an convenience wrapper around `solve`"""
        return self.solve(goal, goalr, init_q)

    def check_ctrl_limits(self) -> None:
        """Check if the joints is under or above its limits"""
        for i in range(len(self.data.qpos)):
            self.data.ctrl[i] = max(
                self.model.jnt_range[i][0],  # type: ignore
                min(self.data.ctrl[i], self.model.jnt_range[i][1]),  # type: ignore
            )

    def check_joint_limits(self) -> None:
        """Check if the joints is under or above its limits"""
        for i in range(len(self.data.qpos)):
            self.data.qpos[i] = max(
                self.model.jnt_range[i][0],  # type: ignore
                min(self.data.qpos[i], self.model.jnt_range[i][1]),  # type: ignore
            )

    def _set_goal(
        self,
        goal,
        goal2: Optional[
            "tuple[float,float,float,float] | tuple[float,float,float] | tuple[tuple[float,float,float], tuple[float,float,float], tuple[float,float,float]]"
        ] = None,
        type: str = "euler",
    ):
        if len(self.data.mocap_pos) == 0:
            return
        self.data.mocap_pos[0] = goal[:]
        if goal2 is not None:
            if type == "euler":
                self.data.mocap_quat[0] = R.from_euler("xyz", goal2).as_quat()  # type: ignore
            elif type == "quat":
                self.data.mocap_quat[0] = goal2[:]  # type: ignore
            elif type == "mat":
                self.data.mocap_quat[0] = R.from_matrix(goal2).as_quat()  # type: ignore

    @property
    def c(self) -> "InverseKinematics":
        """Return a copy of the model"""
        return self.__copy__()

    def __copy__(self):
        cls = self.__class__
        model = cls.__new__(cls)
        model.__dict__.update(self.__dict__)
        model.data = self.data.__copy__()
        model.frames = self.frames.copy()
        return model

    def copies(self, n: int = 1) -> "list[InverseKinematics]":
        copies = []
        for _ in range(n):
            copies.append(self.__copy__())
        return copies

    @property
    def body(self):
        return self.data.body(self.body_id)

    @property
    def jac(self):
        return np.vstack([self.jacp, self.jacr])

    def __init__(
        self,
        model: mujoco.MjModel,
        data: Optional[mujoco.MjData] = None,
        step_size: float = 0.5,
        tol: float = 0.01,
        alpha: float = 0.5,
        body_id: Optional["int | str"] = None,
        jacp: "Optional[np.ndarray[float, np.dtype[np.float64]]]" = None,
        jacr: "Optional[np.ndarray[float, np.dtype[np.float64]]]" = None,
        frame_rate: int = 60,
        init_q: Optional[np.ndarray] = None,
        renderer: mujoco.renderer.Renderer = None,
        log_level: int = logging.WARNING,
    ):
        self.logger.setLevel(log_level)

        self.model = model
        self.data = data or mujoco.MjData(model)
        self.data.qpos = init_q or np.zeros(len(self.data.qpos))
        self.step_size = step_size
        self.tol = tol
        self.alpha = alpha

        # translation jacobian
        self.jacp = jacp or np.zeros((3, model.nv))
        # rotational jacobian
        self.jacr = jacr or np.zeros((3, model.nv))
        self.frame_rate = frame_rate
        self.renderer = renderer or mujoco.renderer.Renderer(
            model, 1080 // 3, 1920 // 3
        )

        self.body_id = body_id or model.body("link7").id
        assert isinstance(self.body_id, int)
        self.scene_option = mujoco.MjvOption()
        self.scene_option.frame = mujoco.mjtFrame.mjFRAME_SITE  # type: ignore
        # Make all sitegroup visiable
        self.scene_option.sitegroup[4] = 1

        self.frames = []
        self.acc = self.data.actuator_force.__copy__()  # type: ignore
        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, self.camera)
        self.camera.distance = 3

    def _get_correct_jac(self, error):
        if error.shape[0] == 3:
            jac = self.jacp
        else:
            jac = self.jac
        return jac

    def _pos_error(
        self, goal: "tuple[float, float, float]"
    ) -> "tuple[float, float, float]":
        return np.subtract(goal, self.body.xpos)

    def _rot_error(self, goal: R) -> "tuple[float, float, float]":
        self.logger.debug(50 * "-")
        self.logger.debug(goal.as_rotvec())
        self.logger.debug(50 * "=")
        return (R.from_quat(self.body.xquat).inv() * goal).as_rotvec()

    def calc_error(
        self, goal_pos: "tuple[float, float, float]", goal_rot: Optional[R] = None
    ) -> "tuple[float, float, float] | tuple[float, float, float, float, float, float]":
        if goal_rot is None:
            pos_error = self._pos_error(goal_pos)
            self._pos_error_log.append(pos_error)
            self._rot_error_log.append((0.0, 0.0, 0.0))
            return pos_error
        pos_error = self._pos_error(goal_pos)
        rot_error = self._rot_error(goal_rot)

        self._pos_error_log.append(pos_error)
        self._rot_error_log.append(rot_error)
        return np.concatenate([pos_error, rot_error])

    def _setup(self, init_q) -> None:
        if init_q is not None:
            self.data.qpos = init_q
        mujoco.mj_forward(self.model, self.data)

    def _record_frame(self) -> None:
        if (
            self.frame_rate is not None
            and len(self.frames) < self.data.time * self.frame_rate
        ):
            self.renderer.update_scene(
                self.data, self.camera, scene_option=self.scene_option
            )
            pixels = self.renderer.render()
            self.frames.append(pixels)

    def copy_error_log(self, into: "list[None|list]") -> "InverseKinematics":
        into.append([(x, y) for x, y in zip(self._pos_error_log, self._rot_error_log)])
        return self

    def copy_torque_log(self, into: "list[None|list]") -> "InverseKinematics":
        into.append([(x, y) for x, y in zip(self._force_log, self._torque_log)])
        return self

    def solve2(
        self,
        goal: "np.ndarray | tuple[float,float,float] | list[float]",
        goalr: "tuple[float,float,float] | list[float] | None" = None,
        init_q: "Optional[list[mujoco.mjtNum]]" = None,
        max_iterations: int = 2000,
    ) -> "InverseKinematics":
        self._pos_error_log = []
        self._rot_error_log = []
        self._force_log = []
        self._torque_log = []
        self._setup(init_q)
        self._set_goal(goal, goalr)

        if goalr is None:
            goalr = [0, 0, 0]  # type: ignore
        goalr = R.from_euler("xyz", goalr)  # type: ignore
        assert isinstance(goalr, R)

        self.show_image()
        old = self.data.qpos.copy()
        mujoco.mj_step(self.model, self.data)
        p_e = (goal - np.array([0, 0, 0.315]))[::-1]
        joint_angles = _ikf(p_e, goalr.as_matrix())

        self.data.ctrl[0] = joint_angles[0]
        self.data.ctrl[1] = joint_angles[1]
        self.data.ctrl[2] = 0
        self.data.ctrl[3] = -joint_angles[2]
        self.data.ctrl[4] = joint_angles[3]
        self.data.ctrl[5] = joint_angles[4]
        self.data.ctrl[6] = joint_angles[5]
        self.check_ctrl_limits()
        counter = 0
        while np.linalg.norm(self.data.qpos - old) > 1e-8 and counter < max_iterations:
            self.logger.info(f"[{counter}/{max_iterations}]")
            counter += 1
            err = self._pos_error(goal)
            self._pos_error_log.append(err)
            self._rot_error_log.append((0.0, 0.0, 0.0))
            old = self.data.qpos.copy()
            mujoco.mj_step(self.model, self.data)
            self._record_frame()
            self._handle_contact_formations()

        return self

    def _handle_contact_formations(self) -> None:
        """Check for contact formations"""
        if self.data.ncon:
            self.logger.info(f"Making {self.data.ncon} contacts")
            for i in range(self.data.ncon):
                force_torque_array = np.zeros(6, dtype=np.float64)
                mujoco.mj_contactForce(self.model, self.data, i, force_torque_array)
                forces = force_torque_array[:3]
                torques = force_torque_array[3:]
                self.logger.debug(
                    f"Direction of force: [{forces[0]:.2f} | {forces[1]:.2f} | {forces[2]:.2f}]"
                )
                self._force_log.append(forces)
                self.logger.debug(
                    f"Direction of torque: [{torques[0]:.2f} | {torques[1]:.2f} | {torques[2]:.2f}]"
                )
                self._torque_log.append(torques)

    def solve(
        self,
        goal: "tuple[float,float,float] | list[float]",
        goalr: "tuple[float,float,float] | list[float] | None" = None,
        init_q: "Optional[list[mujoco.mjtNum]]" = None,
    ) -> "InverseKinematics":
        self._pos_error_log = []
        self._rot_error_log = []
        self._setup(init_q)
        self._set_goal(goal, goalr)

        if goalr is not None:
            goalr = R.from_euler("xyz", goalr)  # type: ignore
            assert isinstance(goalr, R)
        error = self.calc_error(goal, goalr)
        iteration_counter = 0
        self.show_image()
        while np.linalg.norm(error) >= self.tol * 5 and iteration_counter < 2000:
            self.logger.debug(f"Rotation: {R.from_quat(self.body.xquat).as_rotvec()}")
            self._collision_handling()

            mujoco.mj_jac(
                self.model, self.data, self.jacp, self.jacr, goal, self.body_id  # type: ignore
            )

            q = self.data.qpos.copy()

            q += self._step(error) * self.step_size
            self.check_joint_limits()
            self.data.ctrl = q

            # compute forward kinematics
            mujoco.mj_step(self.model, self.data)

            error = self.calc_error(goal, goalr)
            self._record_frame()

            iteration_counter += 1
        if iteration_counter >= 400:
            self.logger.warning("hit max iterations")
        return self

    def _collision_handling(self) -> None:
        if not self.data.ncon:
            return

        force_torque_array = np.zeros(6, dtype=np.float64)
        for i in range(self.data.ncon):
            mujoco.mj_contactForce(self.model, self.data, i, force_torque_array)
            forces = force_torque_array[:3]
            torques = force_torque_array[3:]
            self.logger.debug(f"Forces: {forces}")
            self.logger.debug(f"Total force: {np.linalg.norm(forces)}")
            self.logger.debug(f"Direction of force: {forces / np.linalg.norm(forces)}")

            self.logger.debug(f"Torques: {torques}")
            self.logger.debug(f"Total torque: {np.linalg.norm(torques)}")
            self.logger.debug(
                f"Direction of torque: {torques / np.linalg.norm(torques)}"
            )

    def show_video(self) -> "InverseKinematics":
        assert (
            self.frame_rate is not None and len(self.frames) != 0
        ), "No video has been taken"
        media.show_video(self.frames, fps=self.frame_rate)
        return self

    def show_image(self) -> "InverseKinematics":
        """Create a simple image of the current state"""
        # update to the latest pose
        mujoco.mj_forward(self.model, self.data)
        # update scene settings
        self.renderer.update_scene(
            self.data, self.camera, scene_option=self.scene_option
        )
        # render & show scene
        rendered_image = self.renderer.render()
        media.show_image(rendered_image)
        return self

    @abstractmethod
    def _step(self, error) -> "tuple[float, ...]":
        raise NotImplementedError()
