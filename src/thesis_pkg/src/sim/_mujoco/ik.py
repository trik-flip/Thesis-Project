# # %%  Inverse Kinematics
# from abc import ABCMeta, abstractmethod
# from typing import Optional

# import mediapy as media
# import mujoco
# import mujoco.renderer
# import numpy as np
# from scipy.spatial.transform import Rotation as R

# from thesis_pkg.src.sim._mujoco.LevenbegMarquardtIK import LevenbegMarquardtIK

# pi = np.pi
# # %%
# MODEL = "kuka_iiwa_14"
# JOINT = "link7"
# STARTING_POS = [0, 0, 0, 0, 0, 0, 0]

# # MODEL = "universal_robots_ur5e"
# # JOINT = "wrist_3_link"
# # STARTING_POS = [3 * pi / 2, -pi / 2, pi / 2, 3 * pi / 2, 3 * pi / 2, 0]

# GOAL = [0, 0.5, 0.5]

# xml = f"/home/philip/Code/OtherProjects/mujoco_menagerie/{MODEL}/scene2.xml"

# model = mujoco.MjModel.from_xml_path(xml)
# data = mujoco.MjData(model)
# renderer = mujoco.renderer.Renderer(model, 1080 // 3, 1920 // 3)

# camera = mujoco.MjvCamera()
# mujoco.mjv_defaultFreeCamera(model, camera)
# camera.distance = 3


# # %%
# # Put a position of the joints to get a test point
# def main():
#     data.qpos = STARTING_POS

#     # Initial joint position
#     qpos0 = data.qpos.copy()

#     # Step the simulation.
#     mujoco.mj_forward(model, data)

#     # Use the last piece as an "end effector" to get a test point in cartesian
#     # coordinates

#     target = data.body(JOINT).xpos
#     print("Target =>", target)

#     # Plot results
#     print("Results")
#     mujoco.mj_resetDataKeyframe(model, data, 1)
#     mujoco.mj_forward(model, data)
#     init_point = data.body(JOINT).xpos.copy()
#     renderer.update_scene(data, camera)
#     init_plot = renderer.render()

#     data.qpos = qpos0
#     mujoco.mj_forward(model, data)
#     result_point = data.body(JOINT).xpos.copy()
#     renderer.update_scene(data, camera)
#     result_plot = renderer.render()

#     print("initial point =>", init_point)
#     print("Desire point =>", result_point, "\n")

#     images = {
#         "Initial position": init_plot,
#         " Desire end effector position": result_plot,
#     }

#     media.show_images(images)


# # %%


# def circle(t: float, r: float, h: float, k: float, f: float) -> np.ndarray:
#     """Return the (x, y) coordinates of a circle with radius r centered at (h, k)
#     as a function of time t and frequency f."""
#     x = r * np.cos(2 * np.pi * f * t) + h
#     y = r * np.sin(2 * np.pi * f * t) + k
#     z = 0.5
#     return np.array([x, y, z])


# # class IK(metaclass=ABCMeta):
# #     def _set_goal(self, goal, goal2=None, type="euler"):
# #         self.data.mocap_pos[0] = goal[:]
# #         if goal2 is not None:
# #             if type == "euler":
# #                 self.data.mocap_quat[0] = R.from_euler("xyz", goal2).as_quat()
# #             elif type == "quat":
# #                 self.data.mocap_quat[0] = goal2[:]
# #             elif type == "mat":
# #                 self.data.mocap_quat[0] = R.from_matrix(goal2).as_quat()

# #     def __copy__(self):
# #         cls = self.__class__
# #         model = cls.__new__(cls)
# #         model.__dict__.update(self.__dict__)
# #         model.data = self.data.__copy__()
# #         model.frames = self.frames.copy()
# #         return model

# #     @property
# #     def body(self):
# #         return self.data.body(self.body_id)

# #     @property
# #     def jac(self):
# #         return np.vstack([self.jacp, self.jacr])

# #     def __init__(
# #         self,
# #         model: mujoco.MjModel,
# #         data: mujoco.MjData,
# #         step_size: float,
# #         tol: float,
# #         alpha: float,
# #         body_id: "int | str",
# #         jacp: "Optional[np.ndarray[float, np.dtype[np.float64]]]" = None,
# #         jacr: "Optional[np.ndarray[float, np.dtype[np.float64]]]" = None,
# #         frame_rate: int = 60,
# #         init_q=None,
# #         renderer=renderer,
# #     ):
# #         if jacp is None:
# #             jacp = np.zeros((3, model.nv))  # translation jacobian
# #         if jacr is None:
# #             jacr = np.zeros((3, model.nv))  # rotational jacobian
# #         if init_q is None:
# #             init_q = np.zeros(len(data.qpos))

# #         self.model = model
# #         self.data = data
# #         self.data.qpos = init_q
# #         self.step_size = step_size
# #         self.tol = tol
# #         self.alpha = alpha

# #         self.jacp = jacp
# #         self.jacr = jacr
# #         self.frame_rate = frame_rate
# #         self.renderer = renderer
# #         self.body_id = body_id
# #         self.scene_option = mujoco.MjvOption()
# #         self.scene_option.frame = mujoco.mjtFrame.mjFRAME_SITE
# #         # Make all sitegroup visiable
# #         self.scene_option.sitegroup[4] = 1

# #         self.frames = []
# #         self.acc = self.data.actuator_force.__copy__()

# #     def _get_correct_jac(self, error):
# #         if error.shape[0] == 3:
# #             jac = self.jacp
# #         else:
# #             jac = self.jac
# #         return jac

# #     def _pos_error(self, goal):
# #         return np.subtract(goal, self.body.xpos)

# #     def _rot_error(self, goal):
# #         return (R.from_quat(self.body.xquat).inv() * goal).as_rotvec()

# #     def calc_error(self, goal1, goal2=None):
# #         if goal2 is None:
# #             return self._pos_error(goal1)
# #         return np.concatenate([self._pos_error(goal1), self._rot_error(goal2)])

# #     def _setup(self, init_q):
# #         if init_q is not None:
# #             self.data.qpos = init_q
# #         mujoco.mj_forward(self.model, self.data)

# #     def _setupq(self, init_q):
# #         if init_q is not None:
# #             self.data.qpos = init_q
# #         mujoco.mj_forward(self.model, self.data)

# #     def _record_frame(self) -> None:
# #         if (
# #             self.frame_rate is not None
# #             and len(self.frames) < self.data.time * self.frame_rate
# #         ):
# #             self.renderer.update_scene(
# #                 self.data, camera, scene_option=self.scene_option
# #             )
# #             pixels = self.renderer.render()
# #             self.frames.append(pixels)

# #     def solve(
# #         self,
# #         goal: "tuple[float,float,float] | list[float]",
# #         goalr: "tuple[float,float,float] | list[float] | None" = None,
# #         init_q: "Optional[list[mujoco.mjtNum]]" = None,
# #     ) -> "IK":
# #         self._setup(init_q)
# #         self._set_goal(goal, goalr)

# #         if goalr is not None:
# #             goalr = R.from_euler("xyz", goalr)
# #         error = self.calc_error(goal, goalr)

# #         iteration_counter = 0
# #         self.show_image()
# #         last_error = np.linalg.norm(error)
# #         while np.linalg.norm(error) >= self.tol * 5 and iteration_counter < 2000:
# #             # calculate jacobian
# #             mujoco.mj_jac(
# #                 self.model, self.data, self.jacp, self.jacr, goal, self.body_id
# #             )

# #             q = self.data.qpos.copy()

# #             q += self._step(error) * self.step_size
# #             check_joint_limits(self.data.qpos, self.model)
# #             self.data.ctrl = q

# #             # compute forward kinematics
# #             mujoco.mj_step(self.model, self.data)

# #             # calculate new error

# #             last_error = np.linalg.norm(error)
# #             error = self.calc_error(goal, goalr)
# #             # if np.linalg.norm(error) > last_error:
# #             #     break
# #             # Render and save frames.
# #             self._record_frame()

# #             iteration_counter += 1
# #         if iteration_counter >= 2000:
# #             print("hit max iterations")
# #         return self

# #     def show_video(self):
# #         if self.frame_rate is not None and len(self.frames) != 0:
# #             media.show_video(self.frames, fps=self.frame_rate)
# #         else:
# #             print("No video has been taken")

# #     def show_image(self):
# #         # update to the latest pose
# #         mujoco.mj_forward(self.model, self.data)
# #         # update scene settings
# #         self.renderer.update_scene(self.data, camera, scene_option=self.scene_option)
# #         # render & show scene
# #         media.show_image(self.renderer.render())

# #     # def is_feeling_external_force(self, axis: Axis):
# #     #     for i in range(self.data.ncon):
# #     #         self.data.contact[i]
# #     @abstractmethod
# #     def _step(self, error):
# #         raise NotImplementedError()
