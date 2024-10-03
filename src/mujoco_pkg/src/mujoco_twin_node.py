#!/usr/bin/env python3
import atexit
from collections.abc import Callable
import queue
import threading
import time
from functools import partial
from typing import Optional

import glfw
import mujoco
import mujoco.viewer
import rospy
from mujoco import _simulate

from mujoco_pkg.msg import JointPosition

model_path = (
    "/mnt/c/Users/flipp/Documents/School/Thesis/PIH/models/kuka_iiwa_14/iiwa14.xml"
)
cube_path = "/mnt/c/Users/flipp/Documents/School/Thesis/PIH/models/cube/cube.xml"

m = mujoco.MjModel.from_xml_path(model_path)
d = mujoco.MjData(m)

"""
When settings mujoco settings
options = mujoco.MjvOption()
mujoco.mjv_defaultOption(options)

options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT.value] = True
options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE.value] = True
options.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT.value] = True
"""


# Real 2 Simulation
def R2S_transform_values(x: float, i: int) -> float:
    if i not in range(1, 8):
        raise NotImplementedError()
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


# Simulation 2 Real
def S2R_transform_values(x: float, i: int) -> float:
    if i not in range(1, 8):
        raise NotImplementedError()
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


def create_callback(d: mujoco.MjData) -> "Callable[[JointPosition], None]":
    def call_back(jp: JointPosition) -> None:
        # Step 1: Retrieve Data
        jp = jp.position
        # Step 2: Pass the sensor data into the simulation
        d.ctrl[0] = R2S_transform_values(jp.a1, 1)
        d.ctrl[1] = R2S_transform_values(jp.a2, 2)
        d.ctrl[2] = R2S_transform_values(jp.a3, 3)
        d.ctrl[3] = R2S_transform_values(jp.a4, 4)
        d.ctrl[4] = R2S_transform_values(jp.a5, 5)
        d.ctrl[5] = R2S_transform_values(jp.a6, 6)
        d.ctrl[6] = R2S_transform_values(jp.a7, 7)

    return call_back


def key_callback(state: "dict[str, dict[int,int]|bool]", keycode: int) -> None:
    if isinstance(state["acc"], bool):
        return
    if chr(keycode) == " ":
        state["paused"] = not state["paused"]
    elif chr(keycode) == "Q":
        state["close"] = not state["close"]
    elif chr(keycode) == "C":
        state["copy"] = not state["copy"]
    elif keycode == 265:
        print("Move up")
        state["acc"][0] = True
        state["acc"][1] += 1
    elif keycode == 262:
        print("Move right")
        state["acc"][0] = True
        state["acc"][2] += 1
    elif keycode == 264:
        print("Move down")
        state["acc"][0] = True
        state["acc"][1] -= 1
    elif keycode == 263:
        print("Move left")
        state["acc"][0] = True
        state["acc"][2] -= 1
    else:
        print(f"{keycode}: {chr(keycode)}")


def S2R_translate_position(_) -> JointPosition:
    jp = JointPosition()
    return jp


def S2R_translate_force(frc: "list[float]") -> JointPosition:
    jp = JointPosition()
    jp.position.a1 = frc[0]
    jp.position.a2 = frc[1]
    jp.position.a3 = frc[2]
    jp.position.a4 = frc[3]
    jp.position.a5 = frc[4]
    jp.position.a6 = frc[5]
    jp.position.a7 = frc[6]
    return jp


def test_sim(ros_namespace: str = "digital_twin") -> None:
    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            step_start = time.time()

            mujoco.mj_step(m, d)
            viewer.sync()

            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


def sim3() -> None:
    model = mujoco.MjModel.from_xml_path(cube_path)
    data = mujoco.MjData(model)
    handle = launch_passive(model, data)
    while handle.is_running():
        mujoco.mj_step(model, data)
        handle.sync()


def launch_passive(
    model: mujoco.MjModel = mujoco.MjModel.from_xml_path(cube_path),
    data: Optional[mujoco.MjData] = None,
    *,
    key_callback: Optional[mujoco.viewer.KeyCallbackType] = None,
    show_left_ui: bool = True,
    show_right_ui: bool = True,
) -> mujoco.viewer.Handle:
    """Launches a passive Simulate GUI without blocking the running thread."""
    if not isinstance(model, mujoco.MjModel):
        raise ValueError(f"`model` is not a mujoco.MjModel: got {model!r}")
    if data is None:
        data = mujoco.MjData(model)
    if not isinstance(data, mujoco.MjData):
        raise ValueError(f"`data` is not a mujoco.MjData: got {data!r}")
    if key_callback is not None and not callable(key_callback):
        raise ValueError(f"`key_callback` is not callable: got {key_callback!r}")

    mujoco.mj_forward(model, data)
    handle_return = queue.Queue(1)

    thread = threading.Thread(
        target=launch_sim,
        args=(model, data),
        kwargs=dict(
            run_physics_thread=False,
            handle_return=handle_return,
            key_callback=key_callback,
            show_left_ui=show_left_ui,
            show_right_ui=show_right_ui,
        ),
    )
    thread.daemon = True
    thread.start()

    return handle_return.get()


def launch_sim(
    model: Optional[mujoco.MjModel] = None,
    data: Optional[mujoco.MjData] = None,
    *,
    run_physics_thread: bool = True,
    loader: Optional[mujoco.viewer._InternalLoaderType] = None,
    handle_return: Optional["queue.Queue[mujoco.viewer.Handle]"] = None,
    key_callback: Optional[mujoco.viewer.KeyCallbackType] = None,
    show_left_ui: bool = True,
    show_right_ui: bool = True,
) -> None:
    """Internal API, so that the public API has more readable type annotations."""

    _simulate.set_glfw_dlhandle(glfw._glfw._handle)
    _Simulate = _simulate.Simulate

    if model is None and data is not None:
        raise ValueError("mjData is specified but mjModel is not")
    elif callable(model) and data is not None:
        raise ValueError(
            "mjData should not be specified when an mjModel loader is used"
        )
    elif loader is not None and model is not None:
        raise ValueError("model and loader are both specified")
    elif run_physics_thread and handle_return is not None:
        raise ValueError("run_physics_thread and handle_return are both specified")

    if loader is None and model is not None:

        def _loader(m=model, d=data) -> "tuple[mujoco.MjModel, mujoco.MjData]":
            if d is None:
                d = mujoco.MjData(m)
            return m, d

        loader = _loader

    cam = mujoco.MjvCamera()
    opt = mujoco.MjvOption()
    pert = mujoco.MjvPerturb()
    user_scn = (
        mujoco.MjvScene(model, _Simulate.MAX_GEOM)
        if model and not run_physics_thread
        else None
    )

    simulate = _Simulate(cam, opt, pert, user_scn, run_physics_thread, key_callback)

    simulate.ui0_enable = show_left_ui
    simulate.ui1_enable = show_right_ui

    # Initialize GLFW if not using mjpython.
    if mujoco.viewer._MJPYTHON is None:
        if not glfw.init():
            raise mujoco.FatalError("could not initialize GLFW")
        atexit.register(glfw.terminate)

    notify_loaded: "Optional[Callable[[], None]]" = None
    if handle_return:

        notify_loaded = lambda: handle_return.put_nowait(
            mujoco.viewer.Handle(simulate, cam, opt, pert, user_scn)
        )

    if run_physics_thread:
        side_thread = threading.Thread(
            target=mujoco.viewer._physics_loop, args=(simulate, loader)
        )
    else:
        side_thread = threading.Thread(
            target=mujoco.viewer._reload, args=(simulate, loader, notify_loaded)
        )
    atexit.register(simulate.exit)

    side_thread.start()
    simulate.render_loop()
    atexit.unregister(simulate.exit)
    side_thread.join()
    simulate.destroy()


def sim2() -> None:
    print("init")
    model: Optional[mujoco.MjModel] = mujoco.MjModel.from_xml_path(model_path)
    data: Optional[mujoco.MjData] = mujoco.MjData(model)
    cam: Optional[mujoco.MjvCamera] = mujoco.MjvCamera()
    opt: Optional[mujoco.MjvOption] = mujoco.MjvOption()
    scn: Optional[mujoco.MjvScene] = mujoco.MjvScene()
    con: Optional[mujoco.MjrContext] = mujoco.MjrContext()
    pert: Optional[mujoco.MjvPerturb] = mujoco.MjvPerturb()

    print("glfw init")
    glfw.init()
    window = glfw.create_window(1200, 900, "Demo", None, None)
    glfw.make_context_current(window)
    glfw.swap_interval(1)

    print("mujoco default")
    mujoco.mjv_defaultCamera(cam)
    mujoco.mjv_defaultPerturb(pert)
    mujoco.mjv_defaultOption(opt)
    # mujoco.mjr_defaultContext(con)
    scn = mujoco.MjvScene(model, 1000)

    # mujoco.mjr_makeContext(m, con, mujoco.mjtFontScale.mjFONTSCALE_100.value)

    assert data is not None
    print(f"start loop with {not glfw.window_should_close(window)}")
    while not glfw.window_should_close(window):
        simstart = data.time
        while data.time - simstart < 1.0 / 60.0:
            print("Step")
            mujoco.mj_step(model, data)

        print("create viewport/window")
        fb = glfw.get_framebuffer_size(window)

        print("update scene")
        mujoco.mjv_updateScene(
            model, data, opt, pert, cam, mujoco.mjtCatBit.mjCAT_ALL.value, scn
        )
        print("render")
        try:
            viewport = mujoco.MjrRect(0, 0, *fb)
            mujoco.mjr_render(viewport, scn, con)
        except Exception as e:
            print(e)

        print("swap")
        glfw.swap_buffers(window)

        print("poll")
        glfw.poll_events()
    print("End of loop")

    window = glfw.create_window(1200, 900, "Demo2", None, None)
    ctx1 = mujoco.GLContext(*fb)
    ctx1.make_current()

    cam = mujoco.MjvCamera()
    opt = mujoco.MjvOption()
    scn = mujoco.MjvScene()
    con = mujoco.MjrContext()

    mujoco.mjv_defaultCamera(cam)
    # mujoco.mjv_defaultPerturb(pert)
    mujoco.mjv_defaultOption(opt)
    # mujoco.mjv_defaultScene(opt)

    viewport = mujoco.MjrRect(0, 0, *fb)
    glfw.get_framebuffer_size(window)
    mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
    mujoco.mjr_render(viewport, scn, con)
    # mujoco.viewer.launch_passive(m, d)
    mujoco.mjv_freeScene(scn)
    mujoco.mjr_freeContext(con)


def sim(
    ros_namespace: str = "digital_twin",
    model: mujoco.MjModel = m,
) -> None:
    data = mujoco.MjData(model)
    state = {
        "paused": False,
        "close": False,
        "copy": False,
        "acc": {
            0: False,
            1: data.ctrl[0],
            2: data.ctrl[1],
            3: data.ctrl[2],
            4: data.ctrl[3],
            5: data.ctrl[4],
            6: data.ctrl[5],
            7: data.ctrl[6],
        },
    }
    with mujoco.viewer.launch_passive(
        model,
        data,
        key_callback=partial(key_callback, state),
    ) as viewer:
        print("creating viewers")
        viewers: list[tuple[mujoco.MjModel, mujoco.MjData, mujoco.viewer.Handle]] = []
        print("init ros")
        rospy.init_node("mujoco_listener", anonymous=True)
        print("init sub")
        call_back = create_callback(d)
        rospy.Subscriber(f"{ros_namespace}/joints", JointPosition, call_back)
        print("init pub")
        pub = rospy.Publisher(f"{ros_namespace}/forces", JointPosition, queue_size=10)
        print(f"{dir(model)}")
        while viewer.is_running():
            step_start = data.time
            # mj_step can be replaced with code that also evaluates a policy and applies a control signal before stepping the physics.
            mujoco.mj_step(model, data)
            # * Step 3.a: Retrieve data from simulation, this is implied since we're able to get data from the simulator every timestep
            # * Step 4.a: Pass on reconstructed high-level info
            pub.publish(S2R_translate_force(data.actuator_force))
            # jp.position.a1 = m.sensordata[0]
            # * Step 5.a: Pass on current search-space possibilities

            # * Step 6.a: Command robot to execute movement

            # * Step 3.b: Split in multiple hypothesis
            # * Step 4.c: Pass contact and pose info
            # * Step 5.c: Pass probable models on
            # * Step 6.c: Pass “Best” model on

            # * Step 7: Command to replace mode

            """
            # Example modification of a viewer option: toggle contact points every two seconds.
            with viewer.lock():
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)
            """
            if not state["paused"]:
                mujoco.mj_step(model, data)
                viewer.sync()
            if state["copy"]:
                print("Start copy")
                new_m, new_d = copy_model_and_data(model, data)
                print("copied")
                new_viewer = open_async_viewer(new_m, new_d)
                print("opened new viewer")
                viewers.append((new_m, new_d, new_viewer))
                print("listed new viewer")
            if state["close"]:
                viewer.close()
            if state["acc"][0]:
                state["acc"][0] = False
                data.ctrl[0] = state["acc"][1]
                data.ctrl[1] = state["acc"][2]
                data.ctrl[2] = state["acc"][3]
                data.ctrl[3] = state["acc"][4]
                data.ctrl[4] = state["acc"][5]
                data.ctrl[5] = state["acc"][6]
                data.ctrl[6] = state["acc"][7]
            # Pick up changes to the physics state, apply perturbations, update options from GUI.

            # Rudimentary time keeping, will drift relative to wall clock.
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
        print(viewer.__dict__)
        close_viewers(viewers)


def copy_model_and_data(
    model: mujoco.MjModel, data: mujoco.MjData
) -> "tuple[mujoco.MjModel, mujoco.MjData]":
    m = model.__copy__()
    d = data.__copy__()
    return m, d


def get_flags():
    flags = {}
    a = [x for x in dir(mujoco.mjtVisFlag) if "mjVIS_" in x]
    for x in a:
        for i in range(len(a)):
            if eval(f"{i} == mujoco.mjtVisFlag." + x):
                flags[i] = x
    return [value for _key, value in sorted(flags.items())]
