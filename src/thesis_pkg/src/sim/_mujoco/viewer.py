#!/usr/bin/env python3
import atexit
import queue
import threading
import time
from collections.abc import Callable
from functools import partial
from typing import Optional, Union, cast

import glfw
import mujoco
import mujoco.viewer
from mujoco import _simulate  # type: ignore
from utils.decorators import create_collection

from .call_back import callback, key_callback
from .files import PATH_MODEL_CUBE
from .util import copy_model, create_model_iiwa

__all__ = [
    "simulators",
]

simulators = {}
simulator_register = create_collection(simulators)

"""
When settings mujoco settings
options = mujoco.MjvOption()
mujoco.mjv_defaultOption(options)

options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT.value] = True
options.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE.value] = True
options.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT.value] = True
"""


@simulator_register("test1")
def test_sim(ros_namespace: str = "digital_twin") -> None:
    MODEL_IIWA, MODEL_DATA_IIWA = create_model_iiwa()
    with mujoco.viewer.launch_passive(MODEL_IIWA, MODEL_DATA_IIWA) as viewer:
        while viewer.is_running():
            step_start = time.time()

            mujoco.mj_step(MODEL_IIWA, MODEL_DATA_IIWA)
            viewer.sync()

            time_until_next_step = MODEL_IIWA.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


@simulator_register("test3")
def sim3() -> None:
    model = mujoco.MjModel.from_xml_path(PATH_MODEL_CUBE)
    data = mujoco.MjData(model)
    handle = _launch_passive(model, data)
    print(data.qpos)
    while handle.is_running():
        mujoco.mj_step(model, data)
        handle.sync()


def _launch_passive(
    model: mujoco.MjModel,
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
        target=_launch_sim,
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


def _launch_sim(
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
    assert glfw._glfw is not None
    _simulate.set_glfw_dlhandle(glfw._glfw._handle)
    _Simulate = _simulate.Simulate

    _assert_input_correctness(model, data, run_physics_thread, loader, handle_return)

    if loader is None and model is not None:

        def _loader(
            m: mujoco.MjModel = model, d: Optional[mujoco.MjData] = data
        ) -> "tuple[mujoco.MjModel, mujoco.MjData]":
            if d is None:
                d = mujoco.MjData(m)
            return m, d

        loader = _loader

    assert model is not None
    cam, opt, pert, user_scn, simulate = _setup_viewer(
        model, run_physics_thread, key_callback, show_left_ui, show_right_ui, _Simulate
    )

    # Initialize GLFW if not using mjpython.
    if mujoco.viewer._MJPYTHON is None:
        if not glfw.init():
            raise mujoco.FatalError("could not initialize GLFW")  # type: ignore
        atexit.register(glfw.terminate)

    notify_loaded: Optional[Callable[[], None]] = None
    if handle_return:
        notify_loaded = lambda: handle_return.put_nowait(
            mujoco.viewer.Handle(simulate, cam, opt, pert, user_scn)
        )

    if run_physics_thread:
        viewer_thread = threading.Thread(
            target=mujoco.viewer._physics_loop, args=(simulate, loader)
        )
    else:
        viewer_thread = threading.Thread(
            target=mujoco.viewer._reload, args=(simulate, loader, notify_loaded)
        )

    atexit.register(simulate.exit)
    viewer_thread.start()
    simulate.render_loop()
    atexit.unregister(simulate.exit)
    viewer_thread.join()
    simulate.destroy()


def _setup_viewer(
    model: mujoco.MjModel,
    run_physics_thread: bool,
    key_callback: Optional[mujoco.viewer.KeyCallbackType],
    show_left_ui: bool,
    show_right_ui: bool,
    _Simulate: _simulate.Simulate,
) -> "tuple[mujoco.MjvCamera, mujoco.MjvOption,mujoco.MjvPerturb,Optional[mujoco.MjvScene],_simulate.Simulate]":
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
    return cam, opt, pert, user_scn, simulate


def _assert_input_correctness(
    model: Optional[mujoco.MjModel],
    data: Optional[mujoco.MjData],
    run_physics_thread: bool,
    loader: Optional[mujoco.viewer._InternalLoaderType],
    handle_return: Optional["queue.Queue[mujoco.viewer.Handle]"],
) -> None:
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


@simulator_register("test2")
def sim2() -> None:
    model, data = create_model_iiwa()
    cam: Optional[mujoco.MjvCamera] = mujoco.MjvCamera()
    opt: Optional[mujoco.MjvOption] = mujoco.MjvOption()
    con: Optional[mujoco.MjrContext] = mujoco.MjrContext()
    pert: Optional[mujoco.MjvPerturb] = mujoco.MjvPerturb()
    scn: mujoco.MjvScene

    glfw.init()
    window = glfw.create_window(1200, 900, "Demo", None, None)
    glfw.make_context_current(window)
    glfw.swap_interval(1)

    mujoco.mjv_defaultCamera(cam)
    mujoco.mjv_defaultPerturb(pert)
    mujoco.mjv_defaultOption(opt)
    # mujoco.mjr_defaultContext(con)
    scn = mujoco.MjvScene(model, 1000)

    # mujoco.mjr_makeContext(m, con, mujoco.mjtFontScale.mjFONTSCALE_100.value)

    assert data is not None
    while not glfw.window_should_close(window):
        simstart = data.time
        while data.time - simstart < 1.0 / 60.0:
            mujoco.mj_step(model, data)

        fb = glfw.get_framebuffer_size(window)

        mujoco.mjv_updateScene(
            model, data, opt, pert, cam, mujoco.mjtCatBit.mjCAT_ALL.value, scn
        )
        viewport = mujoco.MjrRect(0, 0, *fb)
        mujoco.mjr_render(viewport, scn, con)

        glfw.swap_buffers(window)

        glfw.poll_events()

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
    mujoco.mjv_updateScene(
        model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL.value, scn
    )
    mujoco.mjr_render(viewport, scn, con)
    # mujoco.viewer.launch_passive(m, d)
    mujoco.mjv_freeScene(scn)
    mujoco.mjr_freeContext(con)


@simulator_register("run")
def sim(
    ros_namespace: str = "digital_twin",
    model: Optional[Union[mujoco.MjModel, str]] = None,
) -> None:
    if model is None:
        model, data = create_model_iiwa()
    else:
        if isinstance(model, str):
            model = mujoco.MjModel.from_xml_path(model)
        model = cast(mujoco.MjModel, model)
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
        # rospy.init_node("mujoco_listener", anonymous=True)
        print("init sub")
        _, data = create_model_iiwa()
        call_back = partial(callback, data)
        # rospy.Subscriber(f"{ros_namespace}/joints", JointPosition, call_back)
        print("init pub")
        # pub = rospy.Publisher(f"{ros_namespace}/forces", JointPosition, queue_size=10)
        print(f"{dir(model)}")
        while viewer.is_running():
            step_start = data.time
            # mj_step can be replaced with code that also evaluates a policy and applies a control signal before stepping the physics.
            mujoco.mj_step(model, data)
            # * Step 3.a: Retrieve data from simulation, this is implied since we're able to get data from the simulator every timestep
            # * Step 4.a: Pass on reconstructed high-level info
            # pub.publish(translate_forces(data.actuator_force, Modes.R2S))
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
                new_m, new_d = copy_model(model, data)
                print("copied")
                # TODO: new_viewer = open_async_viewer(new_m, new_d)
                print("opened new viewer")
                # !viewers.append((new_m, new_d, new_viewer))
                print("listed new viewer")
            if state["close"]:
                viewer.close()
            if state["acc"][0]:

                follow_state_update(data, state)
            # Pick up changes to the physics state, apply perturbations, update options from GUI.

            # Rudimentary time keeping, will drift relative to wall clock.
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
        print(viewer.__dict__)


def follow_state_update(data, state):
    state["acc"][0] = False
    data.ctrl[0] = state["acc"][1]
    data.ctrl[1] = state["acc"][2]
    data.ctrl[2] = state["acc"][3]
    data.ctrl[3] = state["acc"][4]
    data.ctrl[4] = state["acc"][5]
    data.ctrl[5] = state["acc"][6]
    data.ctrl[6] = state["acc"][7]
