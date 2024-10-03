import glob
import os
from typing import Literal

from dm_control import mjcf
from mujoco import MjModel
from pipe import Pipe

path = "/home/philip/Code/OtherProjects/mujoco_menagerie/kuka_iiwa_14/scene.xml"


def gen_target(pos=[0.5, 0, 0.6], rot=[0, 1, 0, 0], mocap=True):

    target = mjcf.RootElement()
    b = target.worldbody.add("body", name="target", pos=pos, quat=rot, mocap=mocap)
    return target


def get_mujoco_model(xml_model):
    export_model(xml_model)
    model = MjModel.from_xml_path("tmp/tmp.xml")
    clean_temp_files()
    return model


def clean_temp_files():
    glob.glob("tmp/*") | remove_files
    if os.path.exists("tmp/"):
        os.removedirs("tmp/")


def export_model(model):
    mjcf.export_with_assets(model, "tmp", "tmp.xml")


@Pipe
def remove_files(files) -> None:
    for file in files:
        os.remove(file)


def gen_end_effector(type: Literal["box", "sphere", "capsule"]) -> mjcf.RootElement:
    end_effector = mjcf.RootElement("end_effector")
    body = end_effector.worldbody.add("body", name="end_effector")
    if type not in [
        "plane",
        "hfield",
        "sphere",
        "capsule",
        "ellipsoid",
        "cylinder",
        "box",
        "mesh",
        "sdf",
    ]:
        raise Exception(f"Unsupported type {type}")
    if type == "sphere":
        body.add("geom", name="end_effector", type="sphere", size=[0.045, 0.15])
    elif type == "capsule":
        body.add("geom", name="end_effector", type="capsule", size=[0.045, 0.15])
    elif type == "box":
        body.add(
            "geom",
            name="end_effector",
            type="box",
            size=[0.025, 0.025, 0.07],
            pos=[0, 0, 0.04],
        )
    else:
        raise Exception(f"Not yet supported type {type}")
    return end_effector


def gen_goal_square_wall(
    z: float = 0.5,
    distance=0.5,
    depth=0.5,
    width_clearance: float = 0.025,
    heigth_clearance: float = 0.025,
    radius=None,
) -> mjcf.RootElement:
    if radius is not None:
        width_clearance = radius
        heigth_clearance = radius
    goal = mjcf.RootElement("goal")
    goal_body = goal.worldbody.add("body", name="goal")
    goal_body.add(
        "geom",
        type="box",
        name="top",
        size=[0.1, width_clearance, 0.025],
        pos=[distance, depth, z + heigth_clearance + 0.025],
        rgba=[1, 0, 0, 1],
    )
    goal_body.add(
        "geom",
        type="box",
        name="bottom",
        size=[0.1, width_clearance, 0.025],
        pos=[distance, depth, z - heigth_clearance - 0.025],
        rgba=[1, 1, 0, 1],
    )
    goal_body.add(
        "geom",
        type="box",
        name="left",
        size=[0.1, 0.025, heigth_clearance],
        pos=[distance, depth - width_clearance - 0.025, z],
        rgba=[0, 1, 0, 1],
    )
    goal_body.add(
        "geom",
        type="box",
        name="right",
        size=[0.1, 0.025, heigth_clearance],
        pos=[distance, depth + width_clearance + 0.025, z],
        rgba=[0, 1, 1, 1],
    )
    return goal


def gen_goal_square(
    center: "tuple[float,float]" = (0.5, 0),
    width_clearance: float = 0.025,
    heigth_clearance: float = 0.025,
    radius=None,
) -> mjcf.RootElement:
    if radius is not None:
        width_clearance = radius
        heigth_clearance = radius
    x, y = center
    goal = mjcf.RootElement("goal")
    goal_body = goal.worldbody.add("body", name="goal")
    goal_body.add(
        "geom",
        type="box",
        name="top_wall",
        size=[width_clearance, 0.025, 0.1],
        pos=[x, y + heigth_clearance + 0.025, 0.1],
        rgba=[1, 0, 0, 1],
    )
    goal_body.add(
        "geom",
        type="box",
        name="bottom_wall",
        size=[width_clearance, 0.025, 0.1],
        pos=[x, y - heigth_clearance - 0.025, 0.1],
        rgba=[0, 1, 0, 1],
    )
    goal_body.add(
        "geom",
        type="box",
        name="right_wall",
        size=[0.025, heigth_clearance, 0.1],
        pos=[x + width_clearance + 0.025, y, 0.1],
        rgba=[0, 1, 1, 1],
    )
    goal_body.add(
        "geom",
        type="box",
        name="left_wall",
        size=[0.025, heigth_clearance, 0.1],
        pos=[x - width_clearance - 0.025, y, 0.1],
        rgba=[0, 0, 1, 1],
    )
    return goal


def attach_to_model(model: mjcf.RootElement, attachment: mjcf.Element):
    attachment_site = model.find("site", "attachment_site")
    assert attachment_site is not None
    return attachment_site.attach(attachment)
