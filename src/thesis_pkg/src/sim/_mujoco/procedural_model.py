import enum
import logging
import os
import re
from glob import glob
from math import pi

from dm_control import mjcf

_logger = logging.getLogger("ProceduralModelLogger")
_logger.setLevel(logging.DEBUG)


def _possible_end_effectors_enum(path_name: str = "end_effectors") -> enum.Enum:
    return enum.Enum(
        f"possible_{path_name}",
        [
            (
                x.group(1)
                if (x := re.search(f"^{path_name}/(.+)\\.obj", s)) is not None
                else s[13:]
            )
            for s in glob(f"{path_name}/*")
        ],
    )


DYN_ENUM = _possible_end_effectors_enum()


def _add_composite_model(
    ee: str,
    xml_model: mjcf.RootElement,
    attachment_site: "mjcf.Element | None",
    path_name: str = "end_effectors",
    **kwargs,
) -> None:
    _logger.info("add composite model")
    assert os.path.exists(f"{path_name}{ee}"), f"{path_name}{ee} does not exist"

    files = glob(f"{path_name}{ee}/*")
    single_file_names = (
        (
            s.group(1)
            if (s := re.search(f"^{path_name}{ee}/(.+)\\.obj", file)) is not None
            else None
        )
        for file in files
    )
    if attachment_site:
        body = attachment_site.add("body", name="ee")
    else:
        body = xml_model.worldbody.add("body", name="hole")
    assert body is not None, "body is None here"

    for file in single_file_names:
        file_name = os.path.abspath(f"{path_name}{ee}/{file}.obj")
        assert os.path.exists(file_name), f"{file_name} does not exist"
        xml_model.asset.add(
            "mesh",
            file=file_name,
            **{k.replace("asset_", ""): v for k, v in kwargs.items() if "asset_" in k},
        )
        body.add(
            "geom",
            type="mesh",
            mesh=file,
            name=file,
            **{k.replace("geom_", ""): v for k, v in kwargs.items() if "geom_" in k},
        )


def _add_singlular(
    ee: str,
    xml_model: mjcf.RootElement,
    attachment_site: "mjcf.Element | None",
    path_name: str = "end_effectors",
    **kwargs,
) -> None:
    _logger.info("add singlular model")
    object_props = {"type": "mesh", "mesh": ee}
    file = os.path.abspath(f"{path_name}/{ee}.obj")
    assert os.path.exists(file)
    xml_model.asset.add(
        "mesh",
        file=file,
        **{k.replace("asset_", ""): v for k, v in kwargs.items() if "asset_" in k},
    )
    if attachment_site:
        body = attachment_site.add("body", name="ee")
    else:
        body = xml_model.worldbody.add("body", name="hole")
    assert body is not None

    regex_search = re.search("^(.+)_(\\d+)", ee)
    if regex_search is not None:
        _logger.info("add a repetative model")
        _add_repetative_model(
            body,
            regex_search,
            **object_props,
            **{k.replace("geom_", ""): v for k, v in kwargs.items() if "geom_" in k},
            **{k: v for k, v in kwargs.items() if "compiler_" in k},
        )
    else:
        body.add(
            "geom",
            name=ee,
            **object_props,
            **{k.replace("geom_", ""): v for k, v in kwargs.items() if "geom_" in k},
        )


def _add_repetative_model(body: mjcf.Element, regex_search: re.Match, **kwargs) -> None:

    ee = regex_search.group(1)
    n = int(regex_search.group(2))
    angle = 360 / n
    _logger.debug(f"kwargs: {kwargs}")
    angle_spec = kwargs.pop("compiler_angle", False)
    if angle_spec:
        _logger.info("using radians")
        angle = 2 * pi / n
    for i in range(n):
        body.add(
            "geom",
            name=f"{ee}_{i+1}",
            euler=[0, 0, angle * i],
            **kwargs,
        )


def add_model(
    ee: "str | int | DYN_ENUM",
    xml_model: mjcf.RootElement,
    attachment_site: "mjcf.Element | None" = None,
    **kwargs,
) -> None:
    update_dyn_emum(kwargs)

    name = get_name_from_ee(ee)

    assert name in [
        e.name for e in DYN_ENUM
    ], f"{name} is not in {[e.name for e in DYN_ENUM]}"

    _logger.debug(kwargs)
    _logger.debug(os.path.abspath(name))
    _logger.debug(os.path.expanduser("~"))

    _select_correct_model_adder(name, xml_model, attachment_site, kwargs)


def _select_correct_model_adder(
    name: str, xml_model: mjcf.RootElement, attachment_site, kwargs
) -> None:
    if name[0] == "/":
        _add_composite_model(name, xml_model, attachment_site, **kwargs)
    else:
        _add_singlular(name, xml_model, attachment_site, **kwargs)


def get_name_from_ee(ee: "str | int | DYN_ENUM"):
    if isinstance(ee, int):
        return ee_convert_int2str(ee)
    elif isinstance(ee, DYN_ENUM):
        return ee_convert_enum2str(ee)
    return ee


def ee_convert_enum2str(ee: DYN_ENUM):
    name = ee.name
    _logger.info(f"using {ee}")
    return name


def ee_convert_int2str(ee: int):
    assert ee > 0
    assert ee <= len(DYN_ENUM)
    l = [t for t in DYN_ENUM if t.value == ee]
    assert len(l) == 1
    name = l[0].name
    _logger.info(f"using {ee}")
    return name


def update_dyn_emum(kwargs) -> None:
    if "path_name" in kwargs:
        global DYN_ENUM
        DYN_ENUM = _possible_end_effectors_enum(kwargs["path_name"])


def _main(model_name: str = "cyl_8") -> None:
    import mujoco.viewer as viewer
    from mujoco import MjModel

    home = os.path.expanduser("~")
    path = f"{home}/Code/OtherProjects/mujoco_menagerie/kuka_iiwa_14/scene2.xml"
    assert os.path.exists(path), path + " does not exist"
    xml_model = mjcf.from_path(path)
    site = xml_model.find("site", "attachment_site").parent
    common_options = {
        "asset_scale": [0.1, 0.1, 0.1],
        "compiler_angle": "radian",
    }

    add_model(f"e_{model_name}", xml_model, site, **common_options)
    add_model(f"h_{model_name}", xml_model, geom_pos=[0.5, 0.5, 0], **common_options)
    mjcf.export_with_assets(xml_model, "tmp", "tmp.xml")
    model = MjModel.from_xml_path("tmp/tmp.xml")
    viewer.launch(model)


if __name__ == "__main__":
    _main()
