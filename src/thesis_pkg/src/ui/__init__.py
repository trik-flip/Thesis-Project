from typing import Callable, Dict, Union

EXPLAINATION = {
    "mjVIS_TEXTURE": "TEXTURE",
    "mjVIS_CONVEXHULL": "CONVEXHULL1",
    "mjVIS_JOINT": "JOINT",
    "mjVIS_CAMERA": "CAMERA",
    "mjVIS_ACTUATOR": "ACTUATOR",
    "mjVIS_ACTIVATION": "ACTIVATION",
    "mjVIS_LIGHT": "LIGHT",
    "mjVIS_TENDON": "TENDON",
    "mjVIS_RANGEFINDER": "RANGEFINDER",
    "mjVIS_CONSTRAINT": "CONSTRAINT",
    "mjVIS_INERTIA": "INERTIAThe ",
    "mjVIS_SCLINERTIA": "SCLINERTIA",
    "mjVIS_PERTFORCE": "PERTFORCE",
    "mjVIS_PERTOBJ": "PERTOBJ",
    "mjVIS_CONTACTPOINT": "CONTACTPOINT",
    "mjVIS_ISLAND": "ISLAND",
    "mjVIS_CONTACTFORCE": "CONTACTFORCE",
    "mjVIS_CONTACTSPLIT": "CONTACTSPLIT",
    "mjVIS_TRANSPARENT": "TRANSPARENT",
    "mjVIS_AUTOCONNECT": "AUTOCONNECT",
    "mjVIS_COM": "COM",
    "mjVIS_SELECT": "SELECT",
    "mjVIS_STATIC": "STATIC",
    "mjVIS_SKIN": "SKIN",
    "mjVIS_FLEXVERT": "FLEXVERT",
    "mjVIS_FLEXEDGE": "FLEXEDGE",
    "mjVIS_FLEXFACE": "FLEXFACE",
    "mjVIS_FLEXSKIN": "FLEXSKIN",
    "mjVIS_BODYBVH": "BODYBVH",
    "mjVIS_FLEXBVH": "FLEXBVH",
    "mjVIS_MESHBVH": "MESHBVH",
    "mjVIS_SDFITER": "SDFITER",
    "mjNVISFLAG": "FLAG",
}
CLI_LEAF = Callable[[], None]
CLI_TREE = Dict[str, Union[CLI_LEAF, "CLI_TREE"]]
