#!/usr/bin/env python3
from argparse import ArgumentParser

from mujoco_twin_node import get_flags, sim, sim2, sim3, test_sim

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


global_parser = ArgumentParser(
    prog="MuJoCo_Sim",
    description="Command Line Interface for thesis project",
    epilog="Have fun using %(prog)s!",
)
global_parser.set_defaults(func=global_parser.print_help)


subparsers = global_parser.add_subparsers(title="subcommands", help="Sub operations")
mjc_parser = subparsers.add_parser("mjc", help="MuJoCo Commands")
mjc_parser.set_defaults(func=mjc_parser.print_help)
mjc_subparsers = mjc_parser.add_subparsers(
    title="mjc_subcommands", help="MuJoCo operations"
)
mjc_run_parser = mjc_subparsers.add_parser("run", help="Run MuJoCo")
mjc_run_parser.set_defaults(func=sim)
mjc_run_parser = mjc_subparsers.add_parser("test", help="Test MuJoCo")
mjc_run_parser.set_defaults(func=test_sim)
mjc_run_parser = mjc_subparsers.add_parser("test2", help="Run MuJoCo Test 2")
mjc_run_parser.set_defaults(func=sim2)
mjc_run_parser = mjc_subparsers.add_parser("test3", help="Run MuJoCo Test 2")
mjc_run_parser.set_defaults(func=sim3)

mjc_flag_parser = mjc_subparsers.add_parser("flag", help="configure MuJoCo flags")
mjc_flag_parser.set_defaults(func=mjc_flag_parser.print_help)

mjc_flag_subparser = mjc_flag_parser.add_subparsers(
    title="mjc_flag_subcommands", help="MuJoCo flag operations"
)


mjc_flag_list_parser = mjc_flag_subparser.add_parser(
    "list", help="list all possible flags"
)
mjc_flag_list_parser.set_defaults(func=lambda: print(get_flags()))


mjc_flag_explain_parser = mjc_flag_subparser.add_parser(
    "explain", help="explain a flag"
)
mjc_flag_explain_parser.add_argument(dest="operands", nargs=1, type=str)
mjc_flag_explain_parser.set_defaults(
    func=lambda x: print(EXPLAINATION.get(x, f"<{x}> Does not exist!"))
)

ros_parser = subparsers.add_parser("ros", help="ROS Commands")
ros_parser.set_defaults(func=ros_parser.print_help)

args = global_parser.parse_args()
if "operands" in dir(args):
    args.func(*args.operands)
else:
    args.func()
