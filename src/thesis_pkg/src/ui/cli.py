#!/usr/bin/env python3
import sys
from argparse import ArgumentParser
from inspect import _empty, signature
from typing import Optional, cast

from insertion.contact_formations import insert
from sim import get_flags, simulators
from sim.inverse_kinematics import get_IK
from ui import CLI_LEAF, CLI_TREE, EXPLAINATION

sys.path.append("/home/philip/catkin_ws/src/thesis_pkg/src")


structure = {
    "run": lambda: None,
    "ros": {},
    # General Robot Assembly
    "GRA": insert,
    "debug": {
        "ik": {
            "gd": lambda: get_IK()
            .solve(
                [0, 0.5, 0.5],
            )
            .show_video(),
        }
    },
    "sim": {
        "mjc": {
            **simulators,
            "flag": {
                "list": lambda: print(get_flags()),
                "explain": lambda x: print(
                    EXPLAINATION.get(x, f"<{x}> Does not exist!")
                ),
            },
        },
    },
}


def create_parser(
    tree: CLI_TREE, parser: Optional[ArgumentParser] = None
) -> ArgumentParser:
    if parser is None:
        parser = ArgumentParser(
            prog="MuJoCo_Sim",
            description="Command Line Interface for thesis project",
            epilog="Have fun using %(prog)s!",
        )
        parser.set_defaults(func=parser.print_help)
    subparsers = parser.add_subparsers(title="subcommands", help="Sub operations")
    for sub in tree:
        _parser = subparsers.add_parser(sub)
        if isinstance(tree[sub], dict):
            _parser.set_defaults(func=_parser.print_help)
            sub_tree = cast(CLI_TREE, tree[sub])
            create_parser(sub_tree, _parser)
        else:
            cmd = cast(CLI_LEAF, tree[sub])
            for name, val in signature(cmd).parameters.items():
                if val.annotation != _empty:
                    _parser.add_argument(
                        name,
                        nargs="?",
                        type=val.annotation,
                        default=val.default,
                        help=f": {str(val.annotation)} -> {val.default}",
                    )
                else:
                    _parser.add_argument(name)

            _parser.set_defaults(func=cmd)
    return parser


def main() -> None:
    parser = create_parser(structure)
    args = parser.parse_args()
    if len([x for x in dir(args) if x[0] != "_"]) > 1:
        params = [x for x in dir(args) if x[0] != "_"]
        params = [x for x in params if x != "func"]
        params = {x: getattr(args, x) for x in params}
        args.func(**params)
    else:
        args.func()
