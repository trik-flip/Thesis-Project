#!/usr/bin/env python3
from argparse import ArgumentParser
from model import *

global_parser = ArgumentParser(
    prog="clap",
    description="simple command line calc",
    epilog="Have fun using %(prog)s!",
)

subparsers = global_parser.add_subparsers(
    title="subcommands", help="arithmetic operations"
)
arg_template = {
    "dest": "operands",
    "type": float,
    "metavar": "OPERAND",
    "help": "a numeric value",
}
add_parser = subparsers.add_parser("add", help="add two numbers a and b")
add_parser.add_argument(**arg_template, nargs=2)
add_parser.set_defaults(func=add, parser=add_parser)

mul_parser = subparsers.add_parser("mul", help="mul two numbers a and b")
mul_parser.add_argument(**arg_template, nargs=2)
mul_parser.set_defaults(func=mul, parser=mul_parser)

div_parser = subparsers.add_parser("div", help="div two numbers a and b")
div_parser.add_argument(**arg_template, nargs=2)
div_parser.set_defaults(func=div, parser=div_parser)

sub_parser = subparsers.add_parser("sub", help="subtract two numbers a and b")
sub_parser.add_argument(**arg_template, nargs=2)
sub_parser.set_defaults(func=subtract, parser=sub_parser)

power_parser = subparsers.add_parser("power", help="power two numbers")
power_subparsers = power_parser.add_subparsers(
    title="power_subcommands", help="power operations"
)


square_parser = power_subparsers.add_parser("square", help="square a number")
square_parser.add_argument(**arg_template, nargs=1)
square_parser.set_defaults(func=square, parser=square_parser)

root_parser = power_subparsers.add_parser("root", help="root a number")
root_parser.add_argument(**arg_template, nargs=1)
root_parser.set_defaults(func=root, parser=root_parser)


import sys

if len(sys.argv) == 1:
    global_parser.print_help(sys.stderr)
    sys.exit(1)
args = global_parser.parse_args()
print(args.func(*args.operands))
