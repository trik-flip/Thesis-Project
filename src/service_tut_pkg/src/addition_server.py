#!/usr/bin/env python3
from __future__ import print_function

import rospy
from service_tut_pkg.srv import addition, additionResponse


def handle_add_two_ints(req: addition) -> additionResponse:
    print(f"getting: {req.x}, {req.y}")
    return additionResponse(req.x + req.y)


def add_two_ints_server() -> None:
    rospy.init_node("add_two_ints_server")
    rospy.Service("add_two_ints", addition, handle_add_two_ints)
    rospy.spin()


if __name__ == "__main__":
    add_two_ints_server()
