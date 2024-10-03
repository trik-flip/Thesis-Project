#!/usr/bin/env python3
import sys
import rospy

from service_tut_pkg.srv import addition

def add_two_ints_client(x: int, y: int) -> int:
    rospy.wait_for_service("add_two_ints")
    add_two_ints = rospy.ServiceProxy("add_two_ints", addition)
    resp = add_two_ints(x, y)
    print(resp.result)
    return resp.result


if __name__ == "__main__":
    x = int(sys.argv[1])
    y = int(sys.argv[2])
    add_two_ints_client(x, y)
