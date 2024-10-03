#!/usr/bin/env python3

from std_srvs.srv import Empty, EmptyResponse
import rospy


def empty_response(_req: Empty) -> EmptyResponse:
    print("Just a service")
    return EmptyResponse()


if __name__ == "__main__":
    rospy.init_node("empty_server_service")
    s = rospy.Service("print_service", Empty, empty_response)
    rospy.spin()
