#!/usr/bin/env python3
import rospy
from message_test_pkg.msg import Person, People


def callback(p: People) -> None:
    print(f"Group is called: {p.name}, has {len(p.people)} members")
    for _p in p.people:
        show(_p)


def show(p: Person) -> None:
    print(f"- name: {p.name}, age: {p.age}")


def listener() -> None:
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("chatter", People, callback)
    rospy.spin()


if __name__ == "__main__":
    listener()
