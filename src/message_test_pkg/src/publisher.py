#!/usr/bin/env python3
import rospy
from message_test_pkg.msg import Person, People
import random


def talker() -> None:
    pub = rospy.Publisher("chatter", People, queue_size=10)
    rospy.init_node("talker", anonymous=True)
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        people = People()
        people.name = "Wonky name!"
        for i in range(random.randint(0, 10)):
            person = Person()
            person.name = f"person[{i}]" + str()
            person.age = random.randint(18, 45)
            people.people.append(person)
        # rospy.loginfo(f"pub: {people} {rospy.get_time()}")
        pub.publish(people)
        rate.sleep()


if __name__ == "__main__":
    try:
        print("/chatter is active")
        talker()
    except rospy.ROSInterruptException:
        pass
