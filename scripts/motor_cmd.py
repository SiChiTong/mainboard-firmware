#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
import sys


def talker():
    pub = rospy.Publisher('/turquoise/thrusters/input', Int16MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    rate = rospy.Rate(10)  # 10hz
    cmd_ = int(sys.argv[1])
    while not rospy.is_shutdown():
        msg = Int16MultiArray()
        msg.data = [cmd_ for x in range(8)]
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
