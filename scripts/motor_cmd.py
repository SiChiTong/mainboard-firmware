#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
import sys
from arming_control import VehicleArming

def talker():
    pub = rospy.Publisher('/turquoise/thrusters/input', Int16MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    arming_control = VehicleArming()

    rate = rospy.Rate(10)  # 10hz
    cmd_ = int(sys.argv[1])
    arming_control.set_arming(True)
    while not rospy.is_shutdown():
        msg = Int16MultiArray()
        msg.data = [cmd_ for x in range(8)]
        pub.publish(msg)
        rate.sleep()
    arming_control.set_arming(False)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
