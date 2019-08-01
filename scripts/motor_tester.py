#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
import sys

pwm_max = 1900
pwm_min = 1100


def talker():
    pub = rospy.Publisher('/turquoise/thrusters/input', Int16MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    rate = rospy.Rate(20)  # 10hz
    motor_index = int(sys.argv[1])

    for i in range((pwm_max - 1500) / 10):
        msg = Int16MultiArray()
        msg.data = [1500 for x in range(8)]
        msg.data[motor_index] = i * 10 + 1500
        pub.publish(msg)
        rate.sleep()

    for i in range((pwm_max - 1500) / 10):
        msg = Int16MultiArray()
        msg.data = [1500 for x in range(8)]
        msg.data[motor_index] = pwm_max - i * 10
        pub.publish(msg)
        rate.sleep()

    for i in range((1500 - pwm_min) / 10):
        msg = Int16MultiArray()
        msg.data = [1500 for x in range(8)]
        msg.data[motor_index] = 1500 - i * 10
        pub.publish(msg)
        rate.sleep()

    for i in range((1500 - pwm_min) / 10):
        msg = Int16MultiArray()
        msg.data = [1500 for x in range(8)]
        msg.data[motor_index] = pwm_min + i * 10
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    talker()
