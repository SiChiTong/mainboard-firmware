#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist
import sys

pwm_max = 1900
pwm_min = 1100

def constrain(val_, min_, max_):
    if val_ > max_:
        return max_
    if val_ < min_:
        return min_
    return val_

class AuvTeleop(object):

    def __init__(self):
        self.cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self.callback)
        self.pub = rospy.Publisher('/turquoise/thrusters/input', Int16MultiArray, queue_size=10)
        self.rate = rospy.Rate(20)

    def callback(self, data):
        yaw = data.angular.z
        surge = data.linear.x
        heave = data.linear.z

        yaw = constrain(yaw, -1.0, 1.0) * 400.0
        surge = constrain(surge, -1.0, 1.0) * 400.0
        heave = constrain(heave, -1.0, 1.0) * 400.0

        msg = Int16MultiArray()
        msg.data = [1500 + heave, 1500 + heave, 1500 + heave, 1500 + heave, 1500 + surge - yaw, 1500 + surge + yaw, 1500, 1500]
        print msg.data
        msg.data = [constrain(i, 1100, 1900) for i in msg.data]
        self.pub.publish(msg)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('cmd_teleop', anonymous=True)
        node = AuvTeleop()
        node.spin()
    except KeyboardInterrupt:
        exit()