#!/usr/bin/env python
import rospy 
from std_msgs.msg import Int16, Int16MultiArray

def constrain(value, min, max):
    if value < min:
        return min
    elif value > max:
        return max
    return value

class AUXRemapNode:
    def __init__(self):
        rospy.init_node("aux_node")
        rospy.loginfo("Starting AUXRemapNode.")
        self.light_sub = rospy.Subscriber("/turquoise/lights/0", Int16, self.light_callback)
        self.aux_pub = rospy.Publisher("/turquoise/aux", Int16MultiArray, queue_size=1)
        self.aux = Int16MultiArray()

    def light_callback(self, data):
        self.aux.data[0] = constrain(data.data, 1100, 1900)
        self.aux_pub.publish(self.aux)


if __name__ == "__main__":
    ros_node = AUXRemapNode()
    rospy.spin()