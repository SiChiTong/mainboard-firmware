#!/usr/bin/python

import rospy
from xavier_stats.msg import StringStamped
import serial


class NucleoStatusNode:
    def __init__(self):
        self.port = rospy.get_param("~port", default="/dev/ttyACM0")
        self.baud = rospy.get_param("~baud", default=57600)
        self.serial_port = serial.Serial(port=self.port, baudrate=self.baud)

        if self.serial_port.is_open:
            self.serial_port.close()
        self.serial_port.open()

        self.pub = rospy.Publisher("/turquoise/nucleo/debug", StringStamped, queue_size=10)
        self.msg = StringStamped()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = "/base_link"
        self.msg.data = """
        ************************************************
        ************* Starting New Session *************
        ************************************************"""
        self.pub.publish(self.msg)
        rospy.loginfo(self.msg.data)

    def spin(self):
        while not rospy.is_shutdown():
            if not self.serial_port.is_open:
                break
            serial_string = None
            try:
                serial_string = self.serial_port.read_until("\n").replace("\n", "")
            except serial.serialutil.SerialException:
                break
            self.msg.header.stamp = rospy.Time.now()
            self.msg.header.frame_id = "/base_link"
            self.msg.data = serial_string
            self.pub.publish(self.msg)
            rospy.loginfo(self.msg.data)

if __name__ == "__main__":
    rospy.init_node("nucleo_status_node")
    node = NucleoStatusNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        node.serial_port.close()
        exit(1)