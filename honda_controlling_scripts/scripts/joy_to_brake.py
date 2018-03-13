#!/usr/bin/env python

from sensor_msgs.msg import Joy
import rospy
from std_msgs.msg import Int16


class JoyToBrake(object):
    def __init__(self):
        self.pub = rospy.Publisher('/braking_amount', Int16, queue_size=1)
        self.sub = rospy.Subscriber('/joy', Joy, self.joy_cb, queue_size=1)

    def joy_cb(self, data):
        down = data.axes[4]
        print data.axes
        amount_brake = int(down * 1023 / 10)
        amount_brake *= -1
        if amount_brake < 0:
            amount_brake = 0
        self.pub.publish(amount_brake)
        print("amount_brake: " + str(amount_brake))


if __name__ == '__main__':
    rospy.init_node('from_joy_to_brake')
    jts = JoyToBrake()
    rospy.spin()
