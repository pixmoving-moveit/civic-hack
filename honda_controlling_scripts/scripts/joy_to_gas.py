#!/usr/bin/env python

from sensor_msgs.msg import Joy
import rospy
from std_msgs.msg import Int16


class JoyToGas(object):
    def __init__(self):
        self.pub = rospy.Publisher('/gasing_amount', Int16, queue_size=1)
        self.sub = rospy.Subscriber('/joy', Joy, self.joy_cb, queue_size=1)

    def joy_cb(self, data):
        up = data.axes[1]
        print data.axes
        amount_gas = int(up * 1023)
        if amount_gas < 0:
            amount_gas = 0
        if amount_gas > 1023:
            amount_gas = 1023
        self.pub.publish(amount_gas)
        print("amount_gas: " + str(amount_gas))


if __name__ == '__main__':
    rospy.init_node('from_joy_to_gas')
    jts = JoyToGas()
    rospy.spin()
