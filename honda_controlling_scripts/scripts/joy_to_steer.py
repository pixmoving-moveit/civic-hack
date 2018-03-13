#!/usr/bin/env python

from sensor_msgs.msg import Joy
import rospy
from std_msgs.msg import Int16


class JoyToSteer(object):
    def __init__(self):
        self.pub = rospy.Publisher('/steering_amount', Int16, queue_size=1)
        self.sub = rospy.Subscriber('/joy', Joy, self.joy_cb, queue_size=1)

    def joy_cb(self, data):
        left_right = data.axes[0] * -1
        amount_steer = int(left_right * 3840)
        self.pub.publish(amount_steer)
        print("amount_steer: " + str(amount_steer))


if __name__ == '__main__':
    rospy.init_node('from_joy_to_steer')
    jts = JoyToSteer()
    rospy.spin()
