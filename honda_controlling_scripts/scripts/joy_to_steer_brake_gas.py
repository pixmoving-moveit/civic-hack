#!/usr/bin/env python

from sensor_msgs.msg import Joy
import rospy
from std_msgs.msg import Int16


class JoyToBrake(object):
    def __init__(self):
        self.pub_braking = rospy.Publisher('/braking_amount', Int16,
                                           queue_size=1)
        self.pub_steering = rospy.Publisher('/steering_amount', Int16,
                                            queue_size=1)
        self.pub_gas = rospy.Publisher('/gasing_amount', Int16, queue_size=1)

        self.sub = rospy.Subscriber('/joy', Joy, self.joy_cb, queue_size=1)

    def joy_cb(self, data):
        up = data.axes[1]
        left_right = data.axes[0] * -1.0
        down = data.axes[4]
        print data.axes
        amount_brake = int(down * 1023 / 5)
        amount_brake *= -1
        if amount_brake < 0:
            amount_brake = 0
        self.pub_braking.publish(amount_brake)
        amount_gas = int(up * 1023)
        if amount_gas < 0:
            amount_gas = 0
        if amount_gas > 1023:
            amount_gas = 1023
        self.pub_gas.publish(amount_gas)
        amount_steer = int(left_right * 3840)
        self.pub_steering.publish(amount_steer)
        print("amount_steer: " + str(amount_steer))
        print("amount_gas: " + str(amount_gas))
        print("amount_brake: " + str(amount_brake))


if __name__ == '__main__':
    rospy.init_node('from_joy_to_brake')
    jts = JoyToBrake()
    rospy.spin()
