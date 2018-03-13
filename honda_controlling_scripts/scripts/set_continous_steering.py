#!/usr/bin/env python

from panda import Panda
from honda_create_msgs import create_steering_control
import time
import rospy
from std_msgs.msg import Int16

"""
Send braking commands on ros topic /steering_amount.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class SteerPublisher(object):
    def __init__(self):
        self.p = Panda()
        self.p.set_safety_mode(self.p.SAFETY_ALLOUTPUT)
        self.steer_val = 3840
        self.sub = rospy.Subscriber(
            '/steering_amount', Int16, self.steer_cb, queue_size=1)

    def steer_cb(self, data):
        print("steer_cb: " + str(data))
        if data.data > 3840:
            self.steer_val = 3840
        elif data.data < -3840:
            self.steer_val = -3840
        self.steer_val = data.data

    def run(self):
        print("Publishing...")
        idx_counter = 0
        total_cmds_sent = 0
        while not rospy.is_shutdown():
            cmd = create_steering_control(self.steer_val, idx_counter)[0]
            print("Sending: " + str(cmd) +
                  " (#" + str(total_cmds_sent) +
                  ") steer val: " + str(self.steer_val))
            self.p.can_send(cmd[0], cmd[2], 0)
            idx_counter += 1
            idx_counter %= 4
            total_cmds_sent += 1
            time.sleep(0.01)


if __name__ == '__main__':
    rospy.init_node('steering_sender')
    sp = SteerPublisher()
    sp.run()
