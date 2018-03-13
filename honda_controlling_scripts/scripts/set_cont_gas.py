#!/usr/bin/env python

from panda import Panda
from honda_create_msgs import create_gas_command
import time
import rospy
from std_msgs.msg import Int16

"""
Send braking commands on ros topic /Gasing_amount.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class GasPublisher(object):
    def __init__(self):
        self.p = Panda()
        self.p.set_safety_mode(self.p.SAFETY_ALLOUTPUT)
        self.gas_val = 0
        self.sub = rospy.Subscriber(
            '/gasing_amount', Int16, self.Gas_cb, queue_size=1)

    def Gas_cb(self, data):
        print("Gas_cb: " + str(data))
        if data.data > 1023:
            self.gas_val = 1023
        elif data.data < 0:
            self.gas_val = 0
        self.gas_val = data.data

    def run(self):
        print("Publishing...")
        idx_counter = 0
        total_cmds_sent = 0
        while not rospy.is_shutdown():

            cmd = create_gas_command(self.gas_val, idx_counter)
            print("command is: " + str(cmd))
            print("Sending: " + str(cmd) +
                  " (#" + str(total_cmds_sent) +
                  ") Gas val: " + str(self.gas_val))
            self.p.can_send(cmd[0], cmd[2], 0)
            idx_counter += 1
            idx_counter %= 4
            total_cmds_sent += 1
            time.sleep(0.01)


if __name__ == '__main__':
    rospy.init_node('Gasing_sender')
    sp = GasPublisher()
    sp.run()
