#!/usr/bin/env python

from panda import Panda
from honda_create_msgs import create_gas_command, create_brake_command, create_steering_control
import time
import rospy
from std_msgs.msg import Int16

"""

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class AllPublisher(object):
    def __init__(self):
        self.p = Panda()
        self.p.set_safety_mode(self.p.SAFETY_ALLOUTPUT)
        self.gas_val = 0
        self.brake_val = 0
        self.steer_val = 0
        self.sub_gas = rospy.Subscriber(
            '/gasing_amount', Int16, self.Gas_cb, queue_size=1)
        self.sub_steer = rospy.Subscriber(
            '/steering_amount', Int16, self.steer_cb, queue_size=1)
        self.sub_brake = rospy.Subscriber(
            '/braking_amount', Int16, self.brake_cb, queue_size=1)

    def Gas_cb(self, data):
        print("Gas_cb: " + str(data))
        if data.data > 1023:
            self.gas_val = 1023
        elif data.data < 0:
            self.gas_val = 0
        self.gas_val = data.data

    def steer_cb(self, data):
        print("steer_cb: " + str(data))
        if data.data > 3840:
            self.steer_val = 3840
        elif data.data < -3840:
            self.steer_val = -3840
        self.steer_val = data.data

    def brake_cb(self, data):
        print("brake_cb: " + str(data))
        if data.data > 1023:
            self.brake_val = 1023
        elif data.data < 0:
            self.brake_val = 0
        self.brake_val = data.data

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

            cmd = create_steering_control(self.steer_val, idx_counter)[0]
            print("Sending: " + str(cmd) +
                  " (#" + str(total_cmds_sent) +
                  ") steer val: " + str(self.steer_val))
            self.p.can_send(cmd[0], cmd[2], 0)

            # (apply_brake, pcm_override, pcm_cancel_cmd, chime, idx):
            cmd = create_brake_command(self.brake_val, 1, 0, 0, idx_counter)
            print("Sending: " + str(cmd) +
                  " (#" + str(total_cmds_sent) +
                  ") brake val: " + str(self.brake_val))
            self.p.can_send(cmd[0], cmd[2], 0)

            idx_counter += 1
            idx_counter %= 4
            total_cmds_sent += 1
            time.sleep(0.01)


if __name__ == '__main__':
    rospy.init_node('all_sender')
    sp = AllPublisher()
    sp.run()
