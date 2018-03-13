#!/usr/bin/env python

from panda import Panda
from honda_create_msgs import create_brake_command
import time
import rospy
from std_msgs.msg import Int16

"""
Send braking commands on ros topic /braking_amount.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class BrakePublisher(object):
    def __init__(self):
        self.p = Panda()
        self.p.set_safety_mode(self.p.SAFETY_ALLOUTPUT)
        self.brake_val = 0
        self.sub = rospy.Subscriber(
            '/braking_amount', Int16, self.brake_cb, queue_size=1)

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
# def create_brake_command(apply_brake, pcm_override, pcm_cancel_cmd, chime, idx):
#     """Creates a CAN message for the Honda DBC BRAKE_COMMAND."""
#     pump_on = apply_brake > 0
#     brakelights = apply_brake > 0
#     brake_rq = apply_brake > 0

#     pcm_fault_cmd = False
#     amount = struct.pack("!H", (apply_brake << 6) + pump_on)
#     msg = amount + struct.pack("BBB", (pcm_override << 4) |
#                                (pcm_fault_cmd << 2) |
#                                (pcm_cancel_cmd << 1) | brake_rq, 0x80,
#                                brakelights << 7) + chr(chime) + "\x00"
#     return make_can_msg(0x1fa, msg, idx, 0)

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
    rospy.init_node('braking_sender')
    sp = BrakePublisher()
    sp.run()
