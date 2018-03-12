#!/usr/bin/env python

import rospy
from panda import Panda
from can_msgs.msg import Frame

"""
Node to connect to a panda board connected to a CAN bus
and show the CAN messages thru a can_msgs/Frame topic, it
also subscribes to /send_can_msg to send the can messages
received there.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
Author: Pier-Marc Comtois-Rivet <pm.rivet at gmail.com>
"""


class PandaBridgePub(object):
    def __init__(self, rate=1000):
        self.can_pub = rospy.Publisher('can_frame_msgs', Frame, queue_size=10)
        rospy.loginfo("Setting up publisher to: " +
                      str(self.can_pub.resolved_name))
        self.rate = rate
        rospy.loginfo("Reading from panda board at " + str(self.rate) + " Hz.")
        rospy.loginfo("Connecting to Panda board...")
        self.panda = Panda()
        rospy.loginfo("Setting safety mode to SAFETY_ALLOUTPUT")
        self.panda.set_safety_mode(self.panda.SAFETY_ALLOUTPUT)
        self.to_send_msgs = []
        self.can_sub = rospy.Subscriber('send_can_msg', Frame, self.can_cb,
                                        queue_size=10)
        rospy.loginfo("Connected.")

    def can_cb(self, data):
        self.to_send_msgs.append(data)

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():

            # Reading gives us up to 256 messages
            can_msg_block = self.panda.can_recv()
            # print can_msg_block
            # A message looks like:
            # [(420, 55639, bytearray(b'\x00f\x00\x00\x00\x00\x00:'), 0),
            # (428, 55761, bytearray(b'\x7f\xff\x00\x00\x00\x08\x002'), 0),
            # ... ]

            if can_msg_block:
                self.process_msg_block(can_msg_block)

            if self.to_send_msgs:
                # copy current msgs to send and reset buffer
                msgs = self.to_send_msgs[:]
                self.to_send_msgs = []
                # send all messages
                for msg in msgs:
                    self.send_can_msg(msg)

            rate.sleep()

    def process_msg_block(self, can_msg_block):
        for msg in can_msg_block:
            frame = self.convert_panda_to_msg(msg)
            self.can_pub.publish(frame)

    def send_can_msg(self, frame):
        self.panda.can_send(frame.id, str(frame.data), 0)

    def convert_panda_to_msg(self, can_msg):

        frame = Frame()
        frame.id = can_msg[0]
        frame.dlc = 8
        frame.is_error = 0
        frame.is_rtr = 0
        frame.is_extended = 0

        # hacky conversion to uint8
        frame.data = str(can_msg[2])
        frame.header.frame_id = ""
        frame.header.stamp = rospy.get_rostime()

        return frame


if __name__ == '__main__':
    rospy.init_node('can_bridge', anonymous=True)
    pb = PandaBridgePub()
    pb.run()
