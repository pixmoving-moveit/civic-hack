#!/usr/bin/env python

from can_msgs.msg import Frame
from cantools.db import load_file as load_dbc_file
from rospkg import RosPack
import rospy
from std_msgs.msg import String
from pprint import PrettyPrinter

"""
Node to subscribe to a can_msgs/Frame topic showing the
CAN messages from a Honda Civic 2016 in a dictionary like format
in a topic string.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
Author: Pier-Marc Comtois-Rivet <pm.rivet at gmail.com>
"""


class FrameDecoder(object):
    def __init__(self):
        rp = RosPack()
        pkg_path = rp.get_path('panda_bridge_ros')
        self.can_msg_parser = load_dbc_file(pkg_path + '/config/' +
                                            'honda_civic_touring_2016_can_for_cantools.dbc')
        self.pub = rospy.Publisher('can_frame_msgs_human_friendly', String,
                                   queue_size=10)
        self.frame_sub = rospy.Subscriber('can_frame_msgs', Frame,
                                          self._cb, queue_size=10)
        self.msg_id_to_msg_name = {}
        understood_msgs = []
        for msg in self.can_msg_parser.messages:
            this_msg = {}
            this_msg['frame_id'] = msg.frame_id
            this_msg['name'] = msg.name
            this_msg['signals'] = msg.signals
            this_msg['nodes'] = msg.nodes
            understood_msgs.append(this_msg)
            self.msg_id_to_msg_name[msg.frame_id] = msg.name

        rospy.loginfo("We can interpret the messages:")
        pp = PrettyPrinter()
        rospy.loginfo(pp.pformat(understood_msgs))

    def _cb(self, data):
        # message looks like:
        # header:
        #   seq: 10450
        #   stamp:
        #     secs: 1520743044
        #     nsecs: 188934087
        #   frame_id: ''
        # id: 304
        # is_rtr: False
        # is_extended: False
        # is_error: False
        # dlc: 8
        # data: [0, 0, 4, 92, 110, 16, 4, 36]
        try:
            msg = self.can_msg_parser.decode_message(data.id, data.data)
            msg['frame_id'] = data.id
            msg['message_name'] = self.msg_id_to_msg_name[data.id]
            msg['raw_msg'] = str(data.data)
            human_friendly = str(msg)
            # string will look like:
            # {'message_name': 'STEERING_CONTROL', 'CHECKSUM': 5, 'COUNTER': 1,
            # 'STEER_TORQUE': 0, 'frame_id': 228, 'SET_ME_X00': 0,
            # 'raw_msg': '\x00\x00\x00\x00\x15\x00\x00\x00',
            # 'STEER_TORQUE_REQUEST': 0, 'SET_ME_X00_2': 0}
        except KeyError:
            msg = {}
            msg['frame_id'] = data.id
            msg['message_name'] = "UNKNOWN_MESSAGE"
            msg['raw_msg'] = str(data.data)
            human_friendly = str(msg)
        except ValueError:
            msg = {}
            msg['frame_id'] = data.id
            msg['message_name'] = "UNKNOWN_MESSAGE"
            msg['raw_msg'] = str(data.data)
            human_friendly = str(msg)

        self.pub.publish(human_friendly)


if __name__ == '__main__':
    rospy.init_node('frame_decoder')
    fd = FrameDecoder()
    rospy.spin()
