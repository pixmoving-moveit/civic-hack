#!/usr/bin/env python

from can_msgs.msg import Frame
from cantools.db import load_file as load_dbc_file
import rospy
from std_msgs.msg import String , Float32 


class FrameDecoder(object):
    def __init__(self):
        self.can_msg_parser = load_dbc_file(
            'honda_civic_touring_2016_can_for_cantools.dbc')
        self.pub = rospy.Publisher('can_frame_msgs_human_friendly', String,
                                   queue_size=10)
        self.pub = rospy.Publisher('can_frame_msgs_human_friendly', String,
                                   queue_size=10)
        self.frame_sub = rospy.Subscriber('can_frame_msgs', Frame,
                                          self._cb, queue_size=10)

    def _cb(self, data):
        try:
            human_friendly = str(
                self.can_msg_parser.decode_message(data.id, data.data))
        except KeyError:
            human_friendly = "ID: " + str(data.id) + " not known"
        except ValueError:
            human_friendly = "ID: " + str(data.id) + " not known"

        self.pub.publish(human_friendly)


if __name__ == '__main__':
    rospy.init_node('frame_decoder')
    fd = FrameDecoder()
    rospy.spin()
