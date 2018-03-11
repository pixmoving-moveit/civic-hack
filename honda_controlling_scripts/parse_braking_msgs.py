#!/usr/bin/env python

from std_msgs.msg import String
import rospy
from ast import literal_eval

if __name__ == '__main__':
    rospy.init_node('parse_braking_msgs')

    def callback_(data):
        d = literal_eval(data.data)
        if d['frame_id'] == 506:
            print("-------START----")
            print(d['frame_id'])
            print(d['message_name'])
            print('COUNTER: ' + str(d['COUNTER']))
            print('COMPUTER_BRAKE_REQUEST: ' + str(d['COMPUTER_BRAKE_REQUEST']))
            print('COMPUTER_BRAKE: ' + str(d['COMPUTER_BRAKE']))
            print('BRAKE_LIGHTS: ' + str(d['BRAKE_LIGHTS']))
            print('CHIME: ' + str(d['CHIME']))
            print(d)
            print("-------END-----")

    sub = rospy.Subscriber('/can_frame_msgs_human_friendly',
                           String, callback_, queue_size=10)

    rospy.spin()
