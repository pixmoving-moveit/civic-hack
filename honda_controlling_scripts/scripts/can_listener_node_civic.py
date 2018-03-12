#!/usr/bin/env python
import rospy
from can_msgs.msg import Frame
from geometry_msgs.msg import TwistStamped
import cantools
import math
from rospkg import RosPack

"""
Node that subscribes to the can_frame_msgs and extracts
the information from the wheels and the steering wheel
to compose a TwistStamped message.

Author: Ahmed Radwan <ahmed.ali.radwan94 at gmail.com>
Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class TwistFromCan(object):
    # Constants
    WHEEL_SPEED_MSG_ID = 464
    STEER_WHEEL_MSG_ID = 330
    # should be changed, this is for the coffee car currently.
    CAR_LENGTH = 2.1
    KPH_TO_MS = 0.277778
    STEER_WHEEL_ANGLE_TO_WHEEL_ANGLE = 50
    MAX_WHEEL_ANGLE = 405.0

    def __init__(self):
        rospy.loginfo("Initializing TwistFromCan")
        rp = RosPack()
        pkg_path = rp.get_path('panda_bridge_ros')
        path_to_dbc = pkg_path + '/config/honda_civic_touring_2016_can_for_cantools.dbc'
        self.can_db = cantools.db.load_file(path_to_dbc)
        self.wheel_speed = None
        self.wheel_angle = None
        self.sub = rospy.Subscriber("can_frame_msgs", Frame,
                                    self.callback, queue_size=10)
        self.pub = rospy.Publisher("twist_stamped",
                                   TwistStamped, queue_size=10)
        rospy.loginfo("Done")

    def callback(self, msg):
        msgID = msg.id
        if (msgID == self.WHEEL_SPEED_MSG_ID):
            data = self.can_db.decode_message(msgID, msg.data)
            speed_RL = data['WHEEL_SPEED_FL']
            speed_RR = data['WHEEL_SPEED_FR']
            self.wheel_speed = (speed_RL + speed_RR) / 2.0

        if (msgID == self.STEER_WHEEL_MSG_ID):
            data = self.can_db.decode_message(msgID, msg.data)
            self.wheel_angle = data['STEER_WHEEL_ANGLE']

    def run(self):
        rospy.loginfo("Running!")
        twist_msg = TwistStamped()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.wheel_speed and self.wheel_angle:
                x = self.wheel_speed * self.KPH_TO_MS
                self.wheel_angle = (
                    self.wheel_angle * self.STEER_WHEEL_ANGLE_TO_WHEEL_ANGLE) / self.MAX_WHEEL_ANGLE
                z = x * (math.tan(math.radians(self.wheel_angle)) /
                         self.CAR_LENGTH)

                twist_msg.twist.linear.x = x
                twist_msg.twist.angular.z = z
                print("Wheel speed: " + str(self.wheel_speed))
                print("Wheel angle: " + str(self.wheel_angle))
                print("z: " + str(twist_msg.twist.angular.z))
                self.pub.publish(twist_msg)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    tfc = TwistFromCan()
    tfc.run()
