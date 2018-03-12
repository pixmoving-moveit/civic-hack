#!/usr/bin/env python
import rospy
from can_msgs.msg import Frame
from geometry_msgs.msg import TwistStamped
import cantools
import math

wheel_angle = 0
DBW_mode = 0
wheel_speed = 0
wheel_angle_2 = 0

msg1_id = 464
msg2_id = 330
counter_msg1 = 0
counter_msg2 = 0
car_length = 2.1 # should be changed

def callback(msg):
    msgID = msg.id
    if (msgID == msg1_id):
        global counter_msg1
        counter_msg1 = 1
        data = can_db.decode_message(msgID, msg.data)
        speed_RL = data['WHEEL_SPEED_FL']
        speed_RR = data['WHEEL_SPEED_FR']
        global wheel_speed
        wheel_speed = (speed_RL+speed_RR)/2


    if (msgID == msg2_id):
        global counter_msg2
        counter_msg2 = 1
        data = can_db.decode_message(msgID, msg.data)
        global wheel_angle
        wheel_angle = data['STEER_WHEEL_ANGLE']



    # else:
        # print("nothing found!!")






if __name__ == '__main__':
    can_db = cantools.db.load_file('./dbc/honda_civic_touring_2016_can_for_cantools.dbc')

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("can_frame_msgs", Frame, callback)
    pub = rospy.Publisher("twist_stamped", TwistStamped)
    twist_msg = TwistStamped()
    rate = rospy.Rate(10)
    global car_length
    while not rospy.is_shutdown():
        if (counter_msg1 + counter_msg2 >= 2):
            twist_msg.twist.linear.x = wheel_speed*0.277778
            wheel_angle = (wheel_angle*50)/405.0
            twist_msg.twist.angular.z = (twist_msg.twist.linear.x*(
                math.tan((wheel_angle*math.pi)/180.0))/(car_length))
            print("Wheel angle: " + str(wheel_angle))
            print("z: " + str(twist_msg.twist.angular.z))
            pub.publish(twist_msg)
        if (counter_msg1 == 1):
            print("msg1(391) only rec.")
        if (counter_msg2 == 1):
            print("msg2(387) only rec.")
        else:
            print("nothing rec.")
        rate.sleep()
