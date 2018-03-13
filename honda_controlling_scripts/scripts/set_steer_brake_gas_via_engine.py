#!/usr/bin/env python

from panda import Panda
from honda_create_msgs import create_brake_command, create_steering_control, create_engine_data
import time
import rospy
from std_msgs.msg import Int16
from cantools.db import load_file as load_dbc_file
from rospkg import RosPack
from sensor_msgs.msg import Joy

"""

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class AllPublisher(object):
    def __init__(self):
        rp = RosPack()
        pkgpath = rp.get_path('panda_bridge_ros')
        pkgpath += '/config/honda_civic_touring_2016_can_for_cantools.dbc'
        self.parser = load_dbc_file(pkgpath)
        self.p = Panda()
        self.p.set_safety_mode(self.p.SAFETY_ALLOUTPUT)
        self.gas_val = 0
        self.brake_val = 0
        self.steer_val = 0
        self.engine_modifier = 0.0
        self.sub_engine = rospy.Subscriber('/joy', Joy,
                                           self.engine_cb, queue_size=1)
        self.sub_steer = rospy.Subscriber(
            '/steering_amount', Int16, self.steer_cb, queue_size=1)
        self.sub_brake = rospy.Subscriber(
            '/braking_amount', Int16, self.brake_cb, queue_size=1)

    def engine_cb(self, data):
        self.engine_modifier = data.axes[1]

    def steer_cb(self, data):
        # print("steer_cb: " + str(data))
        if data.data > 3840:
            self.steer_val = 3840
        elif data.data < -3840:
            self.steer_val = -3840
        self.steer_val = data.data

    def brake_cb(self, data):
        # print("brake_cb: " + str(data))
        if data.data > 1023:
            self.brake_val = 1023
        elif data.data < 0:
            self.brake_val = 0
        self.brake_val = data.data

    def run(self):
        print("Publishing...")
        idx_counter = 0
        idx_counter_engine = 0
        total_cmds_sent = 0
        last_received_xmission_speed = 0
        last_received_odometer = 0
        last_received_xmission_speed_2 = 0
        last_engine_rpm = 0
        iterations = 0
        while not rospy.is_shutdown():
            # receive first
            block = self.p.can_recv()
            for msg in block:
                # if its an engine message
                if msg[0] == 344 and msg[3] == 0:
                    fields = self.parser.decode_message(msg[0], msg[2])
                    # print("we read engine message with speed:")
                    # print(fields['XMISSION_SPEED'])
                    last_received_xmission_speed = fields['XMISSION_SPEED']
                    last_received_odometer = fields['ODOMETER']
                    last_received_xmission_speed_2 = fields['XMISSION_SPEED2']
                    last_engine_rpm = fields['ENGINE_RPM']
                    break

            # Engine rpm stuff
            if self.engine_modifier != 0.0:
                # we need to invert the modifier signal
                # cause we are tricking the cruise control to go at the speed we want
                # so we need to create a engine data message that ssays
                # we are driving slower than we should in order for it to accelerate
                # limit to +- 15kmph the max acceleration requested
                speed = last_received_xmission_speed + \
                    (15.0 * self.engine_modifier * -1.0)
                print("latest_received_xmission_speed: " +
                      str(last_received_xmission_speed))
                print("we modify by: " + str((15.0 * self.engine_modifier * -1.0)))
                print("which results in: " + str(speed))

                # (xmission_speed, engine_rpm=2000, odometer=3, idx=0):
                cmd = create_engine_data(
                    speed, last_engine_rpm, last_received_odometer, idx_counter_engine)
                idx_counter_engine += 1
                idx_counter_engine %= 4
                print("command is: " + str(cmd))
                print("Sending: " + str(cmd) +
                      " (#" + str(total_cmds_sent) +
                      ") engine modifier val: " + str(self.engine_modifier))
                print("speed:" + str(speed) + " rpm: " + str(last_engine_rpm))
                self.p.can_send(cmd[0], cmd[2], 0)

            if iterations % 5 == 0:
                cmd = create_steering_control(self.steer_val, idx_counter)[0]
                # print("Sending: " + str(cmd) +
                #       " (#" + str(total_cmds_sent) +
                #       ") steer val: " + str(self.steer_val))
                self.p.can_send(cmd[0], cmd[2], 1)

                # (apply_brake, pcm_override, pcm_cancel_cmd, chime, idx):
                cmd = create_brake_command(
                    self.brake_val, 1, 0, 0, idx_counter)
                # print("Sending: " + str(cmd) +
                #       " (#" + str(total_cmds_sent) +
                #       ") brake val: " + str(self.brake_val))
                self.p.can_send(cmd[0], cmd[2], 1)

                idx_counter += 1
                idx_counter %= 4
            # its wrong, but who cares
            total_cmds_sent += 1
            time.sleep(0.002)
            iterations += 1


if __name__ == '__main__':
    rospy.init_node('all_sender')
    sp = AllPublisher()
    sp.run()
