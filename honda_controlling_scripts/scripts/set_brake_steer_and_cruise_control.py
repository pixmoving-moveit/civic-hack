#!/usr/bin/env python

from panda import Panda
from honda_create_msgs import create_brake_command, create_steering_control, create_buttons_command
import time
import rospy
from std_msgs.msg import Int16
from cantools.db import load_file as load_dbc_file
from rospkg import RosPack
from sensor_msgs.msg import Joy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

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
        self.cruise_modifier = 0.0
        self.sub_engine = rospy.Subscriber('/joy', Joy,
                                           self.engine_cb, queue_size=1)
        self.sub_steer = rospy.Subscriber(
            '/steering_amount', Int16, self.steer_cb, queue_size=1)
        self.sub_brake = rospy.Subscriber(
            '/braking_amount', Int16, self.brake_cb, queue_size=1)
        self.ddr = DDynamicReconfigure("drive_car")
        self.ddr.add_variable("rate_buttons", 
            "rate as to which send the button presses for cruise control",
             1, 1, 100)

        self.add_variables_to_self()

        self.ddr.start(self.dyn_rec_callback)

    def add_variables_to_self(self):
        var_names = self.ddr.get_variable_names()
        for var_name in var_names:
            self.__setattr__(var_name, None)

    def dyn_rec_callback(self, config, level):
        rospy.loginfo("Received reconf call: " + str(config))
        # Update all variables
        var_names = self.ddr.get_variable_names()
        for var_name in var_names:
            self.__dict__[var_name] = config[var_name]
        return config

    def engine_cb(self, data):
        self.cruise_modifier = data.axes[1]

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
        total_cmds_sent = 0
        iterations = 0
        last_rate = time.time()
        while not rospy.is_shutdown():

            #print("rate: " + str(self.rate_buttons))
            if iterations % (100 / self.rate_buttons) == 0:
                if self.cruise_modifier != 0.0:
                    if self.cruise_modifier > 0.0:
                        cruise_command = 'accel_res'
                    elif self.cruise_modifier < 0.0:
                        cruise_command = 'decel_set'

                    settings_command = 'none'
                    print("Sending cruise command: " + cruise_command)

                    # (xmission_speed, engine_rpm=2000, odometer=3, idx=0):
                    cmd = create_buttons_command(cruise_command, settings_command, idx_counter)
                    print("command is: " + str(cmd))
                    print("Sending: " + str(cmd) +
                          " (#" + str(total_cmds_sent) +
                          ") engine modifier val: " + str(self.cruise_modifier))
                    self.p.can_send(cmd[0], cmd[2], 1)

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
            time.sleep(0.01)
            iterations += 1


if __name__ == '__main__':
    rospy.init_node('all_sender')
    sp = AllPublisher()
    sp.run()
