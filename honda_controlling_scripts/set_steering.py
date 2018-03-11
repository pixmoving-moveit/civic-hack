#!/usr/bin/env python

from panda import Panda
from honda_create_msgs import create_steering_control
from rospkg import RosPack
from cantools.db import load_file as load_dbc_file

"""
Send braking commands on keypress.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""

if __name__ == '__main__':
    print("Connecting to panda...")
    panda = Panda()
    print("Setting safety mode SAFETY_HONDA")
    # panda.set_safety_mode(panda.SAFETY_HONDA)
    panda.set_safety_mode(panda.SAFETY_ALLOUTPUT)

    # What we know about steering:
    #  {'frame_id': 228,
    # 'name': 'STEERING_CONTROL',
    # 'nodes': ['ADAS'],
    # 'signals': [signal('STEER_TORQUE', 7, 16, 'big_endian', True, 1, 0, -3840, 3840, 'None', False, None, None, None),
    #             signal('STEER_TORQUE_REQUEST', 23, 1, 'big_endian', False, 1, 0, 0, 1, 'None', False, None, None, None),
    #             signal('SET_ME_X00', 22, 7, 'big_endian', False, 1, 0, 0, 127, 'None', False, None, None, None),
    #             signal('SET_ME_X00_2', 31, 8, 'big_endian', False, 1, 0, 0, 0, 'None', False, None, None, None),
    #             signal('COUNTER', 37, 2, 'big_endian', False, 1, 0, 0, 3, 'None', False, None, None, None),
    #             signal('CHECKSUM', 35, 4, 'big_endian', False, 1, 0, 0, 15, 'None', False, None, None, None)]},

    #                    apply_steer, idx
    # idx refers to the COUNTER field we see
    steer_cmd = create_steering_control(3840, 0)[0]
    # print steer_cmd
    # Looks like: [228, 0, '\x0f\x00\x80\x00\x0f', 0]

    rp = RosPack()
    pkg_path = rp.get_path('panda_bridge_ros')
    can_msg_parser = load_dbc_file(pkg_path + '/config/' +
                                   'honda_civic_touring_2016_can_for_cantools.dbc')

    # Let's double check the steer_cmd
    decoded_steer_cmd = can_msg_parser.decode_message(steer_cmd[0],
                                                      steer_cmd[2])

    # {'CHECKSUM': 15, 'COUNTER': 0, 'STEER_TORQUE': 3840, 'SET_ME_X00': 0, 'STEER_TORQUE_REQUEST': 1, 'SET_ME_X00_2': 0}

    print("Press Enter to send steering commands, enter a number to change the steer value")
    idx_counter = 1
    total_cmds_sent = 0
    steer_val = 3840
    try:
        while True:
            inp = raw_input()
            if inp == '':
                for i in range(100):
                    cmd = create_steering_control(steer_val, idx_counter)[0]
                    print("Sending: " + str(cmd) +
                          " (#" + str(total_cmds_sent) + ") steer val: " + str(steer_val))
                    panda.can_send(cmd[0], cmd[2], 0)
                    idx_counter += 1
                    idx_counter %= 4
                    total_cmds_sent += 1
            else:
                print("New steer value: " + inp)
                steer_val = int(inp)
    except KeyboardInterrupt:
        panda.close()
