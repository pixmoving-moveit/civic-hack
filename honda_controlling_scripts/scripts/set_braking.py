#!/usr/bin/env python

from panda import Panda
from honda_create_msgs import create_brake_command
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

    # What we know about braking:
    #  {'frame_id': 506,
    # 'name': 'BRAKE_COMMAND',
    # 'nodes': ['ADAS'],
    # 'signals': [signal('COMPUTER_BRAKE', 7, 10, 'big_endian', False, 1, 0, 0, 1, 'None', False, None, None, None),
    #             signal('ZEROS_BOH', 13, 5, 'big_endian', False, 1, 0, 0, 1, 'None', False, None, None, None),
    #             signal('COMPUTER_BRAKE_REQUEST', 8, 1, 'big_endian', False, 1, 0, 0, 1, 'None', False, None, None, None),
    #             signal('CRUISE_BOH2', 23, 3, 'big_endian', False, 1, 0, 0, 1, 'None', False, None, None, None),
    #             signal('CRUISE_OVERRIDE', 20, 1, 'big_endian', False, 1, 0, 0, 1, 'None', False, None, None, None),
    #             signal('CRUISE_BOH3', 19, 1, 'big_endian', False, 1, 0, 0, 1, 'None', False, None, None, None),
    #             signal('CRUISE_FAULT_CMD', 18, 1, 'big_endian', False, 1, 0, 0, 1, 'None', False, None, None, None),
    #             signal('CRUISE_CANCEL_CMD', 17, 1, 'big_endian', False, 1, 0, 0, 1, 'None', False, None, None, None),
    #             signal('COMPUTER_BRAKE_REQUEST_2', 16, 1, 'big_endian', False, 1, 0, 0, 1, 'None', False, None, None, None),
    #             signal('SET_ME_0X80', 31, 8, 'big_endian', False, 1, 0, 0, 1, 'None', False, None, None, None),
    #             signal('BRAKE_LIGHTS', 39, 1, 'big_endian', False, 1, 0, 0, 1, 'None', False, None, None, None),
    #             signal('CRUISE_STATES', 38, 7, 'big_endian', False, 1, 0, 0, 1, 'None', False, None, None, None),
    #             signal('CHIME', 47, 3, 'big_endian', False, 1, 0, 0, 7, 'None', False, None, {4: 'double_chime', 3: 'single_chime', 2: 'continuous_chime', 1: 'repeating_chime', 0: 'no_chime'}, None),
    #             signal('ZEROS_BOH6', 44, 1, 'big_endian', False, 1, 0, 0, 1, 'None', False, None, None, None),
    #             signal('FCW', 43, 2, 'big_endian', False, 1, 0, 0, 3, 'None', False, None, {3: 'fcw', 2: 'fcw', 1: 'fcw', 0: 'no_fcw'}, None),
    #             signal('ZEROS_BOH4', 55, 8, 'big_endian', False, 1, 0, 0, 0, 'None', False, None, None, None),
    #             signal('COUNTER', 61, 2, 'big_endian', False, 1, 0, 0, 3, 'None', False, None, None, None),
    #             signal('CHECKSUM', 59, 4, 'big_endian', False, 1, 0, 0, 15, 'None', False, None, None, None)]}

    #                    apply_brake, pcm_override, pcm_cancel_cmd, chime, idx
    # idx refers to the COUNTER field we see
    brake_cmd = create_brake_command(True, True, True, 0, 0)
    # Looks like: [506, 0, '\x00A\x13\x80\x80\x00\x00\x05', 0]

    rp = RosPack()
    pkg_path = rp.get_path('panda_bridge_ros')
    can_msg_parser = load_dbc_file(pkg_path + '/config/' +
                                   'honda_civic_touring_2016_can_for_cantools.dbc')

    # Let's double check the brake_cmd
    decoded_brake_cmd = can_msg_parser.decode_message(brake_cmd[0],
                                                      brake_cmd[2])
    # {'BRAKE_LIGHTS': 1,
    # 'CHECKSUM': 5,
    # 'CHIME': 'no_chime',
    # 'COMPUTER_BRAKE': 1,
    # 'COMPUTER_BRAKE_REQUEST': 1,
    # 'COMPUTER_BRAKE_REQUEST_2': 1,
    # 'COUNTER': 0,
    # 'CRUISE_BOH2': 0,
    # 'CRUISE_BOH3': 0,
    # 'CRUISE_CANCEL_CMD': 1,
    # 'CRUISE_FAULT_CMD': 0,
    # 'CRUISE_OVERRIDE': 1,
    # 'CRUISE_STATES': 0,
    # 'FCW': 'no_fcw',
    # 'SET_ME_0X80': 128,
    # 'ZEROS_BOH': 0,
    # 'ZEROS_BOH4': 0,
    # 'ZEROS_BOH6': 0}

    print("Press Enter to send braking commands")
    idx_counter = 1
    total_cmds_sent = 0
    try:
        while raw_input() == '':
            for i in range(100):
                cmd = create_brake_command(True, True, True, 0, idx_counter)
                print("Sending: " + str(cmd) + " (#" + str(total_cmds_sent) + ")")
                panda.can_send(cmd[0], cmd[2], 0)
                idx_counter += 1
                idx_counter %= 4
                total_cmds_sent += 1
    except KeyboardInterrupt:
        panda.close()
