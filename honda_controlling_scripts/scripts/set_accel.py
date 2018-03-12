#!/usr/bin/env python

from panda import Panda
from honda_create_msgs import create_gas_command
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

    # What we know about gas:
    # {'frame_id': 512,
    #  'name': 'GAS_COMMAND',
    #  'nodes': ['EON'],
    #  'signals': [signal('GAS_COMMAND', 7, 16, 'big_endian', False, 0.253984064, -83.3, 0, 1, 'None', False, None, None, None),
    #              signal('GAS_COMMAND2', 23, 16, 'big_endian', False, 0.126992032, -83.3, 0, 1, 'None', False, None, None, None),
    #              signal('ENABLE', 39, 1, 'big_endian', False, 1, 0, 0, 1, 'None', False, None, None, None),
    #              signal('COUNTER', 45, 2, 'big_endian', False, 1, 0, 0, 3, 'None', False, None, None, None),
    #              signal('CHECKSUM', 43, 4, 'big_endian', False, 1, 0, 0, 3, 'None', False, None, None, None)]},

    # idx refers to the COUNTER field we see
    gas_cmd = create_gas_command(1, 0)
    print gas_cmd
    # Looks like: [512, 0, '\x00\x01\x05', 0]
    # The message is too short, wtf

    rp = RosPack()
    pkg_path = rp.get_path('panda_bridge_ros')
    can_msg_parser = load_dbc_file(pkg_path + '/config/' +
                                   'honda_civic_touring_2016_can_for_cantools.dbc')

    # # Let's double check the brake_cmd
    # decoded_gas_cmd = can_msg_parser.decode_message(gas_cmd[0],
    #                                                 gas_cmd[2])
    # print decoded_gas_cmd

    # exit(0)

    print("Press Enter to send gas commands")
    idx_counter = 0
    total_cmds_sent = 0
    try:
        while raw_input() == '':
            for i in range(1):
                cmd = create_gas_command(16000, idx_counter)
                print("Sending: " + str(cmd) +
                      " (#" + str(total_cmds_sent) + ") counter: " + str(idx_counter))
                panda.can_send(cmd[0], cmd[2], 0)
                idx_counter += 1
                idx_counter %= 4
                total_cmds_sent += 1
    except KeyboardInterrupt:
        panda.close()
