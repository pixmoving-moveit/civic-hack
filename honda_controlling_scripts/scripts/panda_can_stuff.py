#!/usr/bin/env python

from cantools.db import load_file as load_dbc_file
from honda_create_msgs import create_brake_command, create_gas_command, create_steering_control, create_ui_commands, create_radar_commands
from panda import Panda


class HondaCivicCan(object):
    def __init__(self):
        print("Loading CAN DBC file...")
        self.can_msg_parser = load_dbc_file(
            'honda_civic_touring_2016_can_for_cantools.dbc')
        print("Connecting to Panda board...")
        self.panda = Panda()
        print("Ready.")
        # ROS subscribers to send the commands?

    def decode_msg(self, message):
        """Return a dictionary with the decoded message

        Looks like:
        {'STEER_ANGLE_OFFSET': -0.5, 'CHECKSUM': 4, 'COUNTER': 0, 
        'STEER_ANGLE_RATE': -338, 'STEER_WHEEL_ANGLE': 86.5, 'STEER_ANGLE': 89.4}
        """
        # A message looks like:
        # (1024, 59709, bytearray(b'\x00\x00\x02\x00\x02'), 0)
        return self.can_msg_parser.decode_message(message[0], message[2])

    def encode_msg(self, frame_id, message):
        """Return the message in a dictionary encoded.
        """
        return self.can_msg_parser.encode_message(frame_id, message)

    def run(self):
        self.panda.set_safety_mode(self.panda.SAFETY_HONDA)

        try:
            while True:
                data = self.panda.can_recv()
                for msg in data:
                    decoded_msg = None
                    try:
                        decoded_msg = self.decode_msg(msg)
                    except KeyError:
                        pass
                    except ValueError:
                        pass
                    if decoded_msg:
                        # print decoded_msg
                        # continue
                        for k in decoded_msg.keys():
                            if 'STEER_TORQUE' in k:
                                print decoded_msg
                                print msg
                                print "---"
                                steer_msg = create_steering_control(1000, 0)[0]
                                print steer_msg
                                print self.decode_msg(steer_msg)
                                print
                                self.panda.can_send(steer_msg[0], steer_msg[2], 0)
                                # msg_to_send = self.encode_msg(891, {'CHECKSUM': 15, 'COUNTER': 1, 'WIPERS': 1})
                                # self.panda.can_send(msg[0], '\x00\x00\x01 \x00\x00\x00\x00', 0)



                # for pending_msg in self.pending_msg:
                #     panda.can_send(0xXXXX, pending_msg, 0)
        except KeyboardInterrupt:
            self.panda.close()


if __name__ == '__main__':
    hcc = HondaCivicCan()
    hcc.run()
