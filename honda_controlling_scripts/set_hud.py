#!/usr/bin/env python

from panda import Panda
from honda_create_msgs import create_ui_commands
from collections import namedtuple


"""
Send ui commands on keypress.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


HUDData = namedtuple("HUDData",
                     ["pcm_accel", "v_cruise", "X2", "car", "X4", "X5",
                      "lanes", "beep", "X8", "chime", "acc_alert"])


if __name__ == '__main__':
    print("Connecting to panda...")
    panda = Panda()
    print("Setting safety mode SAFETY_HONDA")
    # panda.set_safety_mode(panda.SAFETY_HONDA)
    panda.set_safety_mode(panda.SAFETY_ALLOUTPUT)

    # if hud_show_lanes:
    hud_lanes = 0x04
    # else:
    #hud_lanes = 0x00

    # TODO: factor this out better
    # if enabled:
    #   if hud_show_car:
    hud_car = 0xe0
    #   else:
    #     hud_car = 0xd0
    # else:
    # hud_car = 0xc0

    # print chime, alert_id, hud_alert
    fcw_display, steer_required, acc_alert = (4, 1, 1)
    mph_shown_screen = 69
    hud = HUDData(int(100), int(round(mph_shown_screen*1.6)), 0x01, hud_car,
                  0xc1, 0x41, hud_lanes + steer_required,
                  int(0), 0x48, (0 << 5) + fcw_display, acc_alert)

    # idx refers to the COUNTER field we see
    # ui_cmd = create_ui_commands(pcm_speed, hud, CS.CP.carFingerprint, idx)
    ui_cmd = create_ui_commands(0, hud, 0)
    print ui_cmd
    # Looks like:
    # [[780, 0, '\x00\x00\x00\x00\x01\xc0\xc1\x0f', 0],
    # [829, 0, 'A\x00\x00H\x04', 0],
    # [862, 0, '\x00\x00\x00\x00\x00\x00\x00\x02', 0],
    # [927, 0, '\x00\x00\x00\x00\xff\x7f\x00\t', 0]]


    print("Press Enter to send ui commands")
    idx_counter = 1
    total_cmds_sent = 0
    try:
        while raw_input() == '':
            print("Sending: " + str(ui_cmd) +
                  " (#" + str(total_cmds_sent) + ")")
            panda.can_send_many(ui_cmd)
            idx_counter += 1
            idx_counter %= 4
            total_cmds_sent += 1
    except KeyboardInterrupt:
        panda.close()
