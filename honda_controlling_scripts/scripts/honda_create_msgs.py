import struct

# Taken from openpilot project


def clip(x, lo, hi):
    return max(lo, min(hi, x))

# *** Honda specific ***


def can_cksum(mm):
    s = 0
    for c in mm:
        c = ord(c)
        s += (c >> 4)
        s += c & 0xF
    s = 8 - s
    s %= 0x10
    return s


def fix(msg, addr):
    msg2 = msg[0:-1] + \
        chr(ord(msg[-1]) | can_cksum(struct.pack("I", addr) + msg))
    return msg2


def make_can_msg(addr, dat, idx, alt):
    if idx is not None:
        dat += chr(idx << 4)
        dat = fix(dat, addr)
    return [addr, 0, dat, alt]


def create_brake_command(apply_brake, pcm_override, pcm_cancel_cmd, chime, idx):
    """Creates a CAN message for the Honda DBC BRAKE_COMMAND."""
    pump_on = apply_brake > 0
    brakelights = apply_brake > 0
    brake_rq = apply_brake > 0

    pcm_fault_cmd = False
    amount = struct.pack("!H", (apply_brake << 6) + pump_on)
    msg = amount + struct.pack("BBB", (pcm_override << 4) |
                               (pcm_fault_cmd << 2) |
                               (pcm_cancel_cmd << 1) | brake_rq, 0x80,
                               brakelights << 7) + chr(chime) + "\x00"
    return make_can_msg(0x1fa, msg, idx, 0)


def create_buttons_command(cruise_button_press, cruise_setting, idx):
    # cruise_button_press can be accel_res, decel_set, cancel, main, none
    # cruise_Setting can be distance_adj, lkas_button, none
    # {'frame_id': 662,
    #  'name': 'SCM_BUTTONS',
    #  'nodes': ['SCM'],
    #  'signals': [signal('CRUISE_BUTTONS', 7, 3, 'big_endian', False, 1, 0, 0, 7, 'None', False, None, {7: 'tbd', 6: 'tbd', 5: 'tbd', 4: 'accel_res', 3: 'decel_set', 2: 'cancel', 1: 'main', 0: 'none'}, None),
    #              signal('CRUISE_SETTING', 3, 2, 'big_endian', False, 1, 0, 0, 3, 'None', False, None, {3: 'distance_adj', 2: 'tbd', 1: 'lkas_button', 0: 'none'}, None),
    #              signal('COUNTER', 29, 2, 'big_endian', False, 1, 0, 0, 3, 'None', False, None, None, None),
    #              signal('CHECKSUM', 27, 4, 'big_endian', False, 1, 0, 0, 3, 'None', False, None, None, None)]},

    d = {7: 'tbd', 6: 'tbd', 5: 'tbd', 4: 'accel_res', 3: 'decel_set', 2: 'cancel', 1: 'main', 0: 'none'}
    button_name_to_id = {}
    for k in d.keys():
        button_name_to_id[d[k]] = k
    cruise_set_to_id = {}
    d = {3: 'distance_adj', 2: 'tbd', 1: 'lkas_button', 0: 'none'}
    for k in d.keys():
        cruise_set_to_id[d[k]] = k

    cruise_b = button_name_to_id[cruise_button_press]
    cruise_set = cruise_set_to_id[cruise_setting]
    total = cruise_b << 5 | cruise_set << 2

    msg = struct.pack("!BBB", total, 0, 0)

    return make_can_msg(662, msg, idx, 0)


def create_gas_command(gas_amount, idx):
    """Creates a CAN message for the Honda DBC GAS_COMMAND."""
    # enable_bit = 1 << 7
    # max_gas = 176.526
    # offset = -83.3
    # scaled_offset = offset / 1023.0 * max_gas
    # offset_raw = 21
    # offset2_raw = 11
    # print(offset_raw)
    # print(scaled_offset)
    # gas_amount_ = (float(gas_amount) / 1023.0) * max_gas
    # print(gas_amount_)
    # gas_amount_2 = (float(gas_amount) / 1023.0) * max_gas / 2.0
    # gas_amount_ = gas_amount_ + offset_raw
    # print(gas_amount_)
    # gas_amount_2 = gas_amount_2 + offset2_raw

    # gas_amount_ = int(gas_amount_ / max_gas * 1023.0)
    # gas_amount_2 = int(gas_amount_2 / max_gas * 1023.0)
    # print("gas 1 gas 2")
    # print(gas_amount_)
    # print(gas_amount_2)

    offset1_raw = 328  # 21
    offset2_raw = 656  # 11
    gas_amount_1 = gas_amount + offset1_raw
    gas_amount_2 = gas_amount * 2 + offset2_raw
    if gas_amount <= 0:
        enable_bit = 0
    else:
        enable_bit = 1 << 7

    if gas_amount <= 0:
        enable_bit = 0
    msg = struct.pack("!HHB", gas_amount_1, gas_amount_2, enable_bit)
    return make_can_msg(0x200, msg, idx, 0)

def create_engine_data(xmission_speed, engine_rpm=2000, odometer=3, idx=0):
    # Looks like:
    # (344, 827, "\x15[\x07^\x15;\xa3'", 1)
    # {'CHECKSUM': 7, 'ENGINE_RPM': 1886, 'COUNTER': 2, 'XMISSION_SPEED': 54.67, 'ODOMETER': 1.725844, 'XMISSION_SPEED2': 54.35}

    # xmission_speed =  # 0 - 25000, 250kph
    xmission_speed *= 100.0
    if xmission_speed < 0:
        xmission_speed = 0
    odometer *= 100.0
    if odometer > 255:
        odometer = 254
    xmission_speed2 = xmission_speed # 0 - 25000
    engine_rpm = engine_rpm  # 2000~
    # odometer 0 - 255km
    # struct.pack("!H", xmission_speed)
    # struct.pack("!H", engine_rpm)
    # struct.pack("!H", xmission_speed2)


    msg = struct.pack("!HHHB", xmission_speed, engine_rpm, xmission_speed2, odometer)
    return make_can_msg(0x158, msg, idx, 0)


def create_steering_control(apply_steer, idx):
    """Creates a CAN message for the Honda DBC STEERING_CONTROL."""
    commands = []
    # if car_fingerprint in (CAR.CRV, CAR.ACURA_RDX):
    #     msg_0x194 = struct.pack("!h", apply_steer << 4) + \
    #         ("\x80" if apply_steer != 0 else "\x00")
    #     commands.append(make_can_msg(0x194, msg_0x194, idx, 0))
    # else:
    msg_0xe4 = struct.pack("!h", apply_steer) + \
        ("\x80\x00" if apply_steer != 0 else "\x00\x00")
    commands.append(make_can_msg(0xe4, msg_0xe4, idx, 0))
    return commands


def create_ui_commands(pcm_speed, hud, idx):
    """Creates an iterable of CAN messages for the UIs."""
    commands = []
    pcm_speed_real = clip(int(round(pcm_speed / 0.002759506)), 0,
                          64000)  # conversion factor from dbc file
    msg_0x30c = struct.pack("!HBBBBB", pcm_speed_real, hud.pcm_accel,
                            hud.v_cruise, hud.X2, hud.car, hud.X4)
    commands.append(make_can_msg(0x30c, msg_0x30c, idx, 0))

    msg_0x33d = chr(hud.X5) + chr(hud.lanes) + chr(hud.beep) + chr(hud.X8)
    commands.append(make_can_msg(0x33d, msg_0x33d, idx, 0))
    # if car_fingerprint in (CAR.CIVIC, CAR.ODYSSEY):
    msg_0x35e = chr(0) * 7
    commands.append(make_can_msg(0x35e, msg_0x35e, idx, 0))
    msg_0x39f = (chr(0) * 2 + chr(hud.acc_alert) +
                 chr(0) + chr(0xff) + chr(0x7f) + chr(0))
    commands.append(make_can_msg(0x39f, msg_0x39f, idx, 0))
    return commands


def create_radar_commands(v_ego, idx):
    """Creates an iterable of CAN messages for the radar system."""
    commands = []
    v_ego_kph = clip(int(round(v_ego * 3.6)), 0, 255)
    speed = struct.pack('!B', v_ego_kph)

    msg_0x300 = ("\xf9" + speed + "\x8a\xd0" +
                 ("\x20" if idx == 0 or idx == 3 else "\x00") +
                 "\x00\x00")

    # if car_fingerprint == CAR.CIVIC:
    msg_0x301 = "\x02\x38\x44\x32\x4f\x00\x00"
    # add 8 on idx.
    commands.append(make_can_msg(0x300, msg_0x300, idx + 8, 1))
    # elif car_fingerprint == CAR.CRV:
    #     msg_0x301 = "\x00\x00\x50\x02\x51\x00\x00"
    #     commands.append(make_can_msg(0x300, msg_0x300, idx, 1))
    # elif car_fingerprint == CAR.ACURA_RDX:
    #     msg_0x301 = "\x0f\x57\x4f\x02\x5a\x00\x00"
    #     commands.append(make_can_msg(0x300, msg_0x300, idx, 1))
    # elif car_fingerprint == CAR.ODYSSEY:
    #     msg_0x301 = "\x00\x00\x56\x02\x55\x00\x00"
    #     commands.append(make_can_msg(0x300, msg_0x300, idx, 1))
    # elif car_fingerprint == CAR.ACURA_ILX:
    #     msg_0x301 = "\x0f\x18\x51\x02\x5a\x00\x00"
    #     commands.append(make_can_msg(0x300, msg_0x300, idx, 1))
    # elif car_fingerprint == CAR.PILOT:
    #     msg_0x301 = "\x00\x00\x56\x02\x58\x00\x00"
    #     commands.append(make_can_msg(0x300, msg_0x300, idx, 1))

    commands.append(make_can_msg(0x301, msg_0x301, idx, 1))
    return commands
