#!/usr/bin/env python

from panda import Panda
from cPickle import dump, dumps, load, loads
import time
import sys
from accum_msgs_tools import to_sendeable_block, to_sendeable_accum

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage:")
        print(sys.argv[0] + ' accum_filename bus [scale]')
        print("Replay time scale defaults to 1.0.")
        exit(0)
    fname = sys.argv[1]
    fname = fname.replace('.pickle', '')
    accum_fname = fname + ".pickle"
    timestamps_fname = fname + "_timestamps.pickle"
    accum = load(open(accum_fname, 'r'))
    timestamps = load(open(timestamps_fname, 'r'))
    print("Loaded files...")

    print("Connecting to Panda...")
    p = Panda()

    CAMERA_CAN = 1
    OTHER_CAN = 0

    bus = int(sys.argv[2])
    scale = 1.0
    if len(sys.argv) == 4:
        scale = float(sys.argv[3])
    # prepare msgs
    print("Publishing...")
    accum_to_send = to_sendeable_accum(accum, can_bus=bus)

    total_msgs = len(accum_to_send)
    try:
        for idx, block_msg in enumerate(accum_to_send):
            if idx < len(accum_to_send):
                ini_t = timestamps[idx]
                next_t = timestamps[idx + 1]
                sleep_time = next_t - ini_t
                # print(block_msg)
                p.can_send_many(block_msg)
                # print("sleeping for: " + str(sleep_time))
                time.sleep(sleep_time * scale)
            else:
                p.can_send_many(block_msg)
            # Give some feedback
            percent_msgs = int((float(idx) / float(total_msgs)) * 100.0)
            if percent_msgs % 5 == 0 and percent_msgs > 0:
                print("Replayed " + str(percent_msgs) + "% of messages...")
                print("(" + str(idx) + " / " + str(total_msgs) +
                      " blocks of up to 256 msgs)")
    except KeyboardInterrupt:
        p.close()

    print("Done")
