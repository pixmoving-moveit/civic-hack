#!/usr/bin/env python

from accum_msgs_tools import filter_accum_keep, filter_messages_keep, to_sendeable_accum
from accum_msgs_tools import filter_accum_remove, filter_messages_remove, to_sendeable_block
from cPickle import load, loads, dump, dumps
import sys

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: ")
        print(sys.argv[0] + " accum.pickle timestamps.pickle")
        exit(0)
    pickle_accum_file = sys.argv[1]
    pickle_timestamps_file = sys.argv[2]
    print("Cleaning up files: " + pickle_accum_file +
          " and " + pickle_timestamps_file)
    # ADAS messages to send
    # STEERING_CONTROL, BRAKE_COMMAND, GAS_COMMAND, ACC_HUD, LKAS_HUD, HIGHBEAM_CONTROL, RADAR_HUD
    ids_ADAS_hex = [0xe4, 0x1fa, 0x200, 0x30c, 0x33d, 0x35e, 0x39f]
    ids_ADAS = [228, 506, 512, 780, 829, 862, 927]
    print("Keeping only ADAS ids: " + str(ids_ADAS))

    accum = load(open(pickle_accum_file, 'r'))
    timestamps = load(open(pickle_timestamps_file, 'r'))
    filtered_accum, filtered_timestamps = filter_accum_keep(
        ids_ADAS, accum, timestamps)

    filtered_fname = pickle_accum_file.replace('.pickle', '_filtered.pickle')
    filtered_t_fname = pickle_timestamps_file.replace(
        '.pickle', '_filtered.pickle')
    print("Writing in: " + filtered_fname + " and " + filtered_t_fname)
    dump(filtered_accum, open(filtered_fname, 'w'))
    dump(filtered_timestamps, open(filtered_t_fname, 'w'))
