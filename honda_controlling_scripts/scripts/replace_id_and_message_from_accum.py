#!/usr/bin/env python

from accum_msgs_tools import replace_messages_by_id
from cPickle import load, dump
import sys

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: ")
        print(sys.argv[0] + " accum.pickle frame_id new_message")
        exit(0)
    pickle_accum_file = sys.argv[1]
    pickle_timestamps_file = pickle_accum_file.replace('.pickle', '_timestamps.pickle')
    frame_id = int(sys.argv[2])
    new_message = sys.argv[3]
    print("Replacing all messages with frame_id: " + str(frame_id))
    print("To have data field '" + str(new_message) + "'")
    print len(new_message)
    import pdb;pdb.set_trace()

    accum = load(open(pickle_accum_file, 'r'))
    timestamps = load(open(pickle_timestamps_file, 'r'))

    replaced_accum = replace_messages_by_id(frame_id, accum, new_message)

    replaced_accum_file = pickle_accum_file.replace(
        '.pickle', '_replaced.pickle')
    replaced_accum_timestamp_file = pickle_timestamps_file.replace(
        '_timestamps.pickle', '_replaced_timestamps.pickle')
    print("Writing to " + replaced_accum_file + " and " + replaced_accum_timestamp_file)
    dump(replaced_accum, open(replaced_accum_file, 'w'))
    dump(timestamps, open(replaced_accum_timestamp_file, 'w'))
