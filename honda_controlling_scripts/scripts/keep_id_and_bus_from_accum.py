#!/usr/bin/env python

from accum_msgs_tools import filter_accum_keep, filter_messages_keep, to_sendeable_accum
from accum_msgs_tools import filter_accum_remove, filter_messages_remove, to_sendeable_block
from cPickle import load, loads, dump, dumps
import sys

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: ")
        print(sys.argv[0] + " accum.pickle id1 bus")
        exit(0)
    pickle_accum_file = sys.argv[1]
    pickle_timestamps_file = pickle_accum_file.replace(
        '.pickle', '_timestamps.pickle')
    print("Cleaning up files: " + pickle_accum_file +
          " and " + pickle_timestamps_file)
    ids_to_remove = [int(sys.argv[2])]
    bus = sys.argv[3]

    print("Going to keep ids: " + str(ids_to_remove))

    accum = load(open(pickle_accum_file, 'r'))
    timestamps = load(open(pickle_timestamps_file, 'r'))
    filtered_accum, filtered_timestamps = filter_accum_keep(
        ids_to_remove, accum, timestamps)

    filtered_fname = pickle_accum_file.replace('.pickle', '_filtered.pickle')
    filtered_t_fname = pickle_timestamps_file.replace(
        '_timestamps.pickle', '_filtered_timestamps.pickle')
    print("Writing in: " + filtered_fname + " and " + filtered_t_fname)
    dump(filtered_accum, open(filtered_fname, 'w'))
    dump(filtered_timestamps, open(filtered_t_fname, 'w'))
