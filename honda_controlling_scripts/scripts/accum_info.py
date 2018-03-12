#!/usr/bin/env python

from cPickle import load
import sys

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage:")
        print(sys.argv[0] + ' accum.pickle')
        print("(timestamps will be taken automatically)")
        exit(0)
    fname = sys.argv[1]
    fname = fname.replace('.pickle', '')
    # print("fname: " + fname)
    accum_fname = fname + ".pickle"
    # print("accum_fname: " + str(accum_fname))
    timestamps_fname = fname + "_timestamps.pickle"
    accum = load(open(accum_fname, 'r'))
    timestamps = load(open(timestamps_fname, 'r'))
    # print("Loaded files...")

    # Count blocks and messages
    block_amount = len(accum)
    messages_amount = 0
    msg_ids = {}
    buses = {}
    empty_msgs = 0
    for block in accum:
        block_l = len(block)
        if not block_l:
            empty_msgs += 1
        messages_amount += block_l
        for msg in block:
            frame_id, _, data, bus = msg
            if frame_id in msg_ids:
                msg_ids[frame_id] += 1
            else:
                msg_ids[frame_id] = 1

            if 'bus ' + str(bus) in buses:
                buses['bus ' + str(bus)] += 1
            else:
                buses['bus ' + str(bus)] = 1

    amount_ids = len(msg_ids.keys())

    ini_t = timestamps[0]
    fin_t = timestamps[-1]
    total_t = fin_t - ini_t

    print("---------------")
    print(fname + " has " + str(block_amount) + " blocks.")
    print("    With " + str(messages_amount) + " messages in them.")
    print("    Including " + str(empty_msgs) + " empty messages.")
    print("    Containing " + str(amount_ids) + " different message IDs.")
    print("    With a duration of " + str(total_t) + " seconds.")
    print("    Taken from buses: " + str(buses))
    print("    With the amounts: ")
    print(msg_ids)
