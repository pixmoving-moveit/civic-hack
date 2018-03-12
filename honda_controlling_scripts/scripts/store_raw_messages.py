#!/usr/bin/env python

from panda import Panda
from cPickle import dump
import time
import sys


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Store raw messages from panda board can_recv into a pickle file.")
        print("Usage:")
        print(sys.argv[0] + " prefix_name")
        exit(0)
    filename = sys.argv[1]
    if filename.endswith('.pickle'):
        filename = filename[:-7]
    p = Panda()

    accum = []
    timestamps = []
    try:
        while True:
            block = p.can_recv()
            if block:
                accum.append(block)
                timestamps.append(time.time())
            if len(accum) % 10000 == 0:
                print("Stored " + str(len(accum)) + " blocks of messages...")
    except KeyboardInterrupt:
        p.close()
        dump(accum, open(filename + '.pickle', 'w'))
        dump(timestamps, open(filename + '_timestamps.pickle', 'w'))
        print("\n\n")
        print(filename + ".pickle contains " +
              str(len(accum)) + " blocks of CAN messages.")
