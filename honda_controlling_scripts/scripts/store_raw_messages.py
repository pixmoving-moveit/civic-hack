#!/usr/bin/env python

from panda import Panda
from cPickle import dump, dumps, load, loads
import time
import sys


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage:")
        print(sys.argv[0] + " name_for_pickle_file")
        exit(0)
    filename = sys.argv[1]
    p = Panda()

    accum = []
    timestamps = []
    try:
        while True:
            accum.append(p.can_recv())
            timestamps.append(time.time())
            if len(accum) % 10000 == 0:
                print("Stored " + str(len(accum)) + " blocks of messages...")
    except KeyboardInterrupt:
        p.close()
        dump(accum, open(filename + '.pickle', 'w'))
        dump(timestamps, open(filename + '_timestamps.pickle', 'w'))

    print(filename + ".pickle contains " +
          str(len(accum)) + " blocks of CAN messages.")
