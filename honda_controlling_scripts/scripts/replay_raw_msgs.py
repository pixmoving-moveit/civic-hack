#!/usr/bin/env python

from panda import Panda
from cPickle import dump, dumps, load, loads

if __name__ == '__main__':
    p = Panda()

    accum = load(open('accum.pickle', 'r'))
    print("publishing...")

    CAMERA_CAN = 1
    OTHER_CAN = 0

    # try:
    #     for block_msg in accum:
    #         for msg in block_msg:
    #             if msg:
    #                 try:
    #                     p.can_send(msg[0], str(msg[2]), CAMERA_CAN)
    #                 except TypeError as e:
    #                     print("error: " + str(e))
    #                     print(msg)
    #                     break
    # except KeyboardInterrupt:
    #     p.close()



    try:
        for block_msg in accum:
            for msg in block_msg:
                if msg:
                    try:
                        p.can_send(msg[0], str(msg[2]), 0)
                    except TypeError as e:
                        print("error: " + str(e))
                        print(msg)
                        break
    except KeyboardInterrupt:
        p.close()


    print("Done")
