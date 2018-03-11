#!/usr/bin/env python

from panda import Panda

"""
Send braking commands on keypress.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""

if __name__ == '__main__':
    print("Connecting to panda...")
    panda = Panda()
    print("Setting safety mode SAFETY_HONDA")
    panda.set_safety_mode(panda.SAFETY_HONDA)
    # panda.set_safety_mode(panda.SAFETY_ALLOUTPUT)

    print("Press 'b' to send braking commands")
    


