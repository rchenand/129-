#!/usr/bin/env python3
#
#   motordemo.py
#
#   This shows how to interface with the GPIO (general purpose I/O)
#   pins and how to drive the PWM for the motors.  Please use as an
#   example, but change to suit the weekly goals.
#

# Imports
import pigpio
import sys
import time

from Ultrasonic import Ultrasonic
from Motor import Motor




# set up pins
ULTRA1_TRIGGER = 13
ULTRA1_ECHO = 16

ULTRA2_TRIGGER = 19
ULTRA2_ECHO = 20

ULTRA3_TRIGGER = 26
ULTRA3_ECHO = 21


if __name__ == "__main__":

    io = pigpio.pi()

    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)
    ultra1 = Ultrasonic(ULTRA1_TRIGGER, ULTRA1_ECHO, io)
    ultra2 = Ultrasonic(ULTRA2_TRIGGER, ULTRA2_ECHO, io)
    ultra3 = Ultrasonic(ULTRA3_TRIGGER, ULTRA3_ECHO, io)

    try:
        distances = [0,0,0]
        while True:
            #print("1:", ultra1.trigger())
            #print("2:", ultra2.trigger())
            print("3:", ultra3.trigger())
            time.sleep(0.1)
        ultra1.cb.cancel() 
        
    except BaseException as ex:
        print(ex)
        print('hello')
