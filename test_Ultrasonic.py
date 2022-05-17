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


distance = 0
rising_time = 0


#
#   Main
#


if __name__ == "__main__":

    io = pigpio.pi()


    ultra1 = Ultrasonic(ULTRA1_TRIGGER, ULTRA1_ECHO, io)
    
    

    def test_ultra(ultra1):
        ultra1.trigger()

        time.sleep(0.1)
        



    try:
        while True:
            test_ultra(ultra1)

        #cbrise1.cancel() 
        #Cbfalling1.cancel()


    except BaseException as ex:
        print(ex)
        print('hello')
