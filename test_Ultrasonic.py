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


    #ultra1 = Ultrasonic(ULTRA1_TRIGGER, ULTRA1_ECHO, io)
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)


    print("Setting up the GPIO...")
    # Set up the four pins as output (commanding the motors).
    io.set_mode(ULTRA1_TRIGGER, pigpio.OUTPUT)
    io.set_mode(ULTRA1_ECHO, pigpio.INPUT)
        
    def rising(gpio, level, tick):
        global rising_time
        rising_time = tick
        #print(rising_time)

    def falling(gpio, level, tick):

            # divider 10000
        distance = ((343/2) * (tick- rising_time)) / 10000
        print(distance)
    
    def either(gpio, level, tick):
        if level == 1:
            rising(gpio, level, tick)
        elif level == 0:
            falling(gpio,level,tick)
        
        else:
            print("error")

    # *OR* set up one handler for both.
    cb = io.callback(ULTRA1_ECHO, pigpio.EITHER_EDGE, either)


        
    def trigger():
            # Pull one (or all) trigger pins HIGH
        io.write(ULTRA1_TRIGGER, 1)
        # Hold for 10microseconds.
        time.sleep(0.000010)
            # Pull the pins LOW again.
        io.write(ULTRA1_TRIGGER, 0)
        
    try:
        while True:
            trigger()
            time.sleep(0.1)
            

        #cbrise1.cancel() 
        #Cbfalling1.cancel()


    except BaseException as ex:
        print(ex)
        print('hello')
