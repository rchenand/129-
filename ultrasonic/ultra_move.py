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
import threading

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

    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)
    
    ultra1 = Ultrasonic(ULTRA1_TRIGGER, ULTRA1_ECHO, io)
    ultra2 = Ultrasonic(ULTRA2_TRIGGER, ULTRA2_ECHO, io)
    ultra3 = Ultrasonic(ULTRA3_TRIGGER, ULTRA3_ECHO, io)

    motor = Motor()

    thread = threading.Thread(target=ultra2.runcontinual)
    thread.start()



    try:
        while True:
            front_dist = ultra2.distance
            if front_dist < 20:
                motor.set(0,0)
            


    except ValueError as ex:
        print("killed")
        motor.set(0,0)
            

    # Set the stop flag and wait for the thread to stop running,
    # then rejoin the two threads
    ultra1.stopcontinual()
    thread.join()
    
    # Print to confirm there are no extraneous threads still running
    print("Active threads (excluding main thread):", threading.active_count()-1)
    
