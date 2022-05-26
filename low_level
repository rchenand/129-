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
import random
import threading
import pickle

from Ultrasonic import Ultrasonic
from Motor import Motor


from Robot import Robot



# set up pins
ULTRA1_TRIGGER = 13
ULTRA1_ECHO = 16

ULTRA2_TRIGGER = 19
ULTRA2_ECHO = 20

ULTRA3_TRIGGER = 26
ULTRA3_ECHO = 21
IR_R  = 14
IR_M =  15
IR_L = 18

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

    motor = Motor()
    robot = Robot()
    
    # read IR values
    right_val = io.read(IR_R)
    left_val = io.read(IR_L)
    mid_val = io.read(IR_M)
    

    def userinput():
        print('user')
        while True:
        # Grab a command
        
            command = input("Command ? ")
        
        # Compare against possible commands.
        # start exploring
            if (command == 'explore'):
                #set flags so the robot will choose the next intersection
                # to explore the full map.
                print("Exploring without a target")
                robot.keepdriving = True

            # drive to target
            elif (command[:4] == 'goto'):
                #set flags so the robot drive to the given location.3
                # use our dijkstras
                
                print("Driving to a target")
                robot.gotoint = True
                robot.x = int(command[5:6])
                robot.y = int(command[7:8])

            # pause at next intersection
            elif (command == 'pause'):
                print("Pausing at the next intersection")
                robot.keepdriving = False
                robot.driving_stopflag = False
                print(robot.keepdriving)
                motor.set(0,0)

            # save map
            elif (command[:4] == 'save'):
                print("Saving current map into file")
                with open(command[5:]+'.pickle', 'wb') as file:
                    pickle.dump(robot.intersections, file)

            
            # save map
            elif (command[:4] == 'load'):
                print("Loading the map...")
                with open(command[5:]+'.pickle', 'rb') as file:
                    robot.intersections = pickle.load(file)

            # printing curr intersection
            elif (command == 'location'):
                print("Currently at", (robot.long, robot.lat))

            # stop mapping
            elif (command == 'quit'):
                print("Quitting...")
                break
            else:
                print("Unknown command ’%s’" % command)

    # ADD ultrasonic threads too

    driving_thread = threading.Thread(target=robot.driving_loop,args=(motor,))
    driving_thread.start()

    try:
        userinput()

            # run/stop continual triggering?

    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        motor.set(0,0)

    


    robot.driving_stop()
    driving_thread.join()

    #shutdown stuff

            
    
