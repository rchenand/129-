# -*- coding: utf-8 -*-
"""
Created on Tue May  3 13:54:13 2022

@author: 18123
"""

# Imports
import pigpio
import sys
import time

import random
# Motor class
from Motor import Motor

from Intersection import Intersection

# Define the motor pins.
MTR1_LEGA = 7
MTR1_LEGB = 8

MTR2_LEGA = 6
MTR2_LEGB = 5


IR_R  = 14
IR_M =  15
IR_L = 18


# GLOBAL CONSTANTS
# Global Constants:
# Headings
NORTH = 0
WEST = 1
SOUTH = 2
EAST = 3
HEADING = {NORTH:'North', WEST:'West', SOUTH:'South',
EAST:'East', None:'None'} # For printing
# Street status
UNKNOWN = 'Unknown'
NOSTREET = 'NoStreet'
UNEXPLORED = 'Unexplored'
CONNECTED = 'Connected'
HELLO = 'Hello'
# Global Variables:
intersections = [] # List of intersections
lastintersection = None # Last intersection visited
long = 0 # Current east/west coordinate
lat = -1 # Current north/south coordinate

heading = NORTH # Current heading

sides_status = []
check_sides = []


io = pigpio.pi()
if __name__ == "__main__":
    motor = Motor()
    case = 0
    print(io.read(IR_R))
    print(io.read(IR_L))
    print(io.read(IR_M))
    acceptingCal = True
    sees = False

    def drivebehavior(motor):
      drivingConditions = True
      sharp_turn = 0.3
      turn_const = 0.5
      power_const = 0.8
      past_mid = 0
      past_right = 0
      past_left = 0
      acceptingCal = False
      while drivingConditions:
            right_val = io.read(IR_R)
            left_val = io.read(IR_L)
            mid_val = io.read(IR_M)
            # Take appropriate action
            print(right_val)
            print(left_val)
            print(mid_val)
            # veering left
            if mid_val == 1 and right_val == 0 and left_val == 0:
                motor.set(power_const, power_const)

            # sligth left
            elif left_val == 0 and right_val ==1 and mid_val == 1:
                motor.set(power_const, power_const*turn_const)

            # very left
            elif left_val == 0 and mid_val == 0 and right_val == 1:
                motor.set(power_const, power_const*sharp_turn)

            # slight right
            elif left_val == 1 and mid_val ==1 and right_val == 0:
                motor.set(power_const*turn_const, power_const)

            # very right
            elif left_val == 1 and mid_val == 0 and right_val == 0:
                motor.set(power_const*sharp_turn, power_const)

            # off line
            elif left_val == 0 and mid_val == 0 and right_val == 0:
                # off line right
                if past_left == 1:
                   motor.set(power_const * sharp_turn, power_const)

                # off line left
                elif past_right == 1:
                   motor.set(power_const, power_const * sharp_turn)

                # off line center
                elif past_mid == 1:
                   motor.set(power_const, -1 * power_const)
                else:
                   drivingConditions = False
                   motor.set(0.7, 0.7)
                   time.sleep(0.15)

            # seeing an intersection. We check the past values and not the
            #current ones so that the bot has extra time to move forward
            elif past_left == 1 and past_right == 1 and past_mid == 1:
                drivingConditions = False
                motor.set(0.7,0.7)
                time.sleep(0.15)

           # elif past_left == 0 and past_right == 0 and past_mid =
            else:
                drivingConditions = True

            past_left = left_val
            past_right = right_val
            past_mid = mid_val
      motor.set(0,0)

    def lookaround(motor):
        global heading
        checks = [True, True, True, True]
        global acceptingCal
        acceptingCal = True  
        # checks front
        #checks.append(checkbehavior(motor))
        checks[heading] = checkbehavior(motor)
        change_heading = heading
        global sees
        for x in range(4):
           sees = False
           acceptingCal = True
           print(acceptingCal)
           start_time = io.get_current_tick()
           change_heading = (change_heading + 1) % 4
           if x != 3:
                checks[change_heading] = True
           while sees == False: # and hasn't hit 100 deg
                if (io.get_current_tick() - start_time) > 800000:
                    print(io.get_current_tick() - start_time)

                    print(sees)
                    checks[change_heading] = False
                    sees = True
                motor.set(-0.6, 0.6)
                time.sleep(0.1)
        # in the correct order
        print(f"checks: {checks}")
        return checks
        
        

    def found(gpio, level, tick):
        global sees
        global acceptingCal
        if acceptingCal == True:
           sees = True
 
    def checkbehavior(motor):
           right_val = io.read(IR_R)
           left_val = io.read(IR_L)
           mid_val = io.read(IR_M)
           if mid_val == 1 or right_val == 1 or left_val == 1:
              return True
           else:
              return False
    # New longitude/latitude value after a step in the given heading.
    def shift(long, lat, heading):
        if heading % 4 == NORTH:
            return (long, lat+1)
        elif heading % 4 == WEST:
            return (long-1, lat)
        elif heading % 4 == SOUTH:
            return (long, lat-1)
        elif heading % 4 == EAST:
            return (long+1, lat)
        else:
            raise Exception("This can’t be")
    def turn(mag):
        #90 degree turn left
        if mag == 1:
           motor.set(-0.6, 0.6)
           time.sleep(0.75)
        #270 degree turn left
        elif mag == -3:
           motor.set(0,0.7)
           time.sleep(2.5)
        #180 degree turn left
        elif mag == 2:
           motor.set(-0.7, 0.7)
           time.sleep(1)
        #180 turn right
        elif mag == -2:
           motor.set(0.7, -0.7)
           time.sleep(1)
        #90 degree turn right
        elif mag == 3:
           motor.set(0.6, -0.6)
           time.sleep(0.75)
        #270 degree turn right
        elif mag == -1:
           motor.set(0.7, 0)
           time.sleep(2.5)
        #Robot does not turn
        else:
           motor.set(0,0)
    
# Find the intersection
    def intersection(long, lat):
        list = [i for i in intersections if i.long == long and i.lat == lat]
        if len(list) == 0:
            return None
        if len(list) > 1:
            raise Exception("Multiple intersections at (%2d,%2d)" % (long, lat))
        return list[0]
    

        


    try:
        io.callback(IR_M, pigpio.RISING_EDGE, found)
        for x in range(4):
            drivebehavior(motor)
            (curr_long, curr_lat) = shift(long,lat,heading)
            long = curr_long
            lat = curr_lat
            
            if intersection(long, lat) == None:
                intersections.append(Intersection(long,lat))
           
            # get true or false in NWSE order
            check_sides = lookaround(motor)

            for i in range(4):
                if check_sides[i] == True:
                  # IF NONE, then at start
                  intersections[x].streets.insert(i, UNEXPLORED)
                else:
                  intersections[x].streets.insert(i, NOSTREET)
                print()
            
            # connects opposing street if not first intersection
           # global lastintersection
            if lastintersection != None:
                  intersections[x].streets.insert((heading+2)% 4,CONNECTED)

            intersections[x].headingToTarget = (heading+2) % 4
            choice = HELLO
            index = 0
            
            while choice != UNEXPLORED:
              index = random.choice(range(len(intersections[x].streets)))
              choice = intersections[x].streets[index]
            print('hello')
            turn((heading-index) % 4) # does it just keep turning? --> no bc we sleep
          
            # update after  first
            lastintersection = 1
          # once exit loop, drive back to start
        print('done')
    except ValueError as e:
        motor.set(0,0)
        print(e)
        print('hello')

        

# Turn off the robot
# SET MOTOR TO ZERO.
