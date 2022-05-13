# -*- coding: utf-8 -*-
"""
Created on Tue May  3 13:54:13 2022

@author: 18123
"""

# code as of 5/12/22

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
node_cntr = 1

io = pigpio.pi()
if __name__ == "__main__":
    motor = Motor()
    case = 0
    #print(io.read(IR_R))
    #print(io.read(IR_L))
    #print(io.read(IR_M))
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
            #print(right_val)
            #print(left_val)
            #print(mid_val)
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
                   time.sleep(0.5)

            # seeing an intersection. We check the past values and not the
            #current ones so that the bot has extra time to move forward
            elif past_left == 1 and past_right == 1 and past_mid == 1:
                drivingConditions = False
                motor.set(0.7,0.7)
                time.sleep(0.5)

           # elif past_left == 0 and past_right == 0 and past_mid =
            else:
                drivingConditions = True

            past_left = left_val
            past_right = right_val
            past_mid = mid_val
      motor.set(0,0)

    def lookaround(motor):
        checks = []
        global acceptingCal
        acceptingCal = True  
        # checks front
        checks.append(checkbehavior(motor))
        global sees
        global node_cntr
        global node_cntr
        for x in range(4):
           motor.set(-0.8,0.8)
           time.sleep(0.1)
           sees = False
           acceptingCal = True
           start_time = io.get_current_tick()
           checks.append(True)
           while sees == False: # and hasn't hit 100 deg
                if (io.get_current_tick() - start_time) > 500000:
                    #print('100 deg!')
                    checks[x+1] = False
                    break #sees = True
                motor.set(-0.8, 0.8)
                #print(f"slept for turn{x}.")
                time.sleep(0.05)
           time.sleep(0.03)
           sees = False
           motor.set(0,0)
           time.sleep(1)
        checks = checks[0:4]
        list_in_nwse_order = checks[-heading:] + checks[:-heading]
        print(f"list: {list_in_nwse_order}")
        return list_in_nwse_order

        
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
        global acceptingCal
        acceptingCal = True 
        global sees
        sees = False
        if mag == 1:
            while sees == False:
                motor.set(-0.8, 0.8)
                time.sleep(0.1)
        elif mag == 3:
            while sees == False:
                motor.set(0.8, -0.8)
                time.sleep(0.1)
        elif mag == 0:
            motor.set(0,0)

        # turn 180 deg
        elif mag == 2:
           motor.set(-0.7, 0.7)
           time.sleep(1)
        else:
            print("magnitude is not in 0,1 or 3")
            print(f"Mag is {mag}")

            
    
# Find the intersection
    def intersection(long, lat):
        list = [i for i in intersections if i.long == long and i.lat == lat]
        if len(list) == 0:
            return None
        if len(list) > 1:
            raise Exception("Multiple intersections at (%2d,%2d)" % (long, lat))
        return list[0]
    
     


    try:
        io.callback(IR_L, pigpio.RISING_EDGE, found)
        stop_condition = True
        while (stop_condition == True):
            drivebehavior(motor)
            (curr_long, curr_lat) = shift(long,lat,heading)
            long = curr_long
            lat = curr_lat
            print(f"Intersection Number: {node_cntr}")
            node_cntr = node_cntr + 1
            # new intersection
            if intersection(long, lat) == None:
                intersections.append(Intersection(long,lat))
           
            # get true or false in NWSE order [T, F, F, T]
                check_sides = lookaround(motor)

                # if check_sides True --> UNEXPLORED, if false --> NOSTREET
                intersection(long,lat).streets = [UNEXPLORED if k else NOSTREET for k in check_sides]
                print(f"streets: {intersection(long,lat).streets}")

                # it came from this place, so it's connected
            else:
                print("been here before")
                print(f"streets: {intersection(long,lat).streets}")
            
            
            


            # sets current intersection back to connected
            intersection(long,lat).streets[(heading+2) % 4] = CONNECTED

            # update previous intersection as connected
            if lastintersection != None:
                print("adding back as connected")
                intersection(lastintersection[0],lastintersection[1]).streets[heading] = CONNECTED
            # connects opposing street if not first intersection
           # global lastintersection
            
            nint = intersection(long,lat)

            nint.headingToTarget = (heading+2) % 4


            if UNEXPLORED in nint.streets:
                print("there is an unexplored")
                if nint.streets[heading] is UNEXPLORED:
                    print("going forward")
                    turn(0)
                   #heading = heading
                elif nint.streets[(heading + 1) % 4] is UNEXPLORED:
                    print("left")
                    print(nint.streets[heading])
                    turn(1)
                    heading = (heading + 1) % 4
                elif nint.streets[(heading + 3) % 4] is UNEXPLORED:
                    print("right")
                    turn(3)
                    heading = (heading + 3) % 4
            else:
                print("there is a connected")
                connected_streets = []
                for y in range(len(nint.streets)):
                    if nint.streets[y] is CONNECTED:
                        #print("CONNECTED")
                        if (y != (heading + 2) % 4):
                            #print("Not behind")
                            connected_streets.append(y)
                choice2 = random.choice(connected_streets)
                #print(connected_streets)
                #print("turning to a connected")
                turn(random.choice(connected_streets))
                    
                heading = choice2
            
            # update after  first
            lastintersection = [long,lat]
            print(lastintersection)
          # once exit loop, drive back to start
        motor.set(0,0)
        time.sleep(1)
        for y in range(4):
            cint = intersection(long,lat)
            turn((cint.headingToTarget - heading) % 4)
            print(cint.headingToTarget)
            drivebehavior(motor)
            heading = (cint.headingToTarget) % 4
            (long, lat) = shift(long,lat,heading)
            
            
        print('done')
    except ValueError as e:
        motor.set(0,0)
        print(e)
        print('hello')

        

# Turn off the robot
# SET MOTOR TO ZERO.
