# -*- coding: utf-8 -*-
"""
Created on Wed May 18 12:33:41 2022

@author: 18123
"""

# -*- coding: utf-8 -*-
"""
Created on Tue May  3 13:54:13 2022
@author: 18123
"""
# PROBLEM 7, Djistras

# Imports
import pigpio
import sys
import time
import copy

import random
# Motor class
from Motor import Motor

from Intersection import Intersection

# Define the motor pins.
MTR1_LEGA = 8
MTR1_LEGB = 7

MTR2_LEGA = 5
MTR2_LEGB = 6


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
unx_cntr = 0

io = pigpio.pi()
if __name__ == "__main__":
    motor = Motor()
    case = 0
    #print(io.read(IR_R))
    #print(io.read(IR_L))
    #print(io.read(IR_M))
    acceptingCal = True
    sees = False
    turn_correct = False

    def drivebehavior(motor):
        # making line following more robust
      drivingConditions = True
      sharp_turn = 0.25
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
                   time.sleep(0.4)

            # seeing an intersection. We check the past values and not the
            #current ones so that the bot has extra time to move forward
            elif past_left == 1 and past_right == 1 and past_mid == 1:
                drivingConditions = False
                motor.set(0.7,0.7)
                time.sleep(0.4)

           # elif past_left == 0 and past_right == 0 and past_mid =
            else:
                drivingConditions = True

            past_left = left_val
            past_right = right_val
            past_mid = mid_val
      motor.set(0,0)

    def lookaround(motor):
        global acceptingCal
        global sees
        global node_cntr
        global turn_correct

        checks = []
        acceptingCal = True  
        # checks front
        checks.append(checkbehavior(motor))
        
        for x in range(4):
            sees = False
            acceptingCal = True
            start_time = io.get_current_tick()
            motor.set(-0.8, 0.8)
            time.sleep(0.2)
            while sees == False: # and hasn't hit 100 deg
                waittime = 550000 if not turn_correct else 500000
                if (io.get_current_tick() - start_time) > waittime:
                    print('waited', waittime)
                    break #sees = True
                motor.set(-0.8, 0.8)
            
            time.sleep(0.07)
            motor.set(0,0)
            time.sleep(1)
            
            turn_correct = not sees
            checks.append(sees)
            #time.sleep(0.03)

        acceptingCal = False
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
            raise Exception("This can???t be")

    def turn(mag):
        global acceptingCal
        acceptingCal = True 
        global sees
        sees = False
        if mag == 1:
            motor.set(-0.8,0.8)
            time.sleep(0.1)
            while sees == False:
                motor.set(-0.8, 0.8)
                time.sleep(0.05)
        elif mag == 3:
            motor.set(0.8,-0.8)
            time.sleep(0.1)
            while sees == False:
                motor.set(0.8, -0.8)
                time.sleep(0.05)
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
    
    # djikstra's function
    def dijstras(long, lat):
            # store where we ended
            curr_intersection = (long, lat)

            time.sleep(1)
            

            # make a separte list of the intersecitons to work w
                       
            curr_lists_ints = copy.deepcopy(intersections)
            # clear all headingtoTargets
            for i in curr_lists_ints:
                i.headingToTarget = None
        
            int_cntr = 0
            x = int(input("input x"))
            y = int(input("input y"))
            goal = (x,y)
            print(goal)
            on_deck = [(x,y)] # FIFO list
            print(on_deck)
            processed = []

            int_coords = []
            
            for y in curr_list_ints:
                int_coords.append((y.long,y.lat))
            
            print("intersections",int_coords)

            while(curr_intersection not in processed):
                print('1')
                temp_target = on_deck[0]
                on_deck = on_deck[1:]
                processed.append(temp_target)
                
                curr_north = (temp_target[0],temp_target[1]+1)
                curr_west = (temp_target[0]-1,temp_target[1])
                curr_south = (temp_target[0],temp_target[1]-1)
                curr_east = (temp_target[0] + 1,temp_target[1])

                #for x in range(4):
                if (curr_north) in int_coords:
                    print("exists")
                    if intersection(curr_north[0],curr_north[1]).headingToTarget == None:
                        print('add')
                        intersection(curr_north[0],curr_north[1]).headingToTarget = 2
                        on_deck.append(curr_north)
                
                if (curr_west) in int_coords:
                    print("exists")
                    if intersection(curr_west[0],curr_west[1]).headingToTarget == None:
                        intersection(curr_west[0],curr_west[1]).headingToTarget = 3
                        on_deck.append(curr_west)

                if (curr_south) in int_coords:
                    print("exists")
                    if intersection(curr_south[0],curr_south[1]).headingToTarget == None:
                        intersection(curr_south[0],curr_south[1]).headingToTarget = 0
                        on_deck.append(curr_south)

                if (curr_east) in int_coords:
                    print("exists")
                    if intersection(curr_east[0],curr_east[1]).headingToTarget == None:
                        intersection(curr_east[0],curr_east[1]).headingToTarget = 1
                        on_deck.append(curr_east)

            print("going shortest path")       
            while (long,lat)!= goal:
                cint = intersection(long,lat)
                turn((cint.headingToTarget - heading) % 4)
                drivebehavior(motor)
                heading = (cint.headingToTarget) % 4
                (long, lat) = shift(long,lat,heading) 


    try:
        io.callback(IR_M, pigpio.RISING_EDGE, found)
        stop_condition = True
        while (stop_condition == True):
            unx_cntr = 0
            drivebehavior(motor)
            (curr_long, curr_lat) = shift(long,lat,heading)
            long = curr_long
            lat = curr_lat
            print(f"Intersection Number: {node_cntr}")
            node_cntr = node_cntr + 1
            # new intersection
        
            nint = intersection(long,lat)

            
            
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
                
                if UNEXPLORED not in nint.streets:
                    # run Dijstra's to drive to an unexplored street
                    dijstras(long,lat)
                    continue # move to next iteration in while loop 
                    
                print(f"streets: {intersection(long,lat).streets}")
            
            nint.headingToTarget = (heading+2) % 4

            # update previous intersection as connected
            if lastintersection != None:
                #print("adding back and last back as connected")
                            # sets current intersection back to connected
                intersection(long,lat).streets[(heading+2) % 4] = CONNECTED
                intersection(lastintersection[0],lastintersection[1]).streets[heading] = CONNECTED
            # connects opposing street if not first intersection
           # global lastintersection
            
            


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
                turn((choice2 - heading) % 4)
                    
                heading = choice2
            for x in intersections:
                if UNEXPLORED in x.streets:
                    unx_cntr = unx_cntr + 1
                    
            
            # update after  first
            lastintersection = [long,lat]
            print(lastintersection)

            if unx_cntr ==0:
                stop_condition = False

        print("EVERYTHING MAPPED") 
        
        # vicotry dance, spin in place
        motor.set(0.7,0)
        time.sleep(4)
          # once exit loop, drive back to start
          
        motor.set(0,0)
            
        print('done')
    except BaseException as ex:
        motor.set(0,0)
        print(ex)
        print('hello')

        

# Turn off the robot
# SET MOTOR TO ZERO.
