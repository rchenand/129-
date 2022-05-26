# Imports
from Motor import Motor

from Ultrasonic import Ultrasonic

from Intersection import *

import constants

import pigpio
import sys
import time
import copy

# Global Variables:
intersections = [] # List of intersections
lastintersection = None # Last intersection visited
long = 0 # Current east/west coordinate
lat = -1 # Current north/south coordinate

heading = constants.NORTH # Current heading

sides_status = []
check_sides = []
node_cntr = 1
unx_cntr = 0


acceptingCal = True
sees = False
turn_correct = False

class Robot: 
    def __init__(self):    
    	# Initialize the connection to the pigpio daemon (GPIO interface).
        self.io = pigpio.pi()
        if not self.io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)
        
        self.driving_stopflag = True
        self.keepdriving = False
        self.gotoint = False
        self.x = 0
        self.y = 0
        
    # line following to intersection
    def drivebehavior(self,motor):
      drivingConditions = True
      sharp_turn = 0.25
      turn_const = 0.5
      power_const = 0.8
      past_mid = 0
      past_right = 0
      past_left = 0
      acceptingCal = False
      while drivingConditions:
            right_val = self.io.read(constants.IR_R)
            left_val = self.io.read(constants.IR_L)
            mid_val = self.io.read(constants.IR_M)
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
      motor.set(0,0) # stops when gets to intersection

    # check streets at intersection
    def lookaround(self,motor):
        global acceptingCal
        global sees
        global node_cntr
        global turn_correct

        checks = []
        acceptingCal = True  
        # checks front
        checks.append(self.checkbehavior())
        
        for x in range(4):
            sees = False
            acceptingCal = True
            start_time = self.io.get_current_tick()
            motor.set(-0.8, 0.8)
            time.sleep(0.2)
            while sees == False: # and hasn't hit 100 deg
                waittime = 550000 if not turn_correct else 500000
                if (self.io.get_current_tick() - start_time) > waittime:
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

        
    def found(self,gpio, level, tick):
        global sees
        global acceptingCal
        if acceptingCal == True:
           sees = True
 
    def checkbehavior(self):
           right_val = self.io.read(constants.IR_R)
           left_val = self.io.read(constants.IR_L)
           mid_val = self.io.read(constants.IR_M)
           if mid_val == 1 or right_val == 1 or left_val == 1:
              return True
           else:
              return False
    # New longitude/latitude value after a step in the given heading.
    def shift(self,long, lat, heading):
        if heading % 4 == constants.NORTH:
            return (long, lat+1)
        elif heading % 4 == constants.WEST:
            return (long-1, lat)
        elif heading % 4 == constants.SOUTH:
            return (long, lat-1)
        elif heading % 4 == constants.EAST:
            return (long+1, lat)
        else:
            raise Exception("This can’t be")

    def turn(self,motor,mag):
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
            
      # mapping

      # Find the intersection
    def intersection(self,long, lat):
        list = [i for i in intersections if i.long == long and i.lat == lat]
        if len(list) == 0:
            return None
        if len(list) > 1:
            raise Exception("Multiple intersections at (%2d,%2d)" % (long, lat))
        return list[0]



    def mapping(self,motor):
        while self.keepdriving == True: #and self.driving_stopflag == False:
            global long, lat, heading, intersections, lastintersection, node_cntr, unx_cntr, side_status, check_sides
            self.io.callback(constants.IR_M, pigpio.RISING_EDGE, self.found)
            stop_condition = True
            while (stop_condition == True and self.keepdriving == True): # add the pause flag here? 
                unx_cntr = 0
                self.drivebehavior(motor)
                (curr_long, curr_lat) = self.shift(long,lat,heading)
                long = curr_long
                lat = curr_lat
                print(f"Intersection Number: {node_cntr}")
                node_cntr = node_cntr + 1
                # new intersection
                if self.intersection(long, lat) == None:
                    intersections.append(Intersection(long,lat))
            
                # get true or false in NWSE order [T, F, F, T]
                    check_sides = self.lookaround(motor)

                    # if check_sides True --> UNEXPLORED, if false --> NOSTREET
                    self.intersection(long,lat).streets = [constants.UNEXPLORED if k else constants.NOSTREET for k in check_sides]
                    print(f"streets: {self.intersection(long,lat).streets}")

                    # it came from this place, so it's connected
                else:
                    print("been here before")
                    print(f"streets: {self.intersection(long,lat).streets}")
                
                

                # update previous intersection as connected
                if lastintersection != None:
                    #print("adding back and last back as connected")
                                # sets current intersection back to connected
                    self.intersection(long,lat).streets[(heading+2) % 4] = constants.CONNECTED
                    self.intersection(lastintersection[0],lastintersection[1]).streets[heading] = constants.CONNECTED
                # connects opposing street if not first intersection
            # global lastintersection
                
                nint = self.intersection(long,lat)

                nint.headingToTarget = (heading+2) % 4


                if constants.UNEXPLORED in nint.streets:
                    print("there is an unexplored")
                    if nint.streets[heading] is constants.UNEXPLORED:
                        #print("going forward")
                        self.turn(motor, 0)
                    #heading = heading
                    elif nint.streets[(heading + 1) % 4] is constants.UNEXPLORED:
                        #print("left")
                        print(nint.streets[heading])
                        self.turn(motor,1)
                        heading = (heading + 1) % 4
                    elif nint.streets[(heading + 3) % 4] is constants.UNEXPLORED:
                        #print("right")
                        self.turn(motor,3)
                        heading = (heading + 3) % 4
                else:
                    print("there is a connected")
                    connected_streets = []
                    for y in range(len(nint.streets)):
                        if nint.streets[y] is constants.CONNECTED:
                            #print("CONNECTED")
                            if (y != (heading + 2) % 4):
                                #print("Not behind")
                                connected_streets.append(y)
                    choice2 = constants.random.choice(connected_streets)
                    #print(connected_streets)
                    #print("turning to a connected")
                    self.turn(motor,(choice2 - heading) % 4)
                        
                    heading = choice2
                for x in intersections:
                    if constants.UNEXPLORED in x.streets:
                        unx_cntr = unx_cntr + 1
                        
                
                # update after  first
                lastintersection = [long,lat]
                print(lastintersection)

                if unx_cntr ==0:
                    stop_condition = False

            print("EVERYTHING MAPPED") 
            # once exit loop, drive back to start
            motor.set(0,0)
        print("change of command")
        motor.set(0,0)

    # start dijkstras
    def dijkstras(self,motor):
        print("running dijkstras")
        global long, lat, heading, intersections, lastintersection, node_cntr, unx_cntr, check_sides
        while True:
            # store where we ended
            curr_intersection =(long, lat)

            time.sleep(1)


            # clear all headingtoTargets
            for i in intersections:
                i.headingToTarget = None
            
            int_cntr = 0
            goal = (self.x,self.y)
            print(goal)
            on_deck = [(self.x,self.y)] # FIFO list
            print(on_deck)
            processed = []

            int_coords = []
            
            for y in intersections:
                int_coords.append((y.long,y.lat))
            
            print("intersections",int_coords)

            while(curr_intersection not in processed):
                print("hello")
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
                    if self.intersection(curr_north[0],curr_north[1]).headingToTarget == None:
                        print('add')
                        self.intersection(curr_north[0],curr_north[1]).headingToTarget = 2
                        on_deck.append(curr_north)
                
                if (curr_west) in int_coords:
                    print("exists")
                    if self.intersection(curr_west[0],curr_west[1]).headingToTarget == None:
                        print('add')
                        self.intersection(curr_west[0],curr_west[1]).headingToTarget = 3
                        on_deck.append(curr_west)

                if (curr_south) in int_coords:
                    print("exists")
                    if self.intersection(curr_south[0],curr_south[1]).headingToTarget == None:
                        print('add')
                        self.intersection(curr_south[0],curr_south[1]).headingToTarget = 0
                        on_deck.append(curr_south)

                if (curr_east) in int_coords:
                    print("exists")
                    if self.intersection(curr_east[0],curr_east[1]).headingToTarget == None:
                        print('add')
                        self.intersection(curr_east[0],curr_east[1]).headingToTarget = 1
                        on_deck.append(curr_east)

            print("going shortest path")       
            while (long,lat)!= goal:
                cint = self.intersection(long,lat)
                self.turn(motor,(cint.headingToTarget - heading) % 4)
                self.drivebehavior(motor)
                heading = (cint.headingToTarget) % 4
                (long, lat) = self.shift(long,lat,heading) 
                print(long,lat)

            motor.set(0,0)

    # Driving thread: drive from intersection to intersection
    def driving_stop(self):
        self.driving_stopflag = True
    
    def driving_loop(self,motor):
        self.driving_stopflag = False
        while True: #not self.driving_stopflag:
        # Pause at this intersection, if requested
            if self.keepdriving:
                self.mapping(motor)
            
            elif self.gotoint: 
                self.dijkstras(motor,)

            else:
            # Move from this intersection to the next intersection
                motor.set(0,0)





    	


