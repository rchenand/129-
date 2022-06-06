'''Used for mapping, dijkstras, etc. Driving thread'''

# Imports
from Motor import Motor
from Ultrasonic import Ultrasonic
from Intersection import *
import constants
import random

import pigpio
import sys
import time


# global variables
heading = constants.NORTH 
sides_status = []
check_sides = []
node_cntr = 1
unx_cntr = 0

# Booleans 
acceptingCal = True
sees = False
turn_correct = False

turned_around = False #if intersection is partway blocked
in_tunnel = True #if not in tunnel / has line to follow
tunnel_intersection = False # dont do lookaround if it's in a narrow tunnel




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

        self.obstacle1_detected = False
        self.obstacle2_detected = False
        self.obstacle3_detected = False
        self.x = 0
        self.y = 0

        self.ultra1 = Ultrasonic(constants.ULTRA1_TRIGGER, constants.ULTRA1_ECHO, self.io)
        self.ultra2 = Ultrasonic(constants.ULTRA2_TRIGGER, constants.ULTRA2_ECHO, self.io)
        self.ultra3 = Ultrasonic(constants.ULTRA3_TRIGGER, constants.ULTRA3_ECHO, self.io)

        # other attributes for mapping
        self.intersections = [] # List of intersections
        self.lastintersection = None # Last intersection visited
        self.long = 0 # Current east/west coordinate
        self.lat = -1 # Current north/south coordinate
      

    # line following to intersection
    def drivebehavior(self,motor):
      drivingConditions = True
      sharp_turn = 0.25
      turn_const = 0.5
      power_const = 0.8
      past_mid = 0
      past_right = 0
      past_left = 0

      global in_tunnel, turned_around, tunnel_intersection

      while drivingConditions:
        
        right_val = self.io.read(constants.IR_R)
        left_val = self.io.read(constants.IR_L)
        mid_val = self.io.read(constants.IR_M)

        #not in a tunnel --> normal line following
        if(not in_tunnel):  
            
            # veering left
            if mid_val == 1 and right_val == 0 and left_val == 0:
                motor.set(power_const, power_const)
            # slight left
            elif left_val == 0 and right_val ==1 and mid_val == 1:
                motor.set(power_const, power_const*turn_const)
            # very left
            elif left_val == 0 and mid_val == 0 and right_val == 1:
                motor.set(power_const, power_const*sharp_turn)
            # slight right
            elif left_val == 1 and mid_val == 1 and right_val == 0:
                motor.set(power_const*turn_const, power_const)
            # very right
            elif left_val == 1 and mid_val == 0 and right_val == 0:
                motor.set(power_const*sharp_turn, power_const)
            # off line
            elif left_val == 0 and mid_val == 0 and right_val == 0:

                self.ultra_detects() # may not be necessary
                print("left ultra:", self.ultra1.distance)
                print("right ultra:", self.ultra3.distance)
                if(self.obstacle1_detected or self.obstacle3_detected):
                    in_tunnel = True
                else:                
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


            # MAYBE CHECK ULTRASONIC READINGS HERE B4 DRIVING FORWARD
            #
            elif past_left == 1 and past_right == 1 and past_mid == 1:
                drivingConditions = False
                self.ultra_detects()
                #driving foreward before checking intersection
                motor.set(0.7,0.7)
                time.sleep(0.4)

            else:
                drivingConditions = True

            past_left = left_val
            past_right = right_val
            past_mid = mid_val


        # in tunnel
        else:
            #checks to see if it found line again / is past tunnel
            if left_val == 1 or mid_val == 1 or right_val == 1:
                #intersection inside tunnel
                if left_val == 1 and mid_val == 1 and right_val == 1:
                    print("set tunnel intersection bool to true")
                    tunnel_intersection = True
                    drivingConditions = False
                else: 
                    tunnel_intersection = False

                #end of the tunnel
                self.ultra_detects()
                if not self.obstacle1_detected or not self.obstacle3_detected:
                    in_tunnel = False
                    print("out of tunnel")
            else:
                # paste in code from that wall hugging
                # may need to adjust distances so they're smaller (whatever width of tunnel is / 2)

                # ORRR make it so the bot fights to make L/R distnaces equal

                # ORRRRRRR just make one of the sensors hug width / 2
                # only if width is consistent ???

                # assume wall on left
                left_dist = self.ultra1.distance

                e = left_dist - 10.7
                u = 0.1 * e
                #print("u:", u)
                #print("dist", self.ultra1.distance)
                

                PWM_left = max(0.5, min(0.9, 0.7 - u))
                PWM_right = max(0.5, min(0.9, 0.7 + u))

                motor.set(PWM_left, PWM_right)



        # if it runs into an OBSTACLE while ALREADY on street
        # turns aroud
        self.ultra_detects()
        if self.obstacle2_detected:
            self.turn(motor, 2) # can also change to turn until sees line (what if it's in a tunnel tho????)
            turned_around = True


      motor.set(0,0) # stops when gets to intersection

       


    # check streets at intersection
    def lookaround(self,motor):
        global acceptingCal, sees, turn_correct
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

            # exits loop if IR sensor gets reading --> callback: sees --> T
            while sees == False: 
                # turn 100 deg if line not found, turn less if two lines not 
                #                                       found twice
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

        acceptingCal = False
        checks = checks[0:4]
        list_in_nwse_order = checks[-heading:] + checks[:-heading]
        print(f"list: {list_in_nwse_order}")
        return list_in_nwse_order

    # callback function, sets boolean sees
    def found(self,gpio, level, tick):
        global sees, acceptingCal
        if acceptingCal == True:
           sees = True

    # check IR sensor readings
    def checkbehavior(self):
           right_val = self.io.read(constants.IR_R)
           left_val = self.io.read(constants.IR_L)
           mid_val = self.io.read(constants.IR_M)
           if mid_val == 1 or right_val == 1 or left_val == 1:
              return True
           else:
              return False

    #check obstacles
    def ultra_detects(self):
        # when checking sides, first record values of flags before checking 

        if self.ultra2.distance < 20:
            self.obstacle2_detected = True
        else: 
            self.obstacle2_detected = False
        

        if self.ultra1.distance < 20:
            self.obstacle1_detected = True
        else: 
            self.obstacle1_detected = False
        

        if self.ultra3.distance < 20:
            self.obstacle3_detected = True
        else: 
            self.obstacle3_detected = False

            #if both 1 and 3 are true, start wall following
        #print(self.ultra2.distance)
        #print(self.obstacle2_detected)


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

    # turn left, right, back
    def turn(self,motor,mag):
        global acceptingCal
        acceptingCal = True 
        global sees
        sees = False
        if mag == 1: # turn left
            motor.set(-0.8,0.8)
            time.sleep(0.1)
            while sees == False:
                motor.set(-0.8, 0.8)
                time.sleep(0.05)
        elif mag == 3: # turn right
            motor.set(0.8,-0.8)
            time.sleep(0.1)
            while sees == False:
                motor.set(0.8, -0.8)
                time.sleep(0.05)
        elif mag == 0: # don't turn
            motor.set(0,0)

        elif mag == 2: # turn back, 180 deg
           motor.set(-0.8, 0.8)
           time.sleep(1.1)
        else:
            print("magnitude is not in 0,1 or 3")
            print(f"Mag is {mag}")
            
      # Find the intersection
    def intersection(self,long, lat):
        list = [i for i in self.intersections if i.long == long and i.lat == lat]
        if len(list) == 0:
            return None
        if len(list) > 1:
            raise Exception("Multiple intersections at (%2d,%2d)" % (long, lat))
        return list[0]

    # map a grid
    def mapping(self,motor):

        # CHECK FLAGS HERE
        while self.keepdriving == True: # keep running while exploring, stop if paused
            global long, lat, heading, node_cntr, unx_cntr, check_sides, turned_around
            self.io.callback(constants.IR_M, pigpio.RISING_EDGE, self.found)
            stop_condition = True

            while (stop_condition and self.keepdriving and not self.obstacle2_detected):


                #print("ultra1 reading", self.obstacle1_detected) 
                unx_cntr = 0
                self.drivebehavior(motor)

                #leaves drive behavior bc it saw intersection

                # if its its first intesection, just go for it
                # if not, tell it that it's the same one as it just saw
                if(not turned_around or node_cntr == 0):
                    (curr_long, curr_lat) = self.shift(long,lat,heading)
                    long = curr_long
                    lat = curr_lat
                    print(f"Intersection Number: {node_cntr}")
                    node_cntr = node_cntr + 1
                    # new intersection
                    if self.intersection(long, lat) == None:
                        self.intersections.append(Intersection(long,lat))
                
                        # get true or false in NWSE order 

                        check_sides = self.lookaround(motor)

                        #check ultrasonics for close blockades
                        self.ultra_detects()


                        # if check_sides True --> UNEXPLORED, if false --> NOSTREET
                        # included obstacle detected

                        self.intersection(long,lat).streets = [constants.UNEXPLORED if k else constants.NOSTREET for k in check_sides]

                        if(self.obstacle1_detected): # on the left; heading -1
                            dir = (heading - 1) % 4
                            self.intersection(long, lat).streets[dir] = BLOCKED
                    
                        if(self.obstacle2_detected): # in front
                            dir = heading
                            self.intersection(long, lat).streets[dir] = BLOCKED

                        if(self.obstacle3_detected): # on the right
                            dir = (heading + 1) % 4
                            self.intersection(long, lat).streets[dir] = BLOCKED


                        print(f"streets: {self.intersection(long,lat).streets}")

                    # it came from this place, so it's connected
                    else:
                        print("been here before")
                        print(f"streets: {self.intersection(long,lat).streets}")
                    
                    # update previous intersection as connected
                    if self.lastintersection != None:
                        #print("adding back and last back as connected")
                        # sets current intersection back to connected
                        self.intersection(long,lat).streets[(heading+2) % 4] = constants.CONNECTED
                        self.intersection(self.lastintersection[0],self.lastintersection[1]).streets[heading] = constants.CONNECTED
                    
                    # connects opposing street if not first intersection
                    nint = self.intersection(long,lat)
                    nint.headingToTarget = (heading+2) % 4

                    # MAKE SURE HANDLING OBSTACLE DETECTIONS
                    if constants.UNEXPLORED in nint.streets:
                        print("there is an unexplored")
                        if nint.streets[heading] is constants.UNEXPLORED: # go forward
                            self.turn(motor, 0)
                        elif nint.streets[(heading + 1) % 4] is constants.UNEXPLORED: # turn L
                            print(nint.streets[heading])
                            self.turn(motor,1)
                            heading = (heading + 1) % 4
                        elif nint.streets[(heading + 3) % 4] is constants.UNEXPLORED: # turn R
                            self.turn(motor,3)
                            heading = (heading + 3) % 4
                    else:
                        print("there is a connected")
                        connected_streets = []
                        for y in range(len(nint.streets)):
                            if nint.streets[y] is constants.CONNECTED:
                                #print("CONNECTED")
                                if (y != (heading + 2) % 4):
                                    connected_streets.append(y)
                        choice2 = random.choice(connected_streets)
                        self.turn(motor,(choice2 - heading) % 4)
                        heading = choice2
                
                #IF ROAD WAS BLOCKED
                else:
                    turned_around = False
                    self.intersection(long, lat).streets[heading] = BLOCKED #unsure of this is right 
                    heading = (heading + 2) % 4
                    print("due to block, was here just now")

                    #AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
                    if constants.UNEXPLORED in nint.streets:
                        print("there is an unexplored")
                        if nint.streets[heading] is constants.UNEXPLORED: # go forward
                            self.turn(motor, 0)
                        elif nint.streets[(heading + 1) % 4] is constants.UNEXPLORED: # turn L
                            print(nint.streets[heading])
                            self.turn(motor,1)
                            heading = (heading + 1) % 4
                        elif nint.streets[(heading + 3) % 4] is constants.UNEXPLORED: # turn R
                            self.turn(motor,3)
                            heading = (heading + 3) % 4
                    else:
                        print("there is a connected")
                        connected_streets = []
                        for y in range(len(nint.streets)):
                            if nint.streets[y] is constants.CONNECTED:
                                #print("CONNECTED")
                                if (y != (heading + 2) % 4):
                                    connected_streets.append(y)
                        choice2 = constants.random.choice(connected_streets)
                        self.turn(motor,(choice2 - heading) % 4)
                        heading = choice2
                    #AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH


                # checking if unexplored streets still exist
                for x in self.intersections:
                    if constants.UNEXPLORED in x.streets:
                        unx_cntr = unx_cntr + 1                        
                
                # update after  first
                self.lastintersection = [long,lat]
                print(self.lastintersection)

                if unx_cntr ==0: # if everything is explored, exit loop
                    stop_condition = False
                    print("EVERYTHING MAPPED") 

        motor.set(0,0)

    # moving to a target
    def dijkstras(self,motor):
        print("running dijkstras")
        global heading
        while True:
            # store where bot currently is
            curr_intersection =(self.long, self.lat)
            time.sleep(1)

            # clear all headingtoTargets
            for i in self.intersections:
                i.headingToTarget = None
            
            # target intersection is new goal and added to on_deck
            goal = (self.x,self.y)
            on_deck = [(self.x,self.y)] # FIFO list
            processed = []

            # list of all the (long, lat)
            int_coords = []
            for y in self.intersections:
                int_coords.append((y.self.long,y.self.lat))
            
            print("intersections",int_coords)

            while(curr_intersection not in processed):
                print("hello")
                temp_target = on_deck[0]
                on_deck = on_deck[1:]
                processed.append(temp_target)
                
                 # get 4 intersections around it
                curr_north = (temp_target[0],temp_target[1]+1)
                curr_west = (temp_target[0]-1,temp_target[1])
                curr_south = (temp_target[0],temp_target[1]-1)
                curr_east = (temp_target[0] + 1,temp_target[1])

                # check if any of these 4 are valid intersections
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
            while (self.long,self.lat)!= goal:
                cint = self.intersection(self.long,self.lat)
                self.turn(motor,(cint.headingToTarget - heading) % 4)
                self.drivebehavior(motor)
                heading = (cint.headingToTarget) % 4
                (self.long, self.lat) = self.shift(self.long,self.lat,heading) 
            motor.set(0,0)

    # Driving thread: drive from intersection to intersection
    def driving_stop(self):
        self.driving_stopflag = True
    
    def driving_loop(self,motor):
        self.driving_stopflag = False
        while not self.driving_stopflag:
            if self.keepdriving: #keep mapping
                self.mapping(motor)
            
            elif self.gotoint: # go to an intersection
                self.dijkstras(motor)

            else: # Pause at this intersection, if requested
                motor.set(0,0)





    	


