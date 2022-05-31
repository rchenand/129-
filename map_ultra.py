'''Main method file'''

# Imports
import pigpio
import sys
import threading
import pickle
import constants

from Ultrasonic import Ultrasonic
from Motor import Motor
from Robot import Robot

#
#   Main
#
if __name__ == "__main__":
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)

    # create motor and robot objects for moving and mapping, etc.
    motor = Motor()
    robot = Robot()
    


    def ultra_detects(robot):
    # FIGURE OUT WHERE TO PUT THIS
        # blocked intersection
        # blocked street
        # tunnel 

        # when checking sides, first record values of flags before checking 

        # obstacle_detected, and remember to change flag after moving away
        if ultra2.distance < 20:
            robot.obstacle2_deteced = True;
        
        elif ultra1.distance < 20:
            robot.obstacle1_detected = True;
        
        elif ultra3.distance < 20:
            robot.obstacle3_detected = True;

        elif ultra1.distance < 20 and ultra3.distance < 20:
            robot.obstacle1_deteced = True;
            robot.obstacle3_deteced = True;
            # commence wall following
        
        else:
            ("nothing")


    def userinput(): # main thread
        while True:
        # Grab a command
            command = input("Command ? ")

            ultra_detects(robot)     # MAKE SURE THIS IS CONTINUOUSLY RUNNING, AND SETS FLAGS IN DRIVING ROBOT CLASS

            print(robot.obstacle2_detected)
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

            # load map
            elif (command[:4] == 'load'):
                print("Loading the map...")
                with open(command[5:]+'.pickle', 'rb') as file:
                    robot.intersections = pickle.load(file)

            # get curr location
            elif (command == 'location'):
                print("Currently at", (robot.long, robot.lat))
            
            # add change for location

            # quit
            elif (command == 'quit'):
                print("Quitting...")
                break
            else:
                print("Unknown command ’%s’" % command)

    # ultrasonic objects 
    ultra1 = Ultrasonic(constants.ULTRA1_TRIGGER, constants.ULTRA1_ECHO, io)
    ultra2 = Ultrasonic(constants.ULTRA2_TRIGGER, constants.ULTRA2_ECHO, io)
    ultra3 = Ultrasonic(constants.ULTRA3_TRIGGER, constants.ULTRA3_ECHO, io)

    # threads, ultrasonic, driving
    ultrasonic1_thread = threading.Thread(target=ultra1.runcontinual)
    ultrasonic1_thread.start()

    ultrasonic2_thread = threading.Thread(target=ultra2.runcontinual)
    ultrasonic2_thread.start()

    ultrasonic3_thread = threading.Thread(target=ultra3.runcontinual)
    ultrasonic3_thread.start()

    driving_thread = threading.Thread(target=robot.driving_loop,args=(motor,))
    driving_thread.start()

    try:
        userinput()


    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        motor.set(0,0)

    # Wait for the threads to be done (re-joined)
    ultra1.stopcontinual()
    ultrasonic1_thread.join()
    ultra2.stopcontinual()
    ultrasonic2_thread.join()
    ultra3.stopcontinual()
    ultrasonic3_thread.join()

    robot.driving_stop()
    driving_thread.join()

    #shutdown stuff

            
    
