# Ultrasonic Class

# Imports
import pigpio
import sys
import time




rising_time = 0
distance = 0
class Ultrasonic:
    def __init__(self, ULTRA_TRIGGER, ULTRA_ECHO, io):    
    	# Initialize the connection to the pigpio daemon (GPIO interface).
        self.io = io
        if not self.io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)

        self.ULTRA_ECHO = ULTRA_ECHO
        self.ULTRA_TRIGGER = ULTRA_TRIGGER
        self.distance = 0
        self.rising_time = 0

            ############################################################
        # Prepare the GPIO connetion (to command the motors).
        print("Setting up the GPIO...")
    	# Set up the four pins as output (commanding the motors).
        self.io.set_mode(ULTRA_TRIGGER, pigpio.OUTPUT)
        self.io.set_mode(ULTRA_ECHO, pigpio.INPUT)

        
        # *OR* set up one handler for both.
        self.cb = io.callback(ULTRA_ECHO, pigpio.EITHER_EDGE, self.either)


    
    def either(self, gpio, level, tick):
        global rising_time
        global distance
        if level == 1:
            rising_time = tick
        elif level == 0:
            distance = ((343/2) * (tick- rising_time)) / 10000
            print(distance)
        else:
            print("error")
        
    def trigger(self):
            # Pull one (or all) trigger pins HIGH
            self.io.write(self.ULTRA_TRIGGER, 1)
            # Hold for 10microseconds.
            time.sleep(0.000010)
            # Pull the pins LOW again.
            self.io.write(self.ULTRA_TRIGGER, 0)

                    # Set up rising and falling interrupt handlers or callbacks.
            # need rising and falling functions
    
        
    

    
