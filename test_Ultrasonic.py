# Ultrasonic Class

# Imports
import pigpio
import sys
import time








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
        
        
        self.cbrise = self.io.callback(ULTRA_ECHO, pigpio.RISING_EDGE, self.rising)
        self.cbfall = self.io.callback(ULTRA_ECHO, pigpio.FALLING_EDGE, self.falling)

    def rising(self, gpio, level, tick):
            print("hi")
            self.rising_time = tick
            print(self.rising_time)

    def falling(self, gpio, level, tick):

            # divider 10000
            self.distance = ((343/2) * (tick- self.rising_time)) / 10000
            print(self.distance)
        
    def trigger(self):
            print('bye')
            # Pull one (or all) trigger pins HIGH
            self.io.write(self.ULTRA_TRIGGER, 1)
            # Hold for 10microseconds.
            time.sleep(0.000010)
            # Pull the pins LOW again.
            self.io.write(self.ULTRA_TRIGGER, 0)

                    # Set up rising and falling interrupt handlers or callbacks.
            # need rising and falling functions

        
    

    
