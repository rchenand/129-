# Imports
import pigpio
import sys
import time


# Define the motor pins.
MTR1_LEGA = 8
MTR1_LEGB = 7

MTR2_LEGA = 5
MTR2_LEGB = 6

class Motor: 
    def __init__(self):    
    	# Initialize the connection to the pigpio daemon (GPIO interface).
        self.io = pigpio.pi()
        if not self.io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)

    	# Set up the four pins as output (commanding the motors).
        self.io.set_mode(MTR1_LEGA, pigpio.OUTPUT)
        self.io.set_mode(MTR1_LEGB, pigpio.OUTPUT)
        self.io.set_mode(MTR2_LEGA, pigpio.OUTPUT)
        self.io.set_mode(MTR2_LEGB, pigpio.OUTPUT) 

    	# Clear all pins, just in case.
        self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGB, 0)
    
    def shutdown(self):
        self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGB, 0)
        self.io.stop()

	#setting all pins
    def set(self, leftdutycycle, rightdutycycle):
        if leftdutycycle < 0:
            self.io.set_PWM_dutycycle(MTR1_LEGA, abs(leftdutycycle*254))
            self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        if rightdutycycle < 0:
            self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
            self.io.set_PWM_dutycycle(MTR2_LEGB, abs(rightdutycycle*254))
        if leftdutycycle > 0:
            self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
            self.io.set_PWM_dutycycle(MTR1_LEGB, abs(leftdutycycle*254))
        if rightdutycycle > 0:
            self.io.set_PWM_dutycycle(MTR2_LEGA, abs(rightdutycycle*254))
            self.io.set_PWM_dutycycle(MTR2_LEGB, 0)
        if leftdutycycle == 0:
            self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
            self.io.set_PWM_dutycycle(MTR1_LEGB, 0)    
        if rightdutycycle == 0:
            self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
            self.io.set_PWM_dutycycle(MTR2_LEGB, 0)
          
    
    def setlinear(self, speed):
        newspeed = abs(1.43 * speed + 0.213)
        if (speed < 0):
            newspeed *= -1
        print(newspeed)
        if (newspeed) > 1:
            self.set(1 ,1)
        elif newspeed < -1:
            self.set(-1, -1)
        self.set(newspeed, newspeed)
    
    def setspin(self, speed):
        newspin = 1.24 * speed + 0.0461
        if abs(newspin) > 1: 
            self.set(1, 1)
        if newspin < 0:
            self.set(-newspin, newspin)
        else:
            self.set(newspin, -newspin)

    def setvel(self, linear, spin):
        print('hello')
        T = 2 * 3.14 / spin
        d = linear * T / 3.14
        robo_width = 15.24
        ratio = ((d / 2) + (robo_width)) / ( d/ 2)
        self.set(spin / (2 * 3.14), ((spin / (2 * 3.14)) * ratio))
