# Imports
import pigpio
import sys
import time
# Motor class
from Motor import Motor

# Define the motor pins.
MTR1_LEGA = 7
MTR1_LEGB = 8

MTR2_LEGA = 6
MTR2_LEGB = 5


IR_R  = 14
IR_M =  15
IR_L = 18 

io = pigpio.pi()
if __name__ == "__main__":
    motor = Motor()
    
    turn_const = 0.5;
    power_const = 0.8;
    print(io.read(IR_R))
    print(io.read(IR_L))
    print(io.read(IR_M))
    try:
        while True:
            right_val = io.read(IR_R)
            left_val = io.read(IR_L)
            mid_val = io.read(IR_M)
            # Take appropriate action
            print(right_val)
            print(left_val)
            print(mid_val)
            # veering left
            if right_val == 1:
                motor.set(power_const, power_const*turn_const)
                  
            elif left_val == 1:
                motor.set(power_const*turn_const, power_const)
            
            elif mid_val == 1 and right_val == 0 and left_val == 0:
                motor.set(power_const, power_const)
                
            else:
                motor.set(0, 0)
    except BaseException as ex:
        motor.set(0,0)
        
# Turn off the robot
# SET MOTOR TO ZERO.
