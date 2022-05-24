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

    sharp_turn = 0.3;
    turn_const = 0.4;
    power_const = 0.8;

    print(io.read(IR_R))
    print(io.read(IR_L))
    print(io.read(IR_M))
    past_mid = 0
    past_right = 0
    past_left = 0 
    searching = True
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
            if mid_val == 1 and right_val == 0 and left_val == 0:
                motor.set(power_const, power_const)
                searching = False
            # sligth left
            elif left_val == 0 and right_val ==1 and mid_val == 1:
                motor.set(power_const, power_const*turn_const)
                searching = False
            # very left
            elif left_val == 0 and mid_val == 0 and right_val == 1:
                motor.set(power_const, power_const*sharp_turn)
                searching = False
            # slight right
            elif left_val == 1 and mid_val ==1 and right_val == 0:
                motor.set(power_const*turn_const, power_const)
                searching = False
            # very right
            elif left_val == 1 and mid_val == 0 and right_val == 0:
                motor.set(power_const*sharp_turn, power_const)
                searching = False

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
 
                elif searching:
                   motor.set(power_const, power_const)

            elif left_val == 1 and right_val == 1 and mid_val == 1:
                motor.set(power_const, -1*power_const)
                searching = False
            else:
                motor.set(0, 0)
            
            past_left = left_val
            past_right = right_val
            past_mid = mid_val
    except:
        print('hello')
        motor.set(0,0)

# Turn off the robot
# SET MOTOR TO ZERO.
