# Imports
import pigpio
import sys
import time


# Define the motor pins.
MTR1_LEGA = 7
MTR1_LEGB = 8

MTR2_LEGA = 6
MTR2_LEGB = 5

IR_R  = 14
IR_M = 15
IR_L = 18 

io = pigpio.pi()

io.set_mode(IR_R, pigpio.INPUT)
io.set_mode(IR_M, pigpio.INPUT)
io.set_mode(IR_L, pigpio.INPUT)

cntr = 0
while(cntr < 20):
   print(io.read(IR_L))
   print(io.read(IR_M))
   print(io.read(IR_R))
   time.sleep(1)
   cntr = cntr + 1


