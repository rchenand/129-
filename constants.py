# Imports
import pigpio
import sys
import time
import random
import threading


# GPIO pins
IR_R  = 14
IR_M =  15
IR_L = 18

ULTRA1_TRIGGER = 13
ULTRA1_ECHO = 16

ULTRA2_TRIGGER = 19
ULTRA2_ECHO = 20

MTR1_LEGA = 8
MTR1_LEGB = 7

MTR2_LEGA = 5
MTR2_LEGB = 6



# GLOBAL CONSTANTS
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

