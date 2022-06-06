# -*- coding: utf-8 -*-
"""
Created on Tue May  3 13:47:56 2022

@author: 18123
"""
# Global Constants:
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
BLOCKED = 'Blocked'
# Global Variables:
intersections = [] # List of intersections
lastintersecion = None # Last intersection visited
long = 0 # Current east/west coordinate
lat = -1 # Current north/south coordinate
heading = NORTH # Current heading

def intersection(long, lat):    
        list = [i for i in intersections if i.long == long and i.lat == lat]
        if len(list) == 0:
            return None
        if len(list) > 1:
            raise Exception("Multiple intersections at (%2d,%2d)" % (long, lat))
        return list[0]

class Intersection:
# Initialize - create new intersection at (long, let)
    def __init__(self, long, lat):
# Save the parameters.
        self.long = long
        self.lat = lat
# Status of streets at the intersection, in NWSE directions.
        self.streets = [UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN]
# Direction to head from this intersection in planned move.
        self.headingToTarget = None
# You are welcome to implement an arbitrary number of
# "neighbors" to more directly match a general graph.
# But the above should be sufficient for a regular grid.
# Add this to the global list of intersections to make it searchable.
        global intersections
        if intersection(long, lat) is not None:
            raise Exception("Duplicate intersection at (%2d,%2d)" % (long,lat))
        intersections.append(self)
    # Print format.
    def __repr__(self):
        return ("(%2d, %2d) N:%s W:%s S:%s E:%s - head %s\n" %
                (self.long, self.lat, self.streets[0],
                 self.streets[1], self.streets[2], self.streets[3],
                 HEADING[self.headingToTarget]))
    # Find the intersection
    # New longitude/latitude value after a step in the given heading.
    def shift(long, lat, heading):
        if heading % 4 == NORTH:
            return (long, lat+1)
        elif heading % 4 == WEST:
            return (long-1, lat)
        elif heading % 4 == SOUTH:
            return (long, lat-1)
        elif heading % 4 == EAST:
            return (long+1, lat)
        else:
            raise Exception("This can’t be")
