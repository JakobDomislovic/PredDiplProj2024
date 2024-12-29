#!/usr/bin/env python3

# Somewhere on GitHub, I have posted, or will post, an image better detailing
# the thinking process behind this code

import rospy
import math
from geometry_msgs.msg import Point

if __name__ == "__main__":
    
    # Initialization
    start     = Point(0,0,0)
    end       = Point(3,0,5)
    vertex    = Point()
    waypoints = []

    ## Vertex caluculation
    # Currently, we define the vertex to be +1 of max z-value
    vertex.x = (end.x + start.x)/2
    vertex.y = (end.y + start.y)/2
    vertex.z = max(start.z, end.z) + 1

    ## Calculate the number of waypoints
    # Currently, we define the number of waypoints as:
    # n = Double the greater distance + 1
    # We need an odd number of waypoints because of the way we define our vertex
    x_dis = abs(end.x - start.x)
    y_dis = abs(end.y - start.y)
    n = 2 * round(max(x_dis, y_dis)+0.5) + 1

    # Vertex index
    vi = n/2 - 0.5
    
    # In this case we will have 2*3+1 = 7 waypoints (start,end,vertex + 4)
    waypoints[0]   = start
    waypoints[n-1] = end
    waypoints[vi]  = vertex

    # Now we need to find segments of x distance and y distance between each waypoint
    dx = x_dis/(n-1)
    dy = y_dis/(n-1)
    # No need for abs() here beacuse we know the vertex always has the greatest value
    dz_sv = vertex.z - start.z
    dz_ve = vertex.z - end.z

    # We have 4 waypoint which aren't start, end, or vertex
    # That means we need to insert them as [1][2] and then [5][4]

    # A loop for start->vertex segment (i=1,2)
    #TODO chose and implement a fraction that will derermine the z-value of waypoints
    for i in range(1, vi, 1):
        waypoints[i] = Point(start.x+(i*dx), start.y+(i*dy), {})

    # A loop for end->vertex segment (i=5,4)
    #TODO chose and implement a fraction that will derermine the z-value of waypoints
    for i in range (n-2, vi, -1):
        waypoints[i] = Point(start.x+(i*dx), start.y+(i*dy), {})

#TODO Find a way to return waypoint[] and run it through trajectory_ros_testing.py