#!/usr/bin/env python3

# On GitHub, I will post an image better detailing the thinking process behind the code.

import math
from geometry_msgs.msg import Point

if __name__ == "__main__":
    
    ## Initialization
    #TODO add it so we can input the end point
    start = Point(0.0, 0.0, 0.0)
    end   = Point(3.0, 0.0, 5.0)
    print(f"Starting point:\n{start}")
    print(f"End point:\n{end}")

    ## Trajectory parameters
    # How much higher will vertex be above start/end max height
    vertical_offset = 1
    # Waypoint density (TODO currently, it must be an even number, needs fixing)
    k = 6

    ## Vertex calculation
    # Currently, we define the vertex to be +1 of max z-value (see parameters)
    # And its x,y values are exactly in between start and end point
    vertex = Point()
    vertex.x = (end.x + start.x)/2
    vertex.y = (end.y + start.y)/2
    vertex.z = max(start.z, end.z) + vertical_offset
    
    print(f"Vertex:\n{vertex}")

    ## Calculate the number of waypoints
    # Currently, we define the number of waypoints as:
    # n = 'k' times the greater distance (rounded) + 1
    # We need an odd number of waypoints because of the way we define our vertex
    x_dis = abs(end.x - start.x)
    y_dis = abs(end.y - start.y)
    n = k * int(max(x_dis, y_dis)) + 1
    print(f"Number of waypoints: {n}")

    ## Vertex index in waypoints[] list
    vi = int(n/2)
    
    ## Waypoints initialization
    waypoints = [None]*n
    waypoints[0]   = start
    waypoints[n-1] = end
    waypoints[vi]  = vertex

    ## Equation parameters
    # Distance segments between waypoints
    dx = x_dis/(n-1)
    dy = y_dis/(n-1)
    dr = math.sqrt(dx**2 + dy**2)
    # Radial coordinates of start, end, vertex
    start_r  = math.sqrt( start.x**2 +  start.y**2)
    end_r    = math.sqrt(   end.x**2 +    end.y**2)
    vertex_r = math.sqrt(vertex.x**2 + vertex.y**2)
    # Quadratic equation coefficients
    a1 = (start.z - vertex.z)/((start_r - vertex_r)**2)
    a2 = (  end.z - vertex.z)/((  end_r - vertex_r)**2)

    ## A loop for start > vertex segment (currently, i=1,2)
    for i in range(1, vi, 1):
        waypoints[i] = Point(start.x+(i*dx), start.y+(i*dy), a1*(i*dr - vertex_r)**2 + vertex.z)

    ## A loop for end > vertex segment (currently, i=5,4)
    for i in range (n-2, vi, -1):
        waypoints[i] = Point(start.x+(i*dx), start.y+(i*dy), a2*(i*dr - vertex_r)**2 + vertex.z)

    print("Waypoints are:\n")
    for val in waypoints:
        print(f"{val}\n")

#TODO Find a way to return waypoints[] and run it through trajectory_ros_testing.py