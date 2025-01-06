#!/usr/bin/env python3

# Code currently assumes (x,y) of start are always (0,0)
import math
from geometry_msgs.msg import Point

if __name__ == "__main__":
    
    ## Initialization ##
    #TODO add it so we can input the end point
    start = Point(0.0, 0.0, 3.0) #For testing, change only z-value
    end   = Point(3.0, 4.0, -3.0)
    print(f"Starting point:\n{start}")
    print(f"End point:\n{end}\n")

    ## Checks if we are simply descending on a target point ##
    # In which case, we only have 1 half-parabola
    if start.z > end.z + 1:
        descending = True
    else:
        descending = False
    print(f"Descending: {descending}")

    ## Trajectory parameters ##
    # TODO In the ros class, these two should be ROS parameters
    # How much higher will vertex be above target point max height
    overshoot = 1
    if descending:
        overshoot = 0
    else:
        print(f"Overshoot: {overshoot}")
    # Waypoint density - multiplier for number of waypoints in relation to distance
    # Must be an integer
    k = 2
    print(f"Waypoint density: {k}\n")


    ## Vertex calculation ##
    # Currently, we define the vertex to have its (x,y) in between start and end point
    # And its 'z' to be +overshoot above max value
    if descending:
        vertex = start
    else:
        vertex = Point()
        vertex.x = (end.x + start.x)/2
        vertex.y = (end.y + start.y)/2
        vertex.z = max(start.z, end.z) + overshoot
    print(f"Vertex:\n{vertex}\n")

    ## Number of waypoints ##
    # Currently, we define the number of waypoints as:
    # n = 'k' times the radial distance (rounded) + 1
    x_dis = abs(end.x - start.x)
    y_dis = abs(end.y - start.y)
    r_dis = math.sqrt(x_dis**2 + y_dis**2)
    n = k * int(r_dis) + 1
    # Number of waypoints has to be odd (currently)
    if n%2 == 0:
        n=n-1
    print(f"Number of waypoints: {n}\n")

    ## Vertex index in waypoints[] list ##
    if descending:
        vi = 0
    else:
        vi = int(n/2)
    
    ## Waypoints initialization ##
    waypoints = [None]*n
    waypoints[0]   = start
    waypoints[vi]  = vertex
    waypoints[n-1] = end

    ## Equation parameters ##
    # Distance segments between waypoints
    dx = x_dis/(n-1)
    dy = y_dis/(n-1)
    dr = r_dis/(n-1)
    # Radial coordinates of start, end, vertex
    start_r  = math.sqrt( start.x**2 +  start.y**2)
    end_r    = math.sqrt(   end.x**2 +    end.y**2)
    vertex_r = math.sqrt(vertex.x**2 + vertex.y**2)
    # Quadratic equation coefficients
    if not descending: #We need this 'if' otherwise we get division by 0 when descending
        a1 = (start.z - vertex.z)/((start_r - vertex_r)**2)
    a2 = (  end.z - vertex.z)/((  end_r - vertex_r)**2)

    ## Waypoints[] calculation ##
    if descending:
        for i in range(1, n-1, 1):
            waypoints[i] = Point(start.x+(i*dx), start.y+(i*dy), a2*((i*dr - vertex_r)**2) + vertex.z)
    else:
        #A loop for start > vertex segment
        for i in range(1, vi, 1):
            waypoints[i] = Point(start.x+(i*dx), start.y+(i*dy), a1*((i*dr - vertex_r)**2) + vertex.z)

        #A loop for end > vertex segment
        for i in range (n-2, vi, -1):
            waypoints[i] = Point(start.x+(i*dx), start.y+(i*dy), a2*((i*dr - vertex_r)**2) + vertex.z)

    print("Waypoints are:")
    for val in waypoints:
        print(f"{val}\n")

#TODO Find a way to return waypoints[] and run it through trajectory_ros_testing.py
#TODO Coordinates could be rounded (to 2. decimal ideally) to make outputs more readable