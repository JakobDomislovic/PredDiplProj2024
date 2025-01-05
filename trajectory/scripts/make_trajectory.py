#!/usr/bin/env python3

# On GitHub, I will post an image better detailing the thinking process behind the code.

import math
from geometry_msgs.msg import Point

if __name__ == "__main__":
    
    ## Initialization ##
    #TODO add it so we can input the end point
    start = Point(0.0, 0.0, 8.0)
    end   = Point(3.0, 0.0, 5.0)
    print(f"Starting point:\n{start}")
    print(f"End point:\n{end}\n")

    ## Checks if we are simply descending on a target point ##
    # TODO Currently, this doesn't do anything. Ideally, descending=True
    # should make it so that we only need one half-parabola calculation
    if start.z > end.z:
        descending = True
    else:
        descending = False
    print(f"Descending: {descending}")

    ## Trajectory parameters ##
    # How much higher will vertex be above target point max height
    overshoot = 1
    print(f"Overshoot: {overshoot}")
    # Waypoint density - multiplier for number of waypoints in relation to distance
    # Must be an integer
    k = 3
    print(f"Waypoint density: {k}\n")


    ## Vertex calculation ##
    # Currently, we define the vertex to be +1 of max z-value (see parameters)
    # And its x,y values are exactly in between start and end point
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
    vi = int(n/2)
    
    ## Waypoints initialization ##
    waypoints = [None]*n
    waypoints[0]   = start
    waypoints[n-1] = end
    waypoints[vi]  = vertex

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
    a1 = (start.z - vertex.z)/((start_r - vertex_r)**2)
    a2 = (  end.z - vertex.z)/((  end_r - vertex_r)**2)

    ## Waypoints[] calculation ##
    #A loop for start > vertex segment
    for i in range(1, vi, 1):
        waypoints[i] = Point(start.x+(i*dx), start.y+(i*dy), a1*(i*dr - vertex_r)**2 + vertex.z)

    #A loop for end > vertex segment
    for i in range (n-2, vi, -1):
        waypoints[i] = Point(start.x+(i*dx), start.y+(i*dy), a2*(i*dr - vertex_r)**2 + vertex.z)

    print("Waypoints are:")
    for val in waypoints:
        print(f"{val}\n")

#TODO There is no need for a higher vertex in case of end point being much lower than starting point.
#     In that case, one half-parabola will be sufficient and more efficient.
#TODO Find a way to return waypoints[] and run it through trajectory_ros_testing.py