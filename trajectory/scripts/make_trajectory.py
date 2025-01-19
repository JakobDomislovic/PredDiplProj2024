#!/usr/bin/env python3

# Code currently assumes (x,y) of start are always (0,0)
import math
from geometry_msgs.msg import Point

if __name__ == "__main__":
    
    ## Initialization ##
    current = Point( 2.0,-2.0, 1.0)
    target  = Point(-4.0, 3.0, 7.0)
    print(f"Current position:\n{current}")
    print(f"Target position:\n{target}\n")

    ## Transformation of coordinates ##
    start = Point(0.0, 0.0, current.z)
    end   = Point(target.x - current.x, target.y - current.y, target.z)
    print(f"Starting point:\n{start}")
    print(f"End point:\n{end}\n")

    ## Checks if we are simply descending on a target point ##
    # In which case, we only need 1 half-parabola
    if start.z > end.z + 1:
        descending = True
    else:
        descending = False
    print(f"Descending: {descending}")

    ## Trajectory parameters ##
    # How much higher will vertex be above target point max height
    overshoot = 1
    if descending:
        overshoot = 0
    else:
        print(f"Overshoot: {overshoot}")
    # Waypoint density - multiplier for number of waypoints in relation to distance
    # Must be an integer
    k = 1
    print(f"Waypoint density: {k}\n")


    ## Vertex calculation ##
    # Currently, we define the vertex to have its (x,y) in between start and end point
    # And its 'z' to be +overshoot above max value
    vertex = Point()
    if descending:
        vertex = start
    else:
        vertex.x = (end.x + start.x)/2
        vertex.y = (end.y + start.y)/2
        vertex.z = max(start.z, end.z) + overshoot
    print(f"Vertex:\n{vertex}")

    true_vertex = Point(vertex.x + current.x, vertex.y + current.y, vertex.z)
    print(f"True vertex:\n{true_vertex}\n")

    ## Vector distanes between start and end ##
    # These are important for later calculation
    x_dis = (end.x - start.x)
    y_dis = (end.y - start.y)
    r_dis = math.sqrt(x_dis**2 + y_dis**2)

    ## Number of waypoints ##
    # Currently, we define the number of waypoints as:
    # n = 'k' times the radial distance (rounded) + 1
    n = k * int(r_dis) + 1
    # Number of waypoints has to be odd (currently)
    if n%2 == 0:
        n=n-1
    print(f"Number of waypoints: {n}\n")

    ## Vertex index ##
    if descending:
        vi = 0
    else:
        vi = int(n/2)
    
    ## Waypoints initialization ##
    waypoints = [None]*n
    waypoints[0]   = start
    waypoints[vi]  = vertex
    waypoints[n-1] = end

    waypoints_true = waypoints

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
  
    print("Waypoints in (start,end) system:")
    for val in waypoints:
        print(f"{val}\n")

    ## Inverse system transformation ##
    print("Waypoints in true system:")
    for i in range (len(waypoints)):
        temp = waypoints[i]
        temp.x = temp.x + current.x
        temp.y = temp.y + current.y
        waypoints_true[i] = temp
        print(f"{temp}\n")

#TODO Coordinates could be rounded (to 2. decimal ideally) to make outputs more readable