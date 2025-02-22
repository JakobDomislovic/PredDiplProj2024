#!/usr/bin/env python3
import math
from geometry_msgs.msg import Point
from typing import List

def get_waypoints(start: Point, end: Point) -> List[Point]:
    
## Initialization ##
    print(f"Starting point:\n{start}")
    print(f"End point:\n{end}\n")

## Descending check ##
    # If descending we only need 1 half-parabola
    if start.z > end.z + 1:
        descending = True
    else:
        descending = False
    print(f"Descending: {descending}")

## Trajectory parameters ##
    # Vertex height overshoot (modifiable)
    overshoot = 1
    if descending:
        overshoot = 0
    else:
        print(f"Overshoot: {overshoot}")
    
    # Waypoint density multiplier - must be an integer (modifiable)
    k = 2
    print(f"Waypoint density multiplier: {k}\n")


## Vertex initialization ##
    if descending:
        vertex = start
    else:
        # We choose vertex to be exactly halfway (modifiable)
        vertex = Point()
        vertex.x = (end.x + start.x)/2
        vertex.y = (end.y + start.y)/2
        vertex.z = max(start.z, end.z) + overshoot
    print(f"Vertex:\n{vertex}\n")

## Coordinate shifting ##
    end.x    = end.x    - start.x
    end.y    = end.y    - start.y
    vertex.x = vertex.x - start.x
    vertex.y = vertex.y - start.y
    start.x  = 0
    start.y  = 0

## Distances ##
    x_dis = (end.x - start.x)
    y_dis = (end.y - start.y)
    r_dis = math.sqrt(x_dis**2 + y_dis**2)

## Number of waypoints (must be odd) ##
    n = k * int(r_dis) + 1
    if (n%2 == 0):
        n = n - 1
    print(f"Number of waypoints: {n}\n")

## Vertex indexing ##
    if descending:
        vi = 0
    else:
        vi = int(n/2)
    
## Waypoints[] initialization ##
    waypoints = [None]*n
    waypoints[0]   = start
    waypoints[vi]  = vertex
    waypoints[n-1] = end

## Equation parameters ##
    # Distance segments between waypoints
    dx = x_dis/(n-1)
    dy = y_dis/(n-1)
    dr = r_dis/(n-1)

    # Radial coordinates
    start_r  = math.sqrt( start.x**2 +  start.y**2)
    end_r    = math.sqrt(   end.x**2 +    end.y**2)
    vertex_r = math.sqrt(vertex.x**2 + vertex.y**2)

    # Quadratic equation coefficients
    if not descending:
        a1 = (start.z - vertex.z)/((start_r - vertex_r)**2)
    a2 = (end.z - vertex.z)/((end_r - vertex_r)**2)

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
        val.x = round(val.x, 2)
        val.y = round(val.y, 2)
        val.z = round(val.z, 2)
        print(f"{val}\n")
    return waypoints


def get_waypoints_with_rpy(start: Point, end: Point) -> List[List]:
    waypoints = get_waypoints(start, end)
    new_waypoints = []
    for val in waypoints:
        # x, y, z, R, P, Y, vel_xy, acc_xy, vel_z, acc_z
        point = [val.x, val.y, val.z, 0, 0, 0, 0, 0, 0, 0]
        new_waypoints.append(point)
    return new_waypoints

# Testing
if __name__ == "__main__":
    start = Point(-1.0, 2.0, 1.0)
    end   = Point(4.0, -4.0, 3.0)

    waypoints = get_waypoints(start, end)
