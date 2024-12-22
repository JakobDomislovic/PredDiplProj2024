#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point

if __name__ == "__main__":
    
    # We can make a parabola using 2 point and a vertex
    # TODO make it so we can input the point
    start = Point(0,0,0)
    end = Point(2,3,5)
    vertex = Point()

    # To keep things simple, we can make the vertex
    # x-value and y-value be right between x and y
    # value of our start and end point
    # TODO fix for negative values
    vertex.x = (end.x - start.x)/2
    vertex.y = (end.y - start.y)/2

    # s for the height of the vertex, for now we will
    # just use the higher of the 2 points +1
    vertex.z = max(start.z, end.z) + 1

    # Now we find a plane on which parabola lays
    p0 = start
    p1 = end
    p2 = vertex

    # Now we find 2 vectors on the plane
    p01 = [p1.x-p0.x, p1.y-p0.y, p1.z-p0.z]
    p02 = [p2.x-p0.x, p2.y-p0.y, p2.z-p0.z]

    # With these vectors we can calculate normal vector of the plane
    # n=[a,b,c]
    n = [p01[1]*p02[2]-p01[2]*p02[1],
         p01[2]*p02[0]-p01[0]*p02[2],
         p01[0]*p02[1]-p01[1]*p02[0]]

    # We use the following plane equation:
    # aX + bY + cZ + k = 0
    # Which means we need to find 'k' by inputing
    # any of the points into our equation
    a = n[0]
    b = n[1]
    c = n[2]

    k = - a*p0.x - b*p0.y - c*p0.z

    # Finally, we have our plane equation
    print("Equation of our plane is:")
    print(f"{a}X + {b}*Y + {c}*Z + {k} = 0")

    # TODO seperate plane calcualtion in a seperate method/function/class