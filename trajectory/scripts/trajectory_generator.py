#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point

if __name__ == "__main__":
    
    #We can make a parabole using 2 point and a vertex
    start = Point(0,0,0)
    end = Point(2,3,5)
    vertex = Point()

    #To keep things simple, we can make the vertex
    #x-value and y-value be right between x and y
    # value of our start and end point
    vertex.x = end.x - start.x
    vertex.y = end.y - start.y

    #As for the height of the vertex, for now we will
    #just use the higher of the 2 points +1
    vertex.z = max(start.z, end.z) + 1

    #No we find a plane on which parabola lays
    p0 = start
    p1 = end
    p2 = vertex

    #Now we find 2 vectors on the plane
    p01 = [p1.x-p0.x, p1.y-p0.y, p1.z-p0.z]
    p02 = [p2.x-p0.x, p2.y-p0.y, p2.z-p0.z]


    
    
    



