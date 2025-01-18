#!/usr/bin/env python

import tf
import rospy
import numpy as np
import math
from typing import List
from std_msgs.msg import String
from geometry_msgs.msg import Transform, Twist
from geometry_msgs.msg import Point
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from nav_msgs.msg import Odometry


class SimulationTesting:
    
    def __init__(self, rate: int, target: Point) -> None:
        
        self.rate: rospy.Rate = rospy.Rate(rate)
        self.target = Point(target[0], target[1], target[2])
        self.tracker_status: str = "idle"

        self.current_position = Odometry
        
        # subscribers 
        self.tracker_status_sub: rospy.Subscriber = rospy.Subscriber(
            '/duckorange/tracker/status', String, self.tracker_status_cb)
        self.current_position_sub: rospy.Subscriber = rospy.Subscriber(
            '/duckorange/odometry', Odometry, self.get_current_position)
        
        # publishers
        self.traj_pub: rospy.Publisher = rospy.Publisher(
            '/duckorange/tracker/input_trajectory', MultiDOFJointTrajectory, queue_size=1)

#--------------------------------------------------------------------------------------------------
    def tracker_status_cb(self, msg: String) -> None:
        self.tracker_status = msg.data

#--------------------------------------------------------------------------------------------------    
    def run(self) -> None:
        rospy.loginfo("Started...")
        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.tracker_status == "ACCEPT":
                print("Tracker status ACCEPT. Publishing trajectory...")
                self.publish_trajectory()

#--------------------------------------------------------------------------------------------------    
    def publish_trajectory(self) -> None:
        traj = self.create_trajectory(self.target)
        self.traj_pub.publish(traj)

#--------------------------------------------------------------------------------------------------
    def get_current_position(self, msg: Odometry) -> None:
        self.current_position = msg

#--------------------------------------------------------------------------------------------------
    def waypoints_generator(self, target: Point) -> List:
        # Coordinates need to be shift so that the start is (0,0,z)
        start = Point(0, 0, self.current_position.pose.pose.position.z)
        end = Point()
        end.x = target.x - self.current_position.pose.pose.position.x
        end.y = target.y - self.current_position.pose.pose.position.y
        end.z = target.z
        
        # Check if we are descending
        if start.z > end.z + 1:
            descending = True
        else:
            descending = False

        # Parabola parameters
        k = 2
        overshoot = 1
        if descending:
            overshoot = 0

        # Defining vertex
        if descending:
            vertex = start
        else:
            vertex = Point()
            vertex.x = (end.x + start.x)/2
            vertex.y = (end.y + start.y)/2
            vertex.z = max(start.z, end.z) + overshoot

        # Number of waypoints
        x_dis = abs(end.x - start.x)
        y_dis = abs(end.y - start.y)
        r_dis = math.sqrt(x_dis**2 + y_dis**2)
        n = k * int(r_dis) + 1
        if n%2 == 0:
            n=n-1

        # Vertex indexing
        if descending:
            vi = 0
        else:
            vi = int(n/2)

        # List initialization
        waypoints_true = [None]*n
        waypoints = [None]*n
        waypoints[0]   = start
        waypoints[vi]  = vertex
        waypoints[n-1] = end

        # Distances between waypoints and radial distances
        dx = x_dis/(n-1)
        dy = y_dis/(n-1)
        dr = r_dis/(n-1)
        start_r  = math.sqrt( start.x**2 +  start.y**2)
        end_r    = math.sqrt(   end.x**2 +    end.y**2)
        vertex_r = math.sqrt(vertex.x**2 + vertex.y**2)

        # Parabola cooeficients
        if not descending:
            a1 = (start.z - vertex.z)/((start_r - vertex_r)**2)
        a2 = (  end.z - vertex.z)/((  end_r - vertex_r)**2)

        # Waypoint factory
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
        
        # Reformating for MultiDOFJointTrajectory and inverse transforming
        for i in range(0, len(waypoints)):
           temp = [0]*10
           temp[0] = waypoints[i].x + self.current_position.pose.pose.position.x
           temp[1] = waypoints[i].y + self.current_position.pose.pose.position.y
           temp[2] = waypoints[i].z
           waypoints_true[i] = temp
        
        return waypoints_true

#--------------------------------------------------------------------------------------------------
    def create_trajectory(self, target : Point) -> MultiDOFJointTrajectory:
        multi_dof_trajectory = MultiDOFJointTrajectory()
        waypoints = self.waypoints_generator(target)
        for i in range(0, len(waypoints)):
            temp_point = MultiDOFJointTrajectoryPoint()
            temp_transform = Transform()
            temp_vel = Twist()
            temp_acc = Twist()
            temp_transform.translation.x = waypoints[i][0]
            temp_transform.translation.y = waypoints[i][1]
            temp_transform.translation.z = waypoints[i][2]
            quaternion = tf.transformations.quaternion_from_euler(
                waypoints[i][3], waypoints[i][4], waypoints[i][5])
            temp_transform.rotation.x = quaternion[0]
            temp_transform.rotation.y = quaternion[1]
            temp_transform.rotation.z = quaternion[2]
            temp_transform.rotation.w = quaternion[3]
            temp_vel.linear.x = waypoints[i][6]
            temp_vel.linear.y = waypoints[i][6]
            temp_vel.linear.z = waypoints[i][8]            
            temp_acc.linear.x = waypoints[i][7]
            temp_acc.linear.y = waypoints[i][7]
            temp_acc.linear.z = waypoints[i][9]
            temp_point.transforms.append(temp_transform)
            temp_point.velocities.append(temp_vel)
            temp_point.accelerations.append(temp_acc)

            multi_dof_trajectory.points.append(temp_point)

        return multi_dof_trajectory

#--------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        rospy.init_node('simulation_testing')
        
        rate: int = 1
        target: Point = [3.0, 3.0, 4.0]
        
        traj_testing: SimulationTesting = SimulationTesting(rate, target)
        traj_testing.run()
    except rospy.ROSInterruptException as e:
        print("ROSInterruptException")
        print(e)