#!/usr/bin/env python

import tf
import rospy
import numpy as np
from typing import List
from std_msgs.msg import String
from geometry_msgs.msg import Transform, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from make_trajectory import get_waypoints_with_rpy
from geometry_msgs.msg import Point


class TrajectoryROSTesting:
    
    def __init__(self, rate: int, waypoints: List) -> None:
        
        self.rate: rospy.Rate = rospy.Rate(rate)
        self.waypoints: List = waypoints
        self.tracker_status: str = "idle"
        
        # subscribers 
        self.tracker_status_sub: rospy.Subscriber = rospy.Subscriber(
            '/duckorange/tracker/status', String, self.tracker_status_cb)
        # TODO: dodati subscriber za dohvacanje sljedece tocke
        # TODO: dodati subscriber za dohvacanje trenutne pozicije, tako da
        #       kreiramo eliptičku trajektoriju u odnosu na to
        
        # publishers
        self.traj_pub: rospy.Publisher = rospy.Publisher(
            '/duckorange/tracker/input_trajectory', MultiDOFJointTrajectory, queue_size=1)

    def tracker_status_cb(self, msg: String) -> None:
        self.tracker_status = msg.data
    
    def run(self) -> None:
        rospy.loginfo("Started...")
        at_endpoint = False
        landed = False

        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.tracker_status == "ACCEPT":
                if not at_endpoint:
                    print("Tracker status ACCEPT. Publishing trajectory...")
                    self.publish_trajectory()
                    at_endpoint = True
        
                elif at_endpoint and not landed:
                    print("At endpoint. Publishing landing trajectory...")
                    last_waypoint = self.waypoints[-1]
                    current_position = Point()
                    current_position.x = last_waypoint[0]
                    current_position.y = last_waypoint[1]
                    current_position.z = last_waypoint[2]
                    self.stop_and_land(current_position)
                    landed = True

                else:
                    print("Landed. Shutting down the node.")
                    break
    
    def publish_trajectory(self) -> None:
        traj = self.create_trajectory(self.waypoints)
        self.traj_pub.publish(traj)
    
    def stop_and_land(self, current_position: Point) -> None:
        landing_waypoints = [
            [current_position.x, current_position.y, current_position.z, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [current_position.x, current_position.y, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ]  # One waypoint for current position and one for landing position
        landing_trajectory = self.create_trajectory(landing_waypoints)
        self.traj_pub.publish(landing_trajectory)
    
    def create_trajectory(self, waypoints) -> MultiDOFJointTrajectory:
        multi_dof_trajectory = MultiDOFJointTrajectory()
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


if __name__ == '__main__':
    try:
        rospy.init_node('trajectory_ros_elliptic')
        
        rate: int = 1
        start = Point(0.0, 0.0, 3.0)
        end   = Point(3.0, 4.0, 3.0)
        waypoints: List = get_waypoints_with_rpy(start, end)
        
        traj_testing: TrajectoryROSTesting = TrajectoryROSTesting(rate, waypoints)
        traj_testing.run()
    except rospy.ROSInterruptException as e:
        print("ROSInterruptException")
        print(e)