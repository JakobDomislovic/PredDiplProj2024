#!/usr/bin/env python

import tf
import rospy
import numpy as np
from typing import List
from std_msgs.msg import String
from geometry_msgs.msg import Transform, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from make_trajectory import get_waypoints_with_rpy
from geometry_msgs.msg import Point, PoseStamped


class TrajectoryROSTesting:
    
    def __init__(self, rate: int) -> None:
        
        self.rate: rospy.Rate = rospy.Rate(rate)
        self.tracker_status: str = "idle"

        self.start  = Point(0.0, 0.0, 3.0)
        self.target = Point(3.0, 4.0, 5.0)

        # subscribers 
        self.tracker_status_sub: rospy.Subscriber = rospy.Subscriber(
            '/duckorange/tracker/status', String, self.tracker_status_cb)
        self.tracker_target_sub: rospy.Subscriber = rospy.Subscriber(
            '/duckorange/tracker/target', Point, self.get_target) # Maybe change the topic
        self.tracker_target_sub: rospy.Subscriber = rospy.Subscriber(
            '/duckorange/pose', PoseStamped, self.get_current)
        
        # publishers
        self.traj_pub: rospy.Publisher = rospy.Publisher(
            '/duckorange/tracker/input_trajectory', MultiDOFJointTrajectory, queue_size=1)

    def get_target(self, msg: Point) -> None:
        # TODO: Add coordinate converter
        if self.target is None:
            self.target = msg

    def get_current(self, msg: PoseStamped) -> None:
        # TODO: Add coordinate converter
        if self.start is None:
            self.start = msg.pose.position

    def tracker_status_cb(self, msg: String) -> None:
        self.tracker_status = msg.data
    
    def run(self) -> None:
        rospy.loginfo("Started...")
        at_endpoint = False
        landed = False

        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.tracker_status == "ACCEPT":
                if self.start is None or self.target is None:
                    continue

                elif not at_endpoint: # Ready to start the trajectory
                    print("Tracker status ACCEPT. Publishing trajectory...")
                    print("Current: ", self.start)
                    print("Target: ", self.target)
                    waypoints = get_waypoints_with_rpy(self.start, self.target)
                    self.publish_trajectory(waypoints)
                    at_endpoint = True

                elif not landed: # Ended trajectory, landing on the current position
                    print("At endpoint. Publishing landing trajectory...")
                    print("Current: ", self.target)
                    self.stop_and_land(self.target)
                    self.target = None
                    self.current = None
                    landed = True
                else: # Ended trajectory and landed
                    if self.target is None: # Next target is not known
                        print("Landed. Shutting down the node.")
                        break 
                    else: # Next target is not known
                        print("Landed. Continuing to a new target.")
                        landed = False
                        at_endpoint = False
                
    
    def publish_trajectory(self, waypoints: List[List]) -> None:
        traj = self.create_trajectory(waypoints)
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
        
        traj_testing: TrajectoryROSTesting = TrajectoryROSTesting(rate)
        traj_testing.run()
    except rospy.ROSInterruptException as e:
        print("ROSInterruptException")
        print(e)