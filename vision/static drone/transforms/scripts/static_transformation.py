#!/usr/bin/env python2

import rospy
import tf2_ros
import numpy as np
import tf
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf.transformations import euler_from_quaternion, euler_matrix, translation_matrix, concatenate_matrices, quaternion_from_matrix

# Global variables to store the marker pose relative to the camera
marker_translation = [0.0, 0.0, 0.0]
marker_rotation = [0.0, 0.0, 0.0]  # Roll, pitch, yaw

def pose_callback(msg):
    """
    Callback function for /aruco_single/pose to extract translation and rotation.
    """
    global marker_translation, marker_rotation

    # Extract translation
    marker_translation[0] = msg.pose.position.x
    marker_translation[1] = msg.pose.position.y
    marker_translation[2] = msg.pose.position.z

    # Extract rotation (quaternion -> roll, pitch, yaw)
    quat = msg.pose.orientation
    marker_rotation = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

def pose_to_matrix(translation, rotation):
    """
    Convert pose (translation and rotation) into a transformation matrix.
    """
    t_matrix = translation_matrix(translation)
    r_matrix = euler_matrix(rotation[0], rotation[1], rotation[2])  # Roll, pitch, yaw
    return concatenate_matrices(t_matrix, r_matrix)

def broadcast_transform():
    rospy.init_node('dynamic_ar_marker_transform_broadcaster')

    # Create a transform broadcaster
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10)  # 10 Hz

    # Subscribe to the /aruco_single/pose topic
    rospy.Subscriber('/aruco_single/pose', PoseStamped, pose_callback)

    # Static transformation of the UAV in the world frame
    x_world = 1.0  # Fixed X position
    y_world = 2.0  # Fixed Y position
    z_world = 1.5  # Fixed altitude
    yaw_world = 0.0  # Fixed yaw (no rotation)
    T_W_UAV = pose_to_matrix([x_world, y_world, z_world], [0.0, 0.0, yaw_world])

    # Static transformation from UAV to Camera (from URDF)
    T_UAV_CAM = pose_to_matrix([0.1, 0.0, 0.0], [0.0, 0.0, 0.0])  # Example values

    while not rospy.is_shutdown():
        # T_CAM_AR from USB camera pose estimation
        T_CAM_AR = pose_to_matrix(marker_translation, marker_rotation)

        # Compute T_W_AR
        T_W_AR = np.dot(np.dot(T_W_UAV, T_UAV_CAM), T_CAM_AR)

        # Extract translation and rotation for broadcasting
        t_w_ar_translation = T_W_AR[:3, 3]
        t_w_ar_rotation = tf.transformations.quaternion_from_matrix(T_W_AR)

        # Broadcast T_W_AR
        t_broadcast = TransformStamped()
        t_broadcast.header.stamp = rospy.Time.now()
        t_broadcast.header.frame_id = "world"
        t_broadcast.child_frame_id = "ar_marker"

        t_broadcast.transform.translation.x = t_w_ar_translation[0]
        t_broadcast.transform.translation.y = t_w_ar_translation[1]
        t_broadcast.transform.translation.z = t_w_ar_translation[2]
        t_broadcast.transform.rotation.x = t_w_ar_rotation[0]
        t_broadcast.transform.rotation.y = t_w_ar_rotation[1]
        t_broadcast.transform.rotation.z = t_w_ar_rotation[2]
        t_broadcast.transform.rotation.w = t_w_ar_rotation[3]

        br.sendTransform(t_broadcast)
        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_transform()
    except rospy.ROSInterruptException:
        pass