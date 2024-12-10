#!/usr/bin/env python3

from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from scipy.spatial.transform import Rotation as R

def get_transform_matrix(position, orientation):
    # Extract rotation matrix from quaternion
    rotation_matrix = R.from_quat([orientation.x, orientation.y, orientation.z, orientation.w]).as_matrix()

    # Create transformation matrix directly
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = [position.x, position.y, position.z]
    return transform_matrix

def transform_to_parent_frame(pose, ref_frame_pose):
    # Compute transformation matrix for the reference frame
    ref_frame_transform_matrix = get_transform_matrix(ref_frame_pose.position, ref_frame_pose.orientation)
    ref_frame_inverse_transform = np.linalg.inv(ref_frame_transform_matrix)

    # Compute transformation matrix for the object
    object_transform_matrix = get_transform_matrix(pose.position, pose.orientation)

    # Compute transformation from object frame to reference frame
    object_to_ref_frame_transform = np.dot(ref_frame_inverse_transform, object_transform_matrix)

    # Extract transformed position and quaternion
    transformed_position = object_to_ref_frame_transform[:3, 3]
    transformed_quaternion = R.from_matrix(object_to_ref_frame_transform[:3, :3]).as_quat()

    # Create Pose message for the transformed pose
    transformed_pose = Pose()
    transformed_pose.position = Point(*transformed_position)
    transformed_pose.orientation = Quaternion(*transformed_quaternion)

    return transformed_pose
