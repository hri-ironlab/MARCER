#!/usr/bin/env python3

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import time
from geometry_msgs.msg import Pose

if __name__ == "__main__":
    rospy.init_node("stop_tuck_or_ready")

    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    joint_names = []
    pose = Pose()
    move_group.moveToJointPosition(joint_names, pose, wait=False)
