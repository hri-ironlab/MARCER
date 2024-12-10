#!/usr/bin/env python3

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import time

if __name__ == "__main__":
    rospy.init_node("ready_pose")

    # Create move group interface for a fetch robot
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    # Create planning scene interface
    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.getKnownCollisionObjects()
    # Set collision checking for attached objects
    move_group.setPlanningTime(5)  # Set planning time limit (seconds)

    # TF joint names
    joint_names = [
        "torso_lift_joint",
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "upperarm_roll_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint",
    ]
    # Lists of joint angles in the same order as in joint_names
    home_pose = [[0.0, 1.1266, -0.4946, -2.9679, 1.3058, 1.267, 1.2769, 3.1415]]

    time.sleep(5)

    for pose in home_pose:
        if rospy.is_shutdown():
            break

        # Plans the joints in joint_names to angles in pose
        move_group.moveToJointPosition(joint_names, pose, wait=False)

        # Since we passed in wait=False above we need to wait here
        move_group.get_move_action().wait_for_result()
        result = move_group.get_move_action().get_result()

        if result:
            # Checking the MoveItErrorCode
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Ready Pose!")
            else:
                # If you get to this point please search for:
                # moveit_msgs/MoveItErrorCodes.msg
                rospy.logerr(
                    "Arm goal in state: %s", move_group.get_move_action().get_state()
                )
        else:
            rospy.logerr("MoveIt! failure no result returned.")

    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    move_group.get_move_action().cancel_all_goals()
