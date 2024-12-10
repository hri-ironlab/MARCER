#!/usr/bin/env python3

import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest

if __name__ == "__main__":
    rospy.init_node("stop_robot")
    stop_robot_client = rospy.ServiceProxy('/executive/stop_robot', Trigger)
    stop_robot_client.wait_for_service()
    response = stop_robot_client(TriggerRequest())
    if response.success:
        rospy.loginfo("Robot stopped successfully.")
    else:
        rospy.logerr("Failed to stop the robot.")