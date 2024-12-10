#!/usr/bin/env python
import rospy
import actionlib
from action_dispatcher.action_dispatcher import ActionDispatcher
from action_dispatcher.msg import (
    DispatchActionsAction,
    DispatchActionsFeedback,
    DispatchActionsGoal,
    DispatchActionsResult,
)
from geometry_msgs.msg import Pose

from manipulation.msg import ManipulationPlanRequest

def build_action_request(task_type, target_object_name="", description="", place_pose=Pose()) -> ManipulationPlanRequest:
    return ManipulationPlanRequest(
        task_type=task_type,
        target_object_name=target_object_name,
        task_name=description,
        place_pose=place_pose
    )

def send_goal():
    # Initialize the node
    rospy.init_node('action_dispatcher_client')

    # Create an action client
    client = actionlib.SimpleActionClient('action_dispatcher/action_queue', DispatchActionsAction)

    # Wait until the action server has started up and started listening for goals
    rospy.loginfo("Waiting for action server to start...")
    client.wait_for_server()

    # Create a goal to send to the action server
    action_queue = DispatchActionsGoal()
    action_queue.actions = []
    action_queue.actions.append(build_action_request(ManipulationPlanRequest.PICK, "Pringles", "Pick Object"))
    action_queue.actions.append(build_action_request(ManipulationPlanRequest.PLACE, "table", "Place Object"))

    client.send_goal(action_queue)

    rospy.sleep(5)

    print("Cancelling goal")
    client.cancel_goal()

    # Wait for the server to finish performing the action
    client.wait_for_result()

    # Print the result of executing the action
    result = client.get_result()
    rospy.loginfo("Result: %s", result)

if __name__ == '__main__':
    try:
        send_goal()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion")