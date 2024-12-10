#!/usr/bin/env python3

import rospy
from actionlib.simple_action_client import SimpleActionClient
from manipulation.srv import GetManipulationPlan
from moveit_task_constructor_msgs.msg import ExecuteTaskSolutionAction, Solution
from actionlib import SimpleActionClient
from std_srvs.srv import Trigger
from action_dispatcher.action_dispatcher import ActionDispatcher
import actionlib
from action_dispatcher.msg import DispatchActionsAction, ActionDispatcherState
from action_dispatcher.msg import DisplaySolution
from moveit_task_constructor_msgs.msg import Solution
from action_dispatcher.srv import (
    SendSolution,
    SendActionState,
    SendManipulationPlanFeedback,
)
from moveit_msgs.srv import GetPlanningScene


class ActionDispatcherNode:
    def __init__(self):
        rospy.init_node("action_dispatcher_node")

        self.action_dispatcher = ActionDispatcher()

        # this does not get the whole action plan so switching to manipulation_node/solution
        # this will probably be unused
        self.action_dispatcher.get_manipulation_plan_client = rospy.ServiceProxy(
            "get_manipulation_plan", GetManipulationPlan
        )

        self.action_dispatcher.get_planning_scene = rospy.ServiceProxy(
            "/get_planning_scene", GetPlanningScene
        )

        self.action_dispatcher.send_solution = rospy.ServiceProxy(
            "action_dispatcher/send_solution", SendSolution
        )

        self.action_dispatcher.send_manipulation_plan_feedback = rospy.ServiceProxy(
            "action_dispatcher/send_manipulation_plan_feedback",
            SendManipulationPlanFeedback,
        )

        self.action_dispatcher.send_action_state = rospy.ServiceProxy(
            "action_dispatcher/send_state", SendActionState
        )

        self.action_dispatcher.execute_manipulation_plan_client = SimpleActionClient(
            "execute_task_solution", ExecuteTaskSolutionAction
        )

        self.action_dispatcher.solution_publisher = rospy.Publisher(
            "action_dispatcher/solution", Solution, queue_size=1
        )

        # self.action_dispatcher.action_dispatcher_state_publisher = rospy.Publisher(
        #     "action_dispatcher/state", ActionDispatcherState, queue_size=10
        # )

        self.action_dispatcher.publish_display_solution = rospy.Publisher(
            "action_dispatcher/display_solution", DisplaySolution, queue_size=1
        )

        rospy.Subscriber(
            "manipulation_node/solution",
            Solution,
            self.action_dispatcher.handle_solution,
        )

        self.action_dispatcher.action_dispatcher_server = actionlib.SimpleActionServer(
            "action_dispatcher/action_queue",
            DispatchActionsAction,
            execute_cb=self.action_dispatcher.dispatch_actions_callback,
            auto_start=False,
        )
        self.action_dispatcher.action_dispatcher_server.start()


if __name__ == "__main__":
    action_dispatcher_node = ActionDispatcherNode()

    rospy.sleep(3)

    rate = rospy.Rate(5)  # 10hz
    while not rospy.is_shutdown():
        rate.sleep()
