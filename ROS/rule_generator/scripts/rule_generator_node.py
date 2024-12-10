#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from rule_generator.rule_generator import RuleGenerator
from rule_generator.srv import GetUserApproval
from rule_generator.msg import UserFeedback
from rule_monitor.srv import UpdateRule
from rule_generator.msg import RuleGeneratorState
from language_model.srv import QueryLanguageModel

class RuleGeneratorNode:
    def __init__(self):
        rospy.init_node("rule_generator_node")

        self.rule_generator = RuleGenerator()

        # Publishers
        self.rule_generator.display_current_command_publisher = rospy.Publisher(
            "rule_generator/current_command", UserFeedback, queue_size=10
        )
        
        self.rule_generator.rule_generator_state_publisher = rospy.Publisher(
            "rule_generator/state", RuleGeneratorState, queue_size=10
        )

        # Clients
        self.rule_generator.update_rule_service = rospy.ServiceProxy(
            "rule_monitor/update_rule", UpdateRule
        )

        self.rule_generator.language_model_service = rospy.ServiceProxy(
            "language_model/query_language_model", QueryLanguageModel
        )
        
        self.rule_generator.get_user_approval_client = rospy.ServiceProxy(
            "user_interface/get_user_approval", GetUserApproval
        )

        # Subscribers
        rospy.Subscriber(
            "user_input/speech", String, self.rule_generator.handle_speech_input
        )

if __name__ == "__main__":

    rule_generator_node = RuleGeneratorNode()
    rospy.sleep(3)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rule_generator_node.rule_generator.run_state_machine()
        rate.sleep()