#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, SetBoolResponse
from scene_graph.srv import (
    QuerySceneGraph,
    QuerySceneGraphRequest,
    QuerySceneGraphResponse,
)
from rule_monitor.msg import Rule, TriggerType
from rule_monitor.srv import UpdateRule, UpdateRuleRequest, UpdateRuleResponse
from rule_monitor.srv import GetRuleDescription, GetRuleDescriptionRequest, GetRuleDescriptionResponse
from rule_monitor.rule_monitor import RuleMonitor
import actionlib
from action_dispatcher.msg import DispatchActionsAction
from std_srvs.srv import Trigger
from rule_monitor.msg import ActiveRules
from language_model.srv import QueryLanguageModel
from rule_monitor.srv import SendActiveRules

class RuleMonitorNode:
    def __init__(self):
        rospy.init_node("rule_monitor_node")

        self.rule_monitor = RuleMonitor()

        # Puli

        # Services
        rospy.Service(
            "rule_monitor/update_rule",
            UpdateRule,
            self.rule_monitor.handle_update_rule_request,
        )

        rospy.Service(
            "rule_monitor/pause_rule_monitor",
            Trigger,
            self.rule_monitor.handle_pause_rule_monitor,
        )


        rospy.Service(
            "rule_monitor/get_rule_description",
            GetRuleDescription,
            self.rule_monitor.handle_get_rule_description,
        )

        # Clients
        self.rule_monitor.scene_graph_client = rospy.ServiceProxy(
            "scene_graph/query", QuerySceneGraph
        )
        
        rospy.wait_for_service("rule_monitor/send_active_rules")

        self.rule_monitor.send_active_rules = rospy.ServiceProxy(
            "rule_monitor/send_active_rules", SendActiveRules
        )

        self.rule_monitor.action_dispatcher_client = actionlib.SimpleActionClient(
            "action_dispatcher/action_queue", DispatchActionsAction
        )

        self.rule_monitor.active_rules_publisher = rospy.Publisher(
            "rule_monitor/active_rules", ActiveRules, queue_size=1
        )

        rospy.Service(
            "rule_monitor/stop",
            Trigger,
            self.rule_monitor.handle_stop_request,
        )

        self.rule_monitor.language_model_service = rospy.ServiceProxy(
            "language_model/query_language_model", QueryLanguageModel
        )

        # rospy.Subscriber(
        #     "unity/zone_delete", String, self.rule_monitor.handle_delete_zone
        # )


if __name__ == "__main__":

    rule_monitor_node = RuleMonitorNode()
    rospy.sleep(3)
    rule_monitor_node.rule_monitor.load_rules()
    rate = rospy.Rate(5)  # 10hz
    while not rospy.is_shutdown():
        rule_monitor_node.rule_monitor.run_rules()
        # rule_monitor_node.rule_monitor.publish_active_rules()
        rule_monitor_node.rule_monitor.conditions.update_tracked_times()
        rate.sleep()
