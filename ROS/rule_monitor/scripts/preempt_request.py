#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger

def send_goal():
    # Initialize the node
    rospy.init_node('preempt_request')

    stop_rule_client = rospy.ServiceProxy("rule_monitor/stop", Trigger)
    response = stop_rule_client.call()
    rospy.loginfo("Response: %s", response)

if __name__ == '__main__':
    try:
        send_goal()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion")