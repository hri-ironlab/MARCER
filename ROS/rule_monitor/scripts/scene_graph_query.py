#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger
from scene_graph.srv import QuerySceneGraph, QuerySceneGraphRequest, QuerySceneGraphResponse
from rule_monitor.conditions import Conditions

if __name__ == '__main__':
    conditions = Conditions()
    conditions.initialize()
    while not rospy.is_shutdown():
        conditions.update_tracking()
        rospy.sleep(1)