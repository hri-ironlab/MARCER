#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
from vicon_receiver.vicon_data_receiver import ViconDataReceiver
from nav_msgs.msg import Odometry 

class ViconDataPublisher:
    def __init__(self):
        rospy.init_node("vicon_data_publisher")

        # Initialize ROS publisher
        self.model_states_pub = rospy.Publisher("/gazebo/model_states", ModelStates, queue_size=1)
        self.odometry_pub = rospy.Publisher("/vicon/odometry", Odometry, queue_size=1)
        self.unity_model_states_pub = rospy.Publisher("/unity/model_states", ModelStates, queue_size=1)
        # Initialize Vicon data receiver
        self.vicon_data_receiver = ViconDataReceiver(ip='192.168.1.7', port=12345, iter_vicon=10, packet_size=40960)

        # Initialize model states and reference frame pose
        self.model_states = ModelStates()
        self.unity_model_states = ModelStates()
        self.odometry = Odometry()

    def get_vicon_data(self):
        self.odometry, self.model_states = self.vicon_data_receiver.get_data("base_link", "base_link", ["QRCode"])
        _, self.unity_model_states = self.vicon_data_receiver.get_data("QRCode")

    def publish(self):
        # Publish the transformed model states
        self.model_states_pub.publish(self.model_states)
        self.odometry_pub.publish(self.odometry)
        self.unity_model_states_pub.publish(self.unity_model_states)

    # def shutdown_receiver(self):
    #     self.vicon_data_receiver.s.close()

if __name__ == "__main__":
    # Initialize ModelStatesPublisher instance
    vicon_data_publisher = ViconDataPublisher()

    # Set the loop rate
    rate = rospy.Rate(10)

    # Main loop
    # rospy.on_shutdown(vicon_data_publisher.shutdown_receiver)
    while not rospy.is_shutdown():
        vicon_data_publisher.get_vicon_data()
        vicon_data_publisher.publish()
        rate.sleep()
