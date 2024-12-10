#!/usr/bin/env python3

import yaml
import rospy
import rospkg
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def euler_to_quaternion(roll, pitch, yaw):
    quat = quaternion_from_euler(roll, pitch, yaw)
    return quat


class ModelStatesPublisher:
    def __init__(self):
        rospy.init_node("model_states_publisher")

        self.model_states = ModelStates()
        self.model_states_pub = rospy.Publisher(
            "/gazebo/model_states", ModelStates, queue_size=1
        )

    def model_states_publisher(self):
        self.model_states_pub.publish(self.model_states)

    def read_yaml_file(self, file_path):
        try:
            # Open the YAML file for reading
            with open(file_path, "r") as file:
                # Load the YAML data
                yaml_data = yaml.safe_load(file)

                # Iterate through each object in the YAML data
                for item in yaml_data:
                    # Extract relevant attributes
                    name = item["name"]
                    position = item["position"]
                    
                    if not name:
                        rospy.logerr(f"[simulator] Empty object name detected.")

                    if len(position) < 6:
                        rospy.logerr(f"[simulator] Error reading pose for {name}")
                        continue

                    pose = Pose()
                    pose.position.x = float(position[0])
                    pose.position.y = float(position[1])
                    pose.position.z = float(position[2])

                    # Accommodate quaternion or euler
                    if len(position) == 7:
                        pose.orientation.x = float(position[3])
                        pose.orientation.y = float(position[4])
                        pose.orientation.z = float(position[5])
                        pose.orientation.w = float(position[6])
                    else:
                        quat_position = euler_to_quaternion(
                            position[3], position[4], position[5]
                        )
                        pose.orientation.x = float(quat_position[0])
                        pose.orientation.y = float(quat_position[1])
                        pose.orientation.z = float(quat_position[2])
                        pose.orientation.w = float(quat_position[3])

                    self.model_states.name.append(name)
                    self.model_states.pose.append(pose)
                    self.model_states.twist.append(Twist())

        except Exception as e:
            print(f"Error reading YAML file: {e}")
            return None


if __name__ == "__main__":
    model_states_publisher = ModelStatesPublisher()

    # Get the model_states_file_path from launch parameters
    model_states_file_path = rospy.get_param('~model_states_file_path', None)

    if model_states_file_path is None:
        rospy.logerr("Model states file path not provided. Please specify the parameter '~model_states_file_path'.")
    else:
        model_states_publisher.read_yaml_file(model_states_file_path)

    rospy.sleep(3)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        model_states_publisher.model_states_publisher()
        rate.sleep()
