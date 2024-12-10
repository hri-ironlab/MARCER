#!/usr/bin/env python3

import yaml
import rospy
from gazebo_msgs.msg import ModelStates
from vision_msgs.msg import (
    Detection3DArray,
    Detection3D,
    ObjectHypothesisWithPose,
    BoundingBox3D,
    VisionInfo,
)
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningSceneWorld
from shape_msgs.msg import SolidPrimitive
from perception.srv import GetDetectionClasses, GetDetectionClassesResponse


class DetectableObjectBuilder:
    def __init__(self, id, frame_id, size):
        self.detection = Detection3D()
        self.detection.header.frame_id = frame_id
        result = ObjectHypothesisWithPose()
        result.id = id
        self.detection.results.append(result)
        self.detection.bbox.center = Pose()
        self.detection.bbox.center.orientation.w = 1
        self.detection.bbox.size.x = size[0]
        self.detection.bbox.size.y = size[1]
        self.detection.bbox.size.z = size[2]

class PerceptionNode:
    def __init__(self):
        rospy.init_node("gazebo_perception_node")

        # Get the model_states_file_path from launch parameters
        self.file_path = rospy.get_param("~detectable_objects_file_path", None)
        if self.file_path is None:
            rospy.logerr(
                "Detectable objects file path not provided. Please specify the parameter '~detectable_objects_file_path'."
            )

        self.model_states_topic = rospy.get_param(
            "~model_states_topic", "/gazebo/model_states"
        )
        if self.model_states_topic is None:
            rospy.logerr(
                "Model states topic not provided. Please specify the parameter '~model_states_topic'."
            )

        self.model_states = ModelStates()
        self.object_dict = {}

        self.isSceneInitialized = False

        rospy.Subscriber(
            self.model_states_topic, ModelStates, self.model_states_callback
        )

        self.vision_info_pub = rospy.Publisher(
            "/perception/vision_info", VisionInfo, queue_size=1
        )
        self.object_detections_pub = rospy.Publisher(
            "/perception/object_detections", Detection3DArray, queue_size=1
        )

        self.manipulation_plan_service = rospy.Service(
            "/perception/get_detection_classes",
            GetDetectionClasses,
            self.get_detection_classes,
        )

    def model_states_callback(self, msg):
        self.model_states = msg

    def object_detections_publisher(self):
        detections = Detection3DArray()
        
        for i, name in enumerate(self.model_states.name):
            gazebo_object = self.object_dict.get(name)
            if gazebo_object:
                gazebo_object.detection.bbox.center = self.model_states.pose[i]
                detections.detections.append(gazebo_object.detection)
        self.object_detections_pub.publish(detections)

    def vision_info_publisher(self):
        vision_info = VisionInfo()
        vision_info.method = "parameter server"
        vision_info.database_location = "/vision_info_lookup"
        self.vision_info_pub.publish(vision_info)

    def get_detection_classes(self, req):
        return GetDetectionClassesResponse(self.object_dict.keys())

    def read_yaml_file(self):
        try:
            # Open the YAML file for reading
            with open(self.file_path, "r") as file:
                # Load the YAML data
                yaml_data = yaml.safe_load(file)

                objects_info = []

                # Iterate through each object in the YAML data
                for item in yaml_data:
                    # Extract relevant attributes
                    name = item["name"]
                    id = item["id"]
                    frame_id = item["frame_id"]
                    size = item["size"]
                    # Add object to dictionary
                    object_builder = DetectableObjectBuilder(id, frame_id, size)
                    self.object_dict[name] = object_builder
                    objects_info.append(name)

                rospy.set_param("/vision_info_lookup", objects_info)

        except Exception as e:
            print(f"Error reading YAML file: {e}")
            return None


if __name__ == "__main__":
    perception_node = PerceptionNode()
    perception_node.read_yaml_file()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        perception_node.object_detections_publisher()
        perception_node.vision_info_publisher()
        rate.sleep()
