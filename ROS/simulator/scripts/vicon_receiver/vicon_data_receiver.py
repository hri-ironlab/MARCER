#!/usr/bin/env python3

import socket
import json
import numpy as np
from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import PoseStamped, Pose
import math
import time
from vicon_receiver.transform_helpers import transform_to_parent_frame
from nav_msgs.msg import Odometry
import copy
class ViconDataReceiver:
    def __init__(self, ip, port, iter_vicon, packet_size):
        self.ip = ip
        self.port = port
        self.iter_vicon = iter_vicon
        self.packet_size = packet_size
        self.previous_parent_pose = Pose()
        self.previous_model_states = {}
        self.previous_odometry = Odometry()
        self.odometry_start_pose = Pose()
        # self.s = socket.socket()
        # self.s.connect((self.ip, self.port))
        
    def connect_to_vicon_server(self):
        s = socket.socket()
        try:
            s.connect((self.ip, self.port))
            packet = s.recv(self.packet_size).decode("utf-8")
            s.close()
            return packet
        except Exception as e:
            print("VICON error: ")
            raise e

    def parse_vicon_data(self, packet):
        # print(packet)
        # print("package size: ", self.packet_size)
        vicon_data = json.loads(packet)

        object_positions = {}
        object_orientations = {}
        object_names = list(vicon_data.keys())
        for obj_name in object_names:
            obj_inst = vicon_data[obj_name][obj_name]
            object_positions[obj_name] = np.array(obj_inst["global_tx"][0])
            object_orientations[obj_name] = np.array(obj_inst["global_rot"][0])
        return object_names, object_positions, object_orientations

    def average_data(self, data_dict):
        for obj_name, data in data_dict.items():
            data_dict[obj_name] = np.mean(data, axis=0)
        return data_dict

    def normalize_quaternion(self, quaternion):
        x, y, z, w = quaternion
        magnitude = math.sqrt(x ** 2 + y ** 2 + z ** 2 + w ** 2)
        return x / magnitude, y / magnitude, z / magnitude, w / magnitude

    def get_data(self, parent_name, odometry_name="", disregard_objects=[]):
        object_names = None
        object_positions = {}
        object_orientations = {}
        model_states = ModelStates()
        odometry = Odometry()
        parent_pose = Pose()

        for _ in range(self.iter_vicon):
            while True:
                try:
                    packet = self.connect_to_vicon_server()
                    names, locations, orientations = self.parse_vicon_data(packet)
                    break
                except json.decoder.JSONDecodeError:
                    print("json.decoder.JSONDecodeError")
            if names is {}:
                return odometry, model_states
            if object_names is None:
                object_names = names
            for obj_name in object_names:
                object_positions.setdefault(obj_name, []).append(locations[obj_name])
                object_orientations.setdefault(obj_name, []).append(orientations[obj_name])

        object_positions = self.average_data(object_positions)
        object_orientations = self.average_data(object_orientations)


        if object_orientations[parent_name][3] == 0.0:
            parent_pose = self.previous_parent_pose
        else:
            parent_pose.position.x = object_positions[parent_name][0] / 1000.0 
            parent_pose.position.y = object_positions[parent_name][1] / 1000.0 
            parent_pose.position.z = object_positions[parent_name][2] / 1000.0 

            if parent_name == "base_link":
                parent_pose.position.z = 0.0 

            norm_quat = self.normalize_quaternion(object_orientations[parent_name])
            parent_pose.orientation.x = norm_quat[0]
            parent_pose.orientation.y = norm_quat[1]
            parent_pose.orientation.z = norm_quat[2]
            parent_pose.orientation.w = norm_quat[3]
            self.previous_parent_pose = parent_pose


        if odometry_name != "":
            if self.odometry_start_pose == Pose():
                self.odometry_start_pose = parent_pose

            odometry.header.frame_id = parent_name
            
            odometry.pose.pose = transform_to_parent_frame(parent_pose, self.odometry_start_pose)
            
        for object_name in object_names:
            if object_name in disregard_objects or object_name == parent_name:
                continue

            if object_orientations[object_name][3] == 0.0:
                model_states.name.append(object_name)
                model_states.pose.append(self.previous_model_states[object_name])
                continue 

            object_pose = Pose()
            object_pose.position.x = object_positions[object_name][0] / 1000.0 
            object_pose.position.y = object_positions[object_name][1] / 1000.0 
            object_pose.position.z = object_positions[object_name][2] / 1000.0 
            if object_name == "base_link":
                object_pose.position.z = 0.0 
            norm_quat = self.normalize_quaternion(object_orientations[object_name])
            object_pose.orientation.x = norm_quat[0]
            object_pose.orientation.y = norm_quat[1]
            object_pose.orientation.z = norm_quat[2]
            object_pose.orientation.w = norm_quat[3]

            transformed_pose = transform_to_parent_frame(object_pose, parent_pose)
            model_states.name.append(object_name)
            model_states.pose.append(transformed_pose)
            if object_name == "middle shelf":
                temp = copy.deepcopy(transformed_pose)
                temp.position.z -= 0.4
                model_states.name.append("bottom shelf")
                model_states.pose.append(temp)
                model_states.name.append("collisions")
                model_states.pose.append(temp)
            self.previous_model_states[object_name] = transformed_pose
        return odometry, model_states
    
if __name__ == '__main__':
    vicon_receiver = ViconDataReceiver(ip='192.168.1.12', port=12345, iter_vicon=10, packet_size=4096, object_reference_frame_name="base_link")
    while True:
        object_model_states = vicon_receiver.get_data()
        print(object_model_states)
        print("\n")
        time.sleep(1)