import rospy
from scene_graph.srv import (
    QuerySceneGraph,
    QuerySceneGraphRequest,
    QuerySceneGraphResponse,
)
import re 

class Conditions:
    def __init__(self):
        self.scene_graph_client = rospy.ServiceProxy(
            "/scene_graph/query", QuerySceneGraph
        )
        self.scene_graph_client.wait_for_service()

        self.all_scene_objects = []
        self.all_locations = []
        self.function_times = {}

        self.initialize()
        self.time_tracking_dict = {}

    def check_location(self, scene_object, location):
        return self.time_tracking_dict.get((scene_object, location), -1) != -1

    def check_function_time(self, condition, function):
        elapsed_time = int(condition.split()[0])
        tracked_time = self.function_times.get(function, -1)
        return tracked_time != -1 and rospy.Time.now().to_sec() - tracked_time >= elapsed_time

    def check_object_time(self, condition, scene_object, location):
        elapsed_time = int(condition.split()[-2])
        tracked_time = self.time_tracking_dict.get((scene_object, location), -1)
        return tracked_time != -1 and rospy.Time.now().to_sec() - tracked_time >= elapsed_time

    def add_function_timer(self, function_name):
        self.function_times[function_name] = rospy.Time.now().to_sec()

    def update_tracked_times(self):
        self.get_locations()

        # go through all scene objects
        for scene_object in self.all_scene_objects:
            # get all locations related to scene object
            related_locations = self.query_scene_graph("", "get_all_locations", scene_object).related_nodes

            # Update time tracking for all known locations
            for loc in self.all_locations:
                if loc in related_locations:
                    # Update time only if it's currently -1
                    if self.time_tracking_dict.get((scene_object, loc), -1) == -1:
                        self.time_tracking_dict[(scene_object, loc)] = rospy.Time.now().to_sec()
                else:
                    # Reset time to -1 for unoccupied locations
                    self.time_tracking_dict[(scene_object, loc)] = -1

    def initialize(self):
        self.all_scene_objects = self.query_scene_graph("", "object", "").related_nodes
        self.get_locations()

    def get_locations(self):
        response = self.query_scene_graph("", "surface", "")
        self.all_locations = response.related_nodes
        self.all_locations.append("No Location")

    def query_scene_graph(self, relationship_type, attribute_type, node_name):
        query = QuerySceneGraphRequest()
        query.relationship_type = relationship_type
        query.attribute_name = attribute_type
        query.node_name = node_name
        res = self.scene_graph_client.call(query)
        return res