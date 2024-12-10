from dataclasses import dataclass, field
from scene_graph.srv import QuerySceneGraph, QuerySceneGraphRequest, QuerySceneGraphResponse
from world_monitor.srv import GetAttachedObject, GetAttachedObjectRequest, GetAttachedObjectResponse
import rospy
from typing import List

@dataclass
class Context:
    mode: str = ''
    task: str = ''
    object_names: List[str] = field(default_factory=list)
    locations: List[str] = field(default_factory=list)
    actions: List[str] = field(default_factory=list)
    task_feedback: str = ''
    object_locations: List[str] = field(default_factory=list)  # Now a list of strings
    attached_object: str = ''
    condition: str = ''
    action_plan: List[str] = field(default_factory=list)

class ContextBuilder:
    def __init__(self):
        self.scene_graph_query_client = rospy.ServiceProxy(
            "/scene_graph/query", QuerySceneGraph
        )

        self.get_attached_object_client = rospy.ServiceProxy(
            "/world_monitor/get_attached_object", GetAttachedObject
        )

    def get_context(self) -> Context:
        locations = self.query_scene_graph("", "surface")
        object_names = self.query_scene_graph("", "object")
        attached_object = self.get_attached_object()
        object_locations = self.get_object_locations(locations)

        return Context(
            object_names=object_names,
            locations=locations,
            attached_object=attached_object,
            object_locations=object_locations
        )

    def get_attached_object(self) -> str:
        query = GetAttachedObjectRequest()
        res = self.get_attached_object_client.call(query)
        return res.attached_object_name

    def query_scene_graph(self, relationship_type: str, attribute_name: str) -> List[str]:
        # Construct the query with the provided attribute name
        if relationship_type:
            query = QuerySceneGraphRequest(attribute_name, relationship_type, "")
        else:
            query = QuerySceneGraphRequest("", "", attribute_name)
        
        # Call the scene graph query service with the constructed query
        res = self.scene_graph_query_client.call(query)
        return res.related_nodes

    def get_object_locations(self, locations: List[str]) -> List[str]:
        object_locations = []

        for location in locations:
            query_type = "has_inside" if location.lower().startswith("zone") else "supports"
            associated_objects = self.query_scene_graph(query_type, location)

            for object_name in associated_objects:
                # Create a description for the object and location
                description = f"{object_name} is {'in' if query_type == 'has_inside' else 'on the'} {location}"
                object_locations.append(description)
        
        return object_locations
