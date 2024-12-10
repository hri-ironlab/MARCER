import rospy
from scene_graph.srv import (
    QuerySceneGraph,
    QuerySceneGraphRequest,
    QuerySceneGraphResponse,
)


class SceneGraphQuery:
    def __init__(self):
        rospy.init_node("scene_graph_query")
        self.scene_graph_service = rospy.ServiceProxy(
            "/scene_graph/query", QuerySceneGraph
        )

    def query_scene_graph(self):
        # Wait for the service to become available
        try:
            query = QuerySceneGraphRequest()
            query.node_name = "sponge"
            query.relationship_type = "is_on"
            query.attribute_name = ""

            # Call the service
            res = self.scene_graph_service(query)
            print("Relationship:", res.related_nodes)
            print("Attributes:", res.attributes)
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    scene_graph_query = SceneGraphQuery()
    scene_graph_query.query_scene_graph()
    scene_graph_query.run()
