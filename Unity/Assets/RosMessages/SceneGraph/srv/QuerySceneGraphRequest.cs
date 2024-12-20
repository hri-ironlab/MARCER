//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.SceneGraph
{
    [Serializable]
    public class QuerySceneGraphRequest : Message
    {
        public const string k_RosMessageName = "scene_graph/QuerySceneGraph";
        public override string RosMessageName => k_RosMessageName;

        //  Example: "table1", "sponge"
        public string node_name;
        //  These include "is_on" or "supports"
        public string relationship_type;
        //  "subframe_names"
        public string attribute_name;

        public QuerySceneGraphRequest()
        {
            this.node_name = "";
            this.relationship_type = "";
            this.attribute_name = "";
        }

        public QuerySceneGraphRequest(string node_name, string relationship_type, string attribute_name)
        {
            this.node_name = node_name;
            this.relationship_type = relationship_type;
            this.attribute_name = attribute_name;
        }

        public static QuerySceneGraphRequest Deserialize(MessageDeserializer deserializer) => new QuerySceneGraphRequest(deserializer);

        private QuerySceneGraphRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.node_name);
            deserializer.Read(out this.relationship_type);
            deserializer.Read(out this.attribute_name);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.node_name);
            serializer.Write(this.relationship_type);
            serializer.Write(this.attribute_name);
        }

        public override string ToString()
        {
            return "QuerySceneGraphRequest: " +
            "\nnode_name: " + node_name.ToString() +
            "\nrelationship_type: " + relationship_type.ToString() +
            "\nattribute_name: " + attribute_name.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
