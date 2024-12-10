//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.SceneGraph
{
    [Serializable]
    public class QuerySceneGraphResponse : Message
    {
        public const string k_RosMessageName = "scene_graph/QuerySceneGraph";
        public override string RosMessageName => k_RosMessageName;

        //  List of objects and object "is_on" or "supports"
        public string[] related_nodes;
        //  Used to list subframes for placing, TBD: Expand to more attributes
        public string[] attributes;

        public QuerySceneGraphResponse()
        {
            this.related_nodes = new string[0];
            this.attributes = new string[0];
        }

        public QuerySceneGraphResponse(string[] related_nodes, string[] attributes)
        {
            this.related_nodes = related_nodes;
            this.attributes = attributes;
        }

        public static QuerySceneGraphResponse Deserialize(MessageDeserializer deserializer) => new QuerySceneGraphResponse(deserializer);

        private QuerySceneGraphResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.related_nodes, deserializer.ReadLength());
            deserializer.Read(out this.attributes, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.related_nodes);
            serializer.Write(this.related_nodes);
            serializer.WriteLength(this.attributes);
            serializer.Write(this.attributes);
        }

        public override string ToString()
        {
            return "QuerySceneGraphResponse: " +
            "\nrelated_nodes: " + System.String.Join(", ", related_nodes.ToList()) +
            "\nattributes: " + System.String.Join(", ", attributes.ToList());
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}
