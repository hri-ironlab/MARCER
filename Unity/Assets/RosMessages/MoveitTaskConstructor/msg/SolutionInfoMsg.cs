//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.MoveitTaskConstructor
{
    [Serializable]
    public class SolutionInfoMsg : Message
    {
        public const string k_RosMessageName = "moveit_task_constructor_msgs/SolutionInfo";
        public override string RosMessageName => k_RosMessageName;

        //  unique id within task
        public uint id;
        //  associated cost
        public float cost;
        //  associated comment, usually providing failure hint
        public string comment;
        //  id of stage that created this trajectory
        public uint stage_id;
        //  markers, e.g. providing additional hints or illustrating failure
        public Visualization.MarkerMsg[] markers;

        public SolutionInfoMsg()
        {
            this.id = 0;
            this.cost = 0.0f;
            this.comment = "";
            this.stage_id = 0;
            this.markers = new Visualization.MarkerMsg[0];
        }

        public SolutionInfoMsg(uint id, float cost, string comment, uint stage_id, Visualization.MarkerMsg[] markers)
        {
            this.id = id;
            this.cost = cost;
            this.comment = comment;
            this.stage_id = stage_id;
            this.markers = markers;
        }

        public static SolutionInfoMsg Deserialize(MessageDeserializer deserializer) => new SolutionInfoMsg(deserializer);

        private SolutionInfoMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.id);
            deserializer.Read(out this.cost);
            deserializer.Read(out this.comment);
            deserializer.Read(out this.stage_id);
            deserializer.Read(out this.markers, Visualization.MarkerMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.id);
            serializer.Write(this.cost);
            serializer.Write(this.comment);
            serializer.Write(this.stage_id);
            serializer.WriteLength(this.markers);
            serializer.Write(this.markers);
        }

        public override string ToString()
        {
            return "SolutionInfoMsg: " +
            "\nid: " + id.ToString() +
            "\ncost: " + cost.ToString() +
            "\ncomment: " + comment.ToString() +
            "\nstage_id: " + stage_id.ToString() +
            "\nmarkers: " + System.String.Join(", ", markers.ToList());
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