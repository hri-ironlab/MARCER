//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Gazebo
{
    [Serializable]
    public class GetModelStateResponse : Message
    {
        public const string k_RosMessageName = "gazebo_msgs/GetModelState";
        public override string RosMessageName => k_RosMessageName;

        public HeaderMsg header;
        //  Standard metadata for higher-level stamped data types.
        //  * header.seq holds the number of requests since the plugin started
        //  * header.stamp timestamp related to the pose
        //  * header.frame_id not used but currently filled with the relative_entity_name
        public Geometry.PoseMsg pose;
        //  pose of model in relative entity frame
        public Geometry.TwistMsg twist;
        //  twist of model in relative entity frame
        public bool success;
        //  return true if get successful
        public string status_message;
        //  comments if available

        public GetModelStateResponse()
        {
            this.header = new HeaderMsg();
            this.pose = new Geometry.PoseMsg();
            this.twist = new Geometry.TwistMsg();
            this.success = false;
            this.status_message = "";
        }

        public GetModelStateResponse(HeaderMsg header, Geometry.PoseMsg pose, Geometry.TwistMsg twist, bool success, string status_message)
        {
            this.header = header;
            this.pose = pose;
            this.twist = twist;
            this.success = success;
            this.status_message = status_message;
        }

        public static GetModelStateResponse Deserialize(MessageDeserializer deserializer) => new GetModelStateResponse(deserializer);

        private GetModelStateResponse(MessageDeserializer deserializer)
        {
            this.header = HeaderMsg.Deserialize(deserializer);
            this.pose = Geometry.PoseMsg.Deserialize(deserializer);
            this.twist = Geometry.TwistMsg.Deserialize(deserializer);
            deserializer.Read(out this.success);
            deserializer.Read(out this.status_message);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.pose);
            serializer.Write(this.twist);
            serializer.Write(this.success);
            serializer.Write(this.status_message);
        }

        public override string ToString()
        {
            return "GetModelStateResponse: " +
            "\nheader: " + header.ToString() +
            "\npose: " + pose.ToString() +
            "\ntwist: " + twist.ToString() +
            "\nsuccess: " + success.ToString() +
            "\nstatus_message: " + status_message.ToString();
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