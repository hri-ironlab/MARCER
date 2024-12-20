//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Perception
{
    [Serializable]
    public class GetDetectionClassesResponse : Message
    {
        public const string k_RosMessageName = "perception/GetDetectionClasses";
        public override string RosMessageName => k_RosMessageName;

        public string[] detection_classes;

        public GetDetectionClassesResponse()
        {
            this.detection_classes = new string[0];
        }

        public GetDetectionClassesResponse(string[] detection_classes)
        {
            this.detection_classes = detection_classes;
        }

        public static GetDetectionClassesResponse Deserialize(MessageDeserializer deserializer) => new GetDetectionClassesResponse(deserializer);

        private GetDetectionClassesResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.detection_classes, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.detection_classes);
            serializer.Write(this.detection_classes);
        }

        public override string ToString()
        {
            return "GetDetectionClassesResponse: " +
            "\ndetection_classes: " + System.String.Join(", ", detection_classes.ToList());
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
