//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.RuleMonitor
{
    [Serializable]
    public class GetRuleDescriptionResponse : Message
    {
        public const string k_RosMessageName = "rule_monitor/GetRuleDescription";
        public override string RosMessageName => k_RosMessageName;

        public string description;

        public GetRuleDescriptionResponse()
        {
            this.description = "";
        }

        public GetRuleDescriptionResponse(string description)
        {
            this.description = description;
        }

        public static GetRuleDescriptionResponse Deserialize(MessageDeserializer deserializer) => new GetRuleDescriptionResponse(deserializer);

        private GetRuleDescriptionResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.description);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.description);
        }

        public override string ToString()
        {
            return "GetRuleDescriptionResponse: " +
            "\ndescription: " + description.ToString();
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
