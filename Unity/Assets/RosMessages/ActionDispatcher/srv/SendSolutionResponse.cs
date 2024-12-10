//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.ActionDispatcher
{
    [Serializable]
    public class SendSolutionResponse : Message
    {
        public const string k_RosMessageName = "action_dispatcher/SendSolution";
        public override string RosMessageName => k_RosMessageName;

        public bool success;

        public SendSolutionResponse()
        {
            this.success = false;
        }

        public SendSolutionResponse(bool success)
        {
            this.success = success;
        }

        public static SendSolutionResponse Deserialize(MessageDeserializer deserializer) => new SendSolutionResponse(deserializer);

        private SendSolutionResponse(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.success);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.success);
        }

        public override string ToString()
        {
            return "SendSolutionResponse: " +
            "\nsuccess: " + success.ToString();
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