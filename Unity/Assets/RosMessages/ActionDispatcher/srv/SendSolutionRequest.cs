//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.ActionDispatcher
{
    [Serializable]
    public class SendSolutionRequest : Message
    {
        public const string k_RosMessageName = "action_dispatcher/SendSolution";
        public override string RosMessageName => k_RosMessageName;

        public DisplaySolutionMsg solution;

        public SendSolutionRequest()
        {
            this.solution = new DisplaySolutionMsg();
        }

        public SendSolutionRequest(DisplaySolutionMsg solution)
        {
            this.solution = solution;
        }

        public static SendSolutionRequest Deserialize(MessageDeserializer deserializer) => new SendSolutionRequest(deserializer);

        private SendSolutionRequest(MessageDeserializer deserializer)
        {
            this.solution = DisplaySolutionMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.solution);
        }

        public override string ToString()
        {
            return "SendSolutionRequest: " +
            "\nsolution: " + solution.ToString();
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