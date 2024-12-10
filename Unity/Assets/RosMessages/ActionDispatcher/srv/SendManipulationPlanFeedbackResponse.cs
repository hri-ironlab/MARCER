//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.ActionDispatcher
{
    [Serializable]
    public class SendManipulationPlanFeedbackResponse : Message
    {
        public const string k_RosMessageName = "action_dispatcher/SendManipulationPlanFeedback";
        public override string RosMessageName => k_RosMessageName;


        public SendManipulationPlanFeedbackResponse()
        {
        }
        public static SendManipulationPlanFeedbackResponse Deserialize(MessageDeserializer deserializer) => new SendManipulationPlanFeedbackResponse(deserializer);

        private SendManipulationPlanFeedbackResponse(MessageDeserializer deserializer)
        {
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
        }

        public override string ToString()
        {
            return "SendManipulationPlanFeedbackResponse: ";
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
