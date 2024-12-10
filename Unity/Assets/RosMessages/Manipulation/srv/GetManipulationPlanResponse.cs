//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Manipulation
{
    [Serializable]
    public class GetManipulationPlanResponse : Message
    {
        public const string k_RosMessageName = "manipulation/GetManipulationPlan";
        public override string RosMessageName => k_RosMessageName;

        public ManipulationPlanResponseMsg manipulation_plan_response;

        public GetManipulationPlanResponse()
        {
            this.manipulation_plan_response = new ManipulationPlanResponseMsg();
        }

        public GetManipulationPlanResponse(ManipulationPlanResponseMsg manipulation_plan_response)
        {
            this.manipulation_plan_response = manipulation_plan_response;
        }

        public static GetManipulationPlanResponse Deserialize(MessageDeserializer deserializer) => new GetManipulationPlanResponse(deserializer);

        private GetManipulationPlanResponse(MessageDeserializer deserializer)
        {
            this.manipulation_plan_response = ManipulationPlanResponseMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.manipulation_plan_response);
        }

        public override string ToString()
        {
            return "GetManipulationPlanResponse: " +
            "\nmanipulation_plan_response: " + manipulation_plan_response.ToString();
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