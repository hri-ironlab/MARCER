//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.RuleGenerator
{
    [Serializable]
    public class GetUserApprovalRequest : Message
    {
        public const string k_RosMessageName = "rule_generator/GetUserApproval";
        public override string RosMessageName => k_RosMessageName;

        public string plan_description;
        public string generated_function;

        public GetUserApprovalRequest()
        {
            this.plan_description = "";
            this.generated_function = "";
        }

        public GetUserApprovalRequest(string plan_description, string generated_function)
        {
            this.plan_description = plan_description;
            this.generated_function = generated_function;
        }

        public static GetUserApprovalRequest Deserialize(MessageDeserializer deserializer) => new GetUserApprovalRequest(deserializer);

        private GetUserApprovalRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.plan_description);
            deserializer.Read(out this.generated_function);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.plan_description);
            serializer.Write(this.generated_function);
        }

        public override string ToString()
        {
            return "GetUserApprovalRequest: " +
            "\nplan_description: " + plan_description.ToString() +
            "\ngenerated_function: " + generated_function.ToString();
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
