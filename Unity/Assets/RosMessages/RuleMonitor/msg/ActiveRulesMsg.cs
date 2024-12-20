//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.RuleMonitor
{
    [Serializable]
    public class ActiveRulesMsg : Message
    {
        public const string k_RosMessageName = "rule_monitor/ActiveRules";
        public override string RosMessageName => k_RosMessageName;

        public RuleMsg[] rules;
        public sbyte executing_rule_index;

        public ActiveRulesMsg()
        {
            this.rules = new RuleMsg[0];
            this.executing_rule_index = 0;
        }

        public ActiveRulesMsg(RuleMsg[] rules, sbyte executing_rule_index)
        {
            this.rules = rules;
            this.executing_rule_index = executing_rule_index;
        }

        public static ActiveRulesMsg Deserialize(MessageDeserializer deserializer) => new ActiveRulesMsg(deserializer);

        private ActiveRulesMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.rules, RuleMsg.Deserialize, deserializer.ReadLength());
            deserializer.Read(out this.executing_rule_index);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.rules);
            serializer.Write(this.rules);
            serializer.Write(this.executing_rule_index);
        }

        public override string ToString()
        {
            return "ActiveRulesMsg: " +
            "\nrules: " + System.String.Join(", ", rules.ToList()) +
            "\nexecuting_rule_index: " + executing_rule_index.ToString();
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
