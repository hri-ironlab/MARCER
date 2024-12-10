//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.MoveitMsgs
{
    [Serializable]
    public class MoveGroupFeedback : Message
    {
        public const string k_RosMessageName = "moveit_msgs-3175099a2b7fa75f74c36850a716b0fe603c8947/MoveGroup";
        public override string RosMessageName => k_RosMessageName;

        //  The internal state that the move group action currently is in
        public string state;

        public MoveGroupFeedback()
        {
            this.state = "";
        }

        public MoveGroupFeedback(string state)
        {
            this.state = state;
        }

        public static MoveGroupFeedback Deserialize(MessageDeserializer deserializer) => new MoveGroupFeedback(deserializer);

        private MoveGroupFeedback(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.state);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.state);
        }

        public override string ToString()
        {
            return "MoveGroupFeedback: " +
            "\nstate: " + state.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Feedback);
        }
    }
}