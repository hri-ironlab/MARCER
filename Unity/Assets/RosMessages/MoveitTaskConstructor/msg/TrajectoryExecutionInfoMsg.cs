//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.MoveitTaskConstructor
{
    [Serializable]
    public class TrajectoryExecutionInfoMsg : Message
    {
        public const string k_RosMessageName = "moveit_task_constructor_msgs/TrajectoryExecutionInfo";
        public override string RosMessageName => k_RosMessageName;

        //  List of controllers to use when executing the trajectory
        public string[] controller_names;

        public TrajectoryExecutionInfoMsg()
        {
            this.controller_names = new string[0];
        }

        public TrajectoryExecutionInfoMsg(string[] controller_names)
        {
            this.controller_names = controller_names;
        }

        public static TrajectoryExecutionInfoMsg Deserialize(MessageDeserializer deserializer) => new TrajectoryExecutionInfoMsg(deserializer);

        private TrajectoryExecutionInfoMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.controller_names, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.controller_names);
            serializer.Write(this.controller_names);
        }

        public override string ToString()
        {
            return "TrajectoryExecutionInfoMsg: " +
            "\ncontroller_names: " + System.String.Join(", ", controller_names.ToList());
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