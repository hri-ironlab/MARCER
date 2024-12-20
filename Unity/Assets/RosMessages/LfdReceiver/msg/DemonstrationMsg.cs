//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.LfdReceiver
{
    [Serializable]
    public class DemonstrationMsg : Message
    {
        public const string k_RosMessageName = "lfd_receiver/Demonstration";
        public override string RosMessageName => k_RosMessageName;

        public HeaderMsg header;
        public Sensor.JointStateMsg joint_states;
        public Std.BoolMsg pick;
        public Std.BoolMsg place;

        public DemonstrationMsg()
        {
            this.header = new HeaderMsg();
            this.joint_states = new Sensor.JointStateMsg();
            this.pick = new Std.BoolMsg();
            this.place = new Std.BoolMsg();
        }

        public DemonstrationMsg(HeaderMsg header, Sensor.JointStateMsg joint_states, Std.BoolMsg pick, Std.BoolMsg place)
        {
            this.header = header;
            this.joint_states = joint_states;
            this.pick = pick;
            this.place = place;
        }

        public static DemonstrationMsg Deserialize(MessageDeserializer deserializer) => new DemonstrationMsg(deserializer);

        private DemonstrationMsg(MessageDeserializer deserializer)
        {
            this.header = HeaderMsg.Deserialize(deserializer);
            this.joint_states = Sensor.JointStateMsg.Deserialize(deserializer);
            this.pick = Std.BoolMsg.Deserialize(deserializer);
            this.place = Std.BoolMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.joint_states);
            serializer.Write(this.pick);
            serializer.Write(this.place);
        }

        public override string ToString()
        {
            return "DemonstrationMsg: " +
            "\nheader: " + header.ToString() +
            "\njoint_states: " + joint_states.ToString() +
            "\npick: " + pick.ToString() +
            "\nplace: " + place.ToString();
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
