//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.MoveitTaskConstructor
{
    [Serializable]
    public class StageStatisticsMsg : Message
    {
        public const string k_RosMessageName = "moveit_task_constructor_msgs/StageStatistics";
        public override string RosMessageName => k_RosMessageName;

        //  dynamically changing information for a stage
        //  unique id within task
        public uint id;
        //  successful solution IDs of this stage, sorted by increasing cost
        public uint[] solved;
        //  (optional) failed solution IDs of this stage
        public uint[] failed;
        //  number of failed solutions (if failed is empty)
        public uint num_failed;
        //  total computation time in seconds
        public double total_compute_time;

        public StageStatisticsMsg()
        {
            this.id = 0;
            this.solved = new uint[0];
            this.failed = new uint[0];
            this.num_failed = 0;
            this.total_compute_time = 0.0;
        }

        public StageStatisticsMsg(uint id, uint[] solved, uint[] failed, uint num_failed, double total_compute_time)
        {
            this.id = id;
            this.solved = solved;
            this.failed = failed;
            this.num_failed = num_failed;
            this.total_compute_time = total_compute_time;
        }

        public static StageStatisticsMsg Deserialize(MessageDeserializer deserializer) => new StageStatisticsMsg(deserializer);

        private StageStatisticsMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.id);
            deserializer.Read(out this.solved, sizeof(uint), deserializer.ReadLength());
            deserializer.Read(out this.failed, sizeof(uint), deserializer.ReadLength());
            deserializer.Read(out this.num_failed);
            deserializer.Read(out this.total_compute_time);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.id);
            serializer.WriteLength(this.solved);
            serializer.Write(this.solved);
            serializer.WriteLength(this.failed);
            serializer.Write(this.failed);
            serializer.Write(this.num_failed);
            serializer.Write(this.total_compute_time);
        }

        public override string ToString()
        {
            return "StageStatisticsMsg: " +
            "\nid: " + id.ToString() +
            "\nsolved: " + System.String.Join(", ", solved.ToList()) +
            "\nfailed: " + System.String.Join(", ", failed.ToList()) +
            "\nnum_failed: " + num_failed.ToString() +
            "\ntotal_compute_time: " + total_compute_time.ToString();
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
