//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Gazebo
{
    [Serializable]
    public class PerformanceMetricsMsg : Message
    {
        public const string k_RosMessageName = "gazebo_msgs/PerformanceMetrics";
        public override string RosMessageName => k_RosMessageName;

        public HeaderMsg header;
        public double real_time_factor;
        public SensorPerformanceMetricMsg[] sensors;

        public PerformanceMetricsMsg()
        {
            this.header = new HeaderMsg();
            this.real_time_factor = 0.0;
            this.sensors = new SensorPerformanceMetricMsg[0];
        }

        public PerformanceMetricsMsg(HeaderMsg header, double real_time_factor, SensorPerformanceMetricMsg[] sensors)
        {
            this.header = header;
            this.real_time_factor = real_time_factor;
            this.sensors = sensors;
        }

        public static PerformanceMetricsMsg Deserialize(MessageDeserializer deserializer) => new PerformanceMetricsMsg(deserializer);

        private PerformanceMetricsMsg(MessageDeserializer deserializer)
        {
            this.header = HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.real_time_factor);
            deserializer.Read(out this.sensors, SensorPerformanceMetricMsg.Deserialize, deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.real_time_factor);
            serializer.WriteLength(this.sensors);
            serializer.Write(this.sensors);
        }

        public override string ToString()
        {
            return "PerformanceMetricsMsg: " +
            "\nheader: " + header.ToString() +
            "\nreal_time_factor: " + real_time_factor.ToString() +
            "\nsensors: " + System.String.Join(", ", sensors.ToList());
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
