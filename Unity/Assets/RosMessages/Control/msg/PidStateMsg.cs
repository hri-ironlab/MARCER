//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

namespace RosMessageTypes.Control
{
    [Serializable]
    public class PidStateMsg : Message
    {
        public const string k_RosMessageName = "control_msgs/PidState";
        public override string RosMessageName => k_RosMessageName;

        public HeaderMsg header;
        public DurationMsg timestep;
        public double error;
        public double error_dot;
        public double p_error;
        public double i_error;
        public double d_error;
        public double p_term;
        public double i_term;
        public double d_term;
        public double i_max;
        public double i_min;
        public double output;

        public PidStateMsg()
        {
            this.header = new HeaderMsg();
            this.timestep = new DurationMsg();
            this.error = 0.0;
            this.error_dot = 0.0;
            this.p_error = 0.0;
            this.i_error = 0.0;
            this.d_error = 0.0;
            this.p_term = 0.0;
            this.i_term = 0.0;
            this.d_term = 0.0;
            this.i_max = 0.0;
            this.i_min = 0.0;
            this.output = 0.0;
        }

        public PidStateMsg(HeaderMsg header, DurationMsg timestep, double error, double error_dot, double p_error, double i_error, double d_error, double p_term, double i_term, double d_term, double i_max, double i_min, double output)
        {
            this.header = header;
            this.timestep = timestep;
            this.error = error;
            this.error_dot = error_dot;
            this.p_error = p_error;
            this.i_error = i_error;
            this.d_error = d_error;
            this.p_term = p_term;
            this.i_term = i_term;
            this.d_term = d_term;
            this.i_max = i_max;
            this.i_min = i_min;
            this.output = output;
        }

        public static PidStateMsg Deserialize(MessageDeserializer deserializer) => new PidStateMsg(deserializer);

        private PidStateMsg(MessageDeserializer deserializer)
        {
            this.header = HeaderMsg.Deserialize(deserializer);
            this.timestep = DurationMsg.Deserialize(deserializer);
            deserializer.Read(out this.error);
            deserializer.Read(out this.error_dot);
            deserializer.Read(out this.p_error);
            deserializer.Read(out this.i_error);
            deserializer.Read(out this.d_error);
            deserializer.Read(out this.p_term);
            deserializer.Read(out this.i_term);
            deserializer.Read(out this.d_term);
            deserializer.Read(out this.i_max);
            deserializer.Read(out this.i_min);
            deserializer.Read(out this.output);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.timestep);
            serializer.Write(this.error);
            serializer.Write(this.error_dot);
            serializer.Write(this.p_error);
            serializer.Write(this.i_error);
            serializer.Write(this.d_error);
            serializer.Write(this.p_term);
            serializer.Write(this.i_term);
            serializer.Write(this.d_term);
            serializer.Write(this.i_max);
            serializer.Write(this.i_min);
            serializer.Write(this.output);
        }

        public override string ToString()
        {
            return "PidStateMsg: " +
            "\nheader: " + header.ToString() +
            "\ntimestep: " + timestep.ToString() +
            "\nerror: " + error.ToString() +
            "\nerror_dot: " + error_dot.ToString() +
            "\np_error: " + p_error.ToString() +
            "\ni_error: " + i_error.ToString() +
            "\nd_error: " + d_error.ToString() +
            "\np_term: " + p_term.ToString() +
            "\ni_term: " + i_term.ToString() +
            "\nd_term: " + d_term.ToString() +
            "\ni_max: " + i_max.ToString() +
            "\ni_min: " + i_min.ToString() +
            "\noutput: " + output.ToString();
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
