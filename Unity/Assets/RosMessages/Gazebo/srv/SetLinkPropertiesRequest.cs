//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Gazebo
{
    [Serializable]
    public class SetLinkPropertiesRequest : Message
    {
        public const string k_RosMessageName = "gazebo_msgs/SetLinkProperties";
        public override string RosMessageName => k_RosMessageName;

        public string link_name;
        //  name of link
        //  link names are prefixed by model name, e.g. pr2::base_link
        public Geometry.PoseMsg com;
        //  center of mass location in link frame
        //  and orientation of the moment of inertias
        //  relative to the link frame
        public bool gravity_mode;
        //  set gravity mode on/off
        public double mass;
        //  linear mass of link
        public double ixx;
        //  moment of inertia
        public double ixy;
        //  moment of inertia
        public double ixz;
        //  moment of inertia
        public double iyy;
        //  moment of inertia
        public double iyz;
        //  moment of inertia
        public double izz;
        //  moment of inertia

        public SetLinkPropertiesRequest()
        {
            this.link_name = "";
            this.com = new Geometry.PoseMsg();
            this.gravity_mode = false;
            this.mass = 0.0;
            this.ixx = 0.0;
            this.ixy = 0.0;
            this.ixz = 0.0;
            this.iyy = 0.0;
            this.iyz = 0.0;
            this.izz = 0.0;
        }

        public SetLinkPropertiesRequest(string link_name, Geometry.PoseMsg com, bool gravity_mode, double mass, double ixx, double ixy, double ixz, double iyy, double iyz, double izz)
        {
            this.link_name = link_name;
            this.com = com;
            this.gravity_mode = gravity_mode;
            this.mass = mass;
            this.ixx = ixx;
            this.ixy = ixy;
            this.ixz = ixz;
            this.iyy = iyy;
            this.iyz = iyz;
            this.izz = izz;
        }

        public static SetLinkPropertiesRequest Deserialize(MessageDeserializer deserializer) => new SetLinkPropertiesRequest(deserializer);

        private SetLinkPropertiesRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.link_name);
            this.com = Geometry.PoseMsg.Deserialize(deserializer);
            deserializer.Read(out this.gravity_mode);
            deserializer.Read(out this.mass);
            deserializer.Read(out this.ixx);
            deserializer.Read(out this.ixy);
            deserializer.Read(out this.ixz);
            deserializer.Read(out this.iyy);
            deserializer.Read(out this.iyz);
            deserializer.Read(out this.izz);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.link_name);
            serializer.Write(this.com);
            serializer.Write(this.gravity_mode);
            serializer.Write(this.mass);
            serializer.Write(this.ixx);
            serializer.Write(this.ixy);
            serializer.Write(this.ixz);
            serializer.Write(this.iyy);
            serializer.Write(this.iyz);
            serializer.Write(this.izz);
        }

        public override string ToString()
        {
            return "SetLinkPropertiesRequest: " +
            "\nlink_name: " + link_name.ToString() +
            "\ncom: " + com.ToString() +
            "\ngravity_mode: " + gravity_mode.ToString() +
            "\nmass: " + mass.ToString() +
            "\nixx: " + ixx.ToString() +
            "\nixy: " + ixy.ToString() +
            "\nixz: " + ixz.ToString() +
            "\niyy: " + iyy.ToString() +
            "\niyz: " + iyz.ToString() +
            "\nizz: " + izz.ToString();
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