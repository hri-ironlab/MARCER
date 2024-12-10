using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using RosMessageTypes.Sensor;
using ARInterface.Core.Variables;

namespace ARInterface.ROS
{
    public class JointStateSubscriber : ROSBase
    {
        [SerializeField] private string _topicName = "/relaxed_ik/joint_angle_solutions";
        [SerializeField] private JointStatesVariable _jointStatesVariable;

        void Start()
        {
            _ROS.Subscribe<JointStateMsg>(_topicName, HandleMessage);
            _jointStatesVariable.name = _topicName;
            _jointStatesVariable.Clear(); 
        }

        void HandleMessage(JointStateMsg msg)
        {
            _jointStatesVariable.JointStates = msg;
        }
    }
}