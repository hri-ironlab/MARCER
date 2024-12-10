using UnityEngine;
using ARInterface.Core.Variables;
using RosMessageTypes.MoveitTaskConstructor;

namespace ARInterface.ROS
{
    public class SolutionNoVariableSubscriber : ROSBase
    {
        [SerializeField] private string _topicName = "/manipulation_node/solution";
        public SolutionMsg solution; 

        void Start()
        {
            _ROS.Subscribe<SolutionMsg>(_topicName, HandleMessage);
        }

        void HandleMessage(SolutionMsg msg)
        {
            solution = msg; 
        }
    }
}