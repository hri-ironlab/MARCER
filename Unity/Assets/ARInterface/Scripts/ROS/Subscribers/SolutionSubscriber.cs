using UnityEngine;
using ARInterface.Core.Variables;
using RosMessageTypes.MoveitTaskConstructor;

namespace ARInterface.ROS
{
    public class SolutionSubscriber : ROSBase
    {
        [SerializeField] private string _topicName = "/manipulation_node/solution";
        [SerializeField] private SolutionVariable _solutionVariable;
        [SerializeField] private bool _clearVariableOnQuit = false; 

        void Start()
        {
            _ROS.Subscribe<SolutionMsg>(_topicName, HandleMessage);
        }

        void HandleMessage(SolutionMsg msg)
        {
            if(_solutionVariable.Solution.task_id != msg.task_id)
            {
                _solutionVariable.Solution = msg;
            } 
        }

        private void OnApplicationQuit()
        {
            if( _clearVariableOnQuit )
            {
                _solutionVariable.Solution = null;
            }
        }
    }
}