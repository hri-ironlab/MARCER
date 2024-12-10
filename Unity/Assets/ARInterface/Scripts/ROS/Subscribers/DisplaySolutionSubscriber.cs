using UnityEngine;
using ARInterface.Core.Variables;
using RosMessageTypes.ActionDispatcher;

namespace ARInterface.ROS
{
    public class DisplaySolutionSubscriber : ROSBase
    {
        [SerializeField] private string _topicName = "/action_dispatcher/display_solution";
        [SerializeField] private DisplaySolutionVariable _solutionVariable;
        [SerializeField] private bool _clearVariableOnQuit = false; 

        void Start()
        {
            _ROS.Subscribe<DisplaySolutionMsg>(_topicName, HandleMessage);
        }

        void HandleMessage(DisplaySolutionMsg msg)
        {
            Debug.Log("Solution");
            _solutionVariable.Solution = msg;
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