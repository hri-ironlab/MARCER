using UnityEngine;
using ARInterface.Core.Variables;
using RosMessageTypes.ActionDispatcher;

namespace ARInterface.ROS
{
    public class ActionDispatcherStateSubscriber : ROSBase
    {
        [SerializeField] private string _topicName = "/action_dispatcher/state";
        [SerializeField] private ActionDispatcherStateVariable _actionDispatcherStateVariable;

        void Start()
        {
            _ROS.Subscribe<ActionDispatcherStateMsg>(_topicName, HandleMessage);
        }

        void HandleMessage(ActionDispatcherStateMsg msg)
        {
            _actionDispatcherStateVariable.State = msg;
        }

        private void OnApplicationQuit()
        {
            _actionDispatcherStateVariable.State = null;
        }
    }
}