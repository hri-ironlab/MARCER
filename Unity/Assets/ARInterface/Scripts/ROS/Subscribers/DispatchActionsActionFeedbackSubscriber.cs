using UnityEngine;
using ARInterface.Core.Variables;
using RosMessageTypes.ActionDispatcher;

namespace ARInterface.ROS
{
    public class DispatchActionsActionFeedbackSubscriber : ROSBase
    {
        [SerializeField] private string _topicName = "/action_dispatcher/action_queue/feedback";
        [SerializeField] private DispatchActionsActionFeedbackVariable _dispatchActionsActionFeedbackVariable;
        void Start()
        {
            _ROS.Subscribe<DispatchActionsActionFeedback>(_topicName, HandleMessage);
        }

        void HandleMessage(DispatchActionsActionFeedback msg)
        {

            _dispatchActionsActionFeedbackVariable.Feedback = msg;
        }

        private void OnApplicationQuit()
        {
            _dispatchActionsActionFeedbackVariable.Clear();
        }
    }
}