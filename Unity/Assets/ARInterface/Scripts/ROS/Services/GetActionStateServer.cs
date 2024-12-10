using UnityEngine;
using RosMessageTypes.ActionDispatcher;
using ARInterface.Core.Variables;

namespace ARInterface.ROS
{
    public class GetActionStateServer : ROSBase
    {
        [SerializeField] private string _serverTopicName = "action_dispatcher/send_state";
        [SerializeField] private ActionDispatcherStateVariable _actionDispatcherStateVariable;


        private void Start()
        {
            _ROS.ImplementService<SendActionStateRequest, SendActionStateResponse>(_serverTopicName, UpdateState);
        }

        private SendActionStateResponse UpdateState(SendActionStateRequest req)
        {
            _actionDispatcherStateVariable.State = req.state;
            return new SendActionStateResponse(); 
        }
    }
}