using UnityEngine;
using RosMessageTypes.ActionDispatcher;
using ARInterface.Core.Variables;

namespace ARInterface.ROS
{
    public class GetManipulationPlanFeedbackServer : ROSBase
    {
        [SerializeField] private string _serverTopicName = "action_dispatcher/send_manipulation_plan_feedback";
        [SerializeField] private ManipulationPlanRequestVariable _manipulationPlanRequestVariable;


        private void Start()
        {
            _ROS.ImplementService<SendManipulationPlanFeedbackRequest, SendManipulationPlanFeedbackResponse>(_serverTopicName, UpdateFeedback);
        }

        private SendManipulationPlanFeedbackResponse UpdateFeedback(SendManipulationPlanFeedbackRequest req)
        {

            _manipulationPlanRequestVariable.Request = req.current_action;
            return new SendManipulationPlanFeedbackResponse(); 
        }
    }
}