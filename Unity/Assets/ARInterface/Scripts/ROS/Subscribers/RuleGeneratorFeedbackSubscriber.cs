using UnityEngine;
using ARInterface.Core.Variables;
using RosMessageTypes.RuleGenerator;

namespace ARInterface.ROS
{
    public class RuleGeneratorFeedbackSubscriber : ROSBase
    {
        [SerializeField] private string _topicName = "/action_planner/feedback";
        [SerializeField] private RuleGeneratorFeedbackVariable _ruleGeneratorFeedbackVariable;

        void Start()
        {
            _ROS.Subscribe<UserFeedbackMsg>(_topicName, HandleMessage);
        }

        void HandleMessage(UserFeedbackMsg msg)
        {
            _ruleGeneratorFeedbackVariable.Feedback = msg;
        }

        private void OnApplicationQuit()
        {
            _ruleGeneratorFeedbackVariable.Clear();
        }
    }
}