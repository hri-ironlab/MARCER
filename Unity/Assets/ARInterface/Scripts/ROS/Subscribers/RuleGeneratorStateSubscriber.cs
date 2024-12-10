using UnityEngine;
using ARInterface.Core.Variables;
using RosMessageTypes.RuleGenerator;

namespace ARInterface.ROS
{
    public class RuleGeneratorStateSubscriber : ROSBase
    {
        [SerializeField] private string _topicName = "/rule_generator/state";
        [SerializeField] private RuleGeneratorStateVariable _ruleGeneratorStateVariable;

        void Start()
        {
            _ROS.Subscribe<RuleGeneratorStateMsg>(_topicName, HandleMessage);
        }

        void HandleMessage(RuleGeneratorStateMsg msg)
        {
            _ruleGeneratorStateVariable.State = msg;
        }

        private void OnApplicationQuit()
        {
            _ruleGeneratorStateVariable.State = null;
        }
    }
}