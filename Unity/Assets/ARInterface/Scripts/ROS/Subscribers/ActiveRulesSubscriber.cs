using UnityEngine;
using ARInterface.Core.Variables;
using RosMessageTypes.RuleMonitor;

namespace ARInterface.ROS
{
    public class ActiveRulesSubscriber : ROSBase
    {
        [SerializeField] private string _topicName = "/rule_monitor/active_rules";
        [SerializeField] private ActiveRulesVariable _activeRulesVariable;

        void Start()
        {
            _ROS.Subscribe<ActiveRulesMsg>(_topicName, HandleMessage);
        }

        void HandleMessage(ActiveRulesMsg msg)
        {
            _activeRulesVariable.ActiveRules = msg;
        }

        private void OnApplicationQuit()
        {
            _activeRulesVariable.ActiveRules = null;
        }
    }
}