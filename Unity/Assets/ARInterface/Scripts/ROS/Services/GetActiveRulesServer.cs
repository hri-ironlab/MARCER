using UnityEngine;
using RosMessageTypes.RuleMonitor;
using ARInterface.Core.Variables;

namespace ARInterface.ROS
{
    public class GetActiveRulesServer : ROSBase
    {
        [SerializeField] private string _serverTopicName = "rule_monitor/send_active_rules";
        [SerializeField] private ActiveRulesVariable _activeRules;


        private void Start()
        {
            _ROS.ImplementService<SendActiveRulesRequest, SendActiveRulesResponse>(_serverTopicName, UpdateRules);
        }

        private SendActiveRulesResponse UpdateRules(SendActiveRulesRequest req)
        {
            _activeRules.ActiveRules = req.active_rules;
            return new SendActiveRulesResponse(); 
        }
    }
}