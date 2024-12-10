using UnityEngine;
using RosMessageTypes.RuleMonitor;
using ARInterface.Core.Events;
using ARInterface.Core.Variables;
using System;
using static UnityEngine.EventSystems.EventTrigger;

namespace ARInterface.ROS
{
    public class GetRuleDescription : ROSBase, IStringResponse
    {
        [SerializeField] private string _serviceName = "rule_monitor/get_rule_description";
        [SerializeField] private StringEvent _getRuleDescriptionEvent;
        [SerializeField] private StringVariable _ruleDescription;

        private void Start()
        {
            _ROS.RegisterRosService<GetRuleDescriptionRequest, GetRuleDescriptionResponse>(_serviceName);
        }

        private void HandleGetRuleDescriptionResponse(GetRuleDescriptionResponse response)
        {
            _ruleDescription.Value = response.description;
        }

        public void OnStringResponse(string response)
        {
            if (response == "")
            {
                _ruleDescription.Value = "";
            }
            else
            {
                _ROS.SendServiceMessage<GetRuleDescriptionResponse>(_serviceName, new GetRuleDescriptionRequest(response), HandleGetRuleDescriptionResponse);
            }
        }
        private void OnEnable()
        {
            _getRuleDescriptionEvent.RegisterListener(this);
        }

        private void OnDisable()
        {
            _getRuleDescriptionEvent.UnregisterListener(this);
        }
    }
}