using UnityEngine;
using RosMessageTypes.RuleMonitor;
using ARInterface.Core.Events;
using ARInterface.Core.Variables;
using System;
using static UnityEngine.EventSystems.EventTrigger;

namespace ARInterface.ROS
{
    public class PauseRuleClient : ROSBase, IStringResponse
    {
        [SerializeField] private string _serviceName = "rule_monitor/update_rule";
        [SerializeField] private ActiveRulesVariable _activeRules;
        [SerializeField] private StringEvent _pauseRuleEvent;

        private void Start()
        {
            _ROS.RegisterRosService<UpdateRuleRequest, UpdateRuleResponse>(_serviceName);
        }

        private void OnUpdateRuleResponse(UpdateRuleResponse response)
        {
            Debug.Log("Updated Rule Activity Status.");
        }

        public void OnStringResponse(string response)
        {
            UpdateRuleRequest request = new UpdateRuleRequest();
            request.rule = new RuleMsg
            {
                name = response
            };
            request.rule.operation = RuleMsg.TOGGLE_IS_ACTIVE;
            _ROS.SendServiceMessage<UpdateRuleResponse>(_serviceName, request, OnUpdateRuleResponse);
            Debug.Log("Toggle Activity of " + response);
        }
        private void OnEnable()
        {
            _pauseRuleEvent.RegisterListener(this);
        }

        private void OnDisable()
        {
            _pauseRuleEvent.UnregisterListener(this);
        }
    }
}