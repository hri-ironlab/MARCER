using UnityEngine;
using RosMessageTypes.RuleMonitor;
using ARInterface.Core.Events;
using ARInterface.Core.Variables;
using System;
using static UnityEngine.EventSystems.EventTrigger;

namespace ARInterface.ROS
{
    public class DeleteRuleClient : ROSBase, IStringResponse
    {
        [SerializeField] private string _serviceName = "rule_monitor/update_rule";
        [SerializeField] private ActiveRulesVariable _activeRules;
        [SerializeField] private StringEvent _deleteRuleEvent;

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
            request.rule.operation = RuleMsg.REMOVE;
            _ROS.SendServiceMessage<UpdateRuleResponse>(_serviceName, request, OnUpdateRuleResponse);
            Debug.Log("Removed " + response);
        }

        private void OnEnable()
        {
            _deleteRuleEvent.RegisterListener(this);
        }

        private void OnDisable()
        {
            _deleteRuleEvent.UnregisterListener(this);
        }
    }
}