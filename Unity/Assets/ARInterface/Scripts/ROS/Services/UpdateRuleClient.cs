using UnityEngine;
using RosMessageTypes.RuleMonitor;
using ARInterface.Core.Events;
using ARInterface.Core.Variables;
using System;
using static UnityEngine.EventSystems.EventTrigger;

namespace ARInterface.ROS
{
    public class UpdateRuleClient : ROSBase, IStringResponse
    {
        [SerializeField] private string _serviceName = "rule_monitor/update_rule";
        [SerializeField] private ActiveRulesVariable _activeRules;
        [SerializeField] private StringEvent _updateRuleEvent;
        [SerializeField] private BoolVariable _isInDeleteState; 
        private void Start()
        {
            _ROS.RegisterRosService<UpdateRuleRequest, UpdateRuleResponse>(_serviceName);
        }

        private void OnUpdateRuleResponse(UpdateRuleResponse response)
        {
            _activeRules.ActiveRules = response.active_rules;
        }

        public void OnStringResponse(string response)
        {
            UpdateRuleRequest request = new UpdateRuleRequest();
            request.rule = new RuleMsg
            {
                name = response
            };
            if (_isInDeleteState.Value)
            {
                request.rule.operation = RuleMsg.REMOVE;
                _ROS.SendServiceMessage<UpdateRuleResponse>(_serviceName, request, OnUpdateRuleResponse);
                Debug.Log("Removed " + response);
            }
            else
            {
                request.rule.operation = RuleMsg.TOGGLE_IS_ACTIVE;
                _ROS.SendServiceMessage<UpdateRuleResponse>(_serviceName, request, OnUpdateRuleResponse);
                Debug.Log("Toggle Activity of " + response);
            }

        }
        private void OnEnable()
        {
            _updateRuleEvent.RegisterListener(this);
        }

        private void OnDisable()
        {
            _updateRuleEvent.UnregisterListener(this);
        }
    }
}