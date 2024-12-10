using UnityEngine;
using TMPro;
using ARInterface.Core.Variables;
using ARInterface.Core;
using ARInterface.Core.Events; 
using MixedReality.Toolkit.UX;

namespace ARInterface.HUD
{
    public class RuleMonitorRunningStateEventCaller : MonoBehaviour
    {
        [SerializeField] private RuleMonitorRunningStateEvent _ruleMonitorRunningStateEvent;
        [SerializeField] private RuleMonitorRunningState _callerType;

        public void InvokeEvent()
        {
            _ruleMonitorRunningStateEvent.Raise(_callerType);
        }
    }
}
