using UnityEngine;

namespace ARInterface.Core.Events
{
    [CreateAssetMenu(menuName = "Event/RuleMonitorRunningStateEvent")]
    public class RuleMonitorRunningStateEvent : EventBase<IRuleMonitorRunningStateResponse>
    {
        public void Raise(RuleMonitorRunningState response)
        {
            for (int i = listeners.Count - 1; i >= 0; i--)
                listeners[i].OnRuleMonitorRunningStateResponse(response);
        }
    }
}