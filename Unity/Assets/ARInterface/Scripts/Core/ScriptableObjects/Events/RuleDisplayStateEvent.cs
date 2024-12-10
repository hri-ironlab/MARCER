using UnityEngine;

namespace ARInterface.Core.Events
{
    [CreateAssetMenu(menuName = "Event/RuleDisplayStateEvent")]
    public class RuleDisplayStateEvent : EventBase<IRuleDisplayStateResponse>
    {
        public void Raise(RuleDisplayState response)
        {
            for (int i = listeners.Count - 1; i >= 0; i--)
                listeners[i].OnRuleDisplayStateResponse(response);
        }
    }
}