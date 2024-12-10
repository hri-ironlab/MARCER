using UnityEngine;

namespace ARInterface.Core.Events
{
    [CreateAssetMenu(menuName = "Event/TriggerEvent")]
    public class TriggerEvent : EventBase<ITriggerAction>
    {
        public void Raise(int id, string action)
        {
            for (int i = listeners.Count - 1; i >= 0; i--)
                listeners[i].OnTriggerActionEvent(id, action);
        }
    }
}