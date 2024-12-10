using UnityEngine;

namespace ARInterface.Core.Events
{
    [CreateAssetMenu(menuName = "Event/BoolEvent")]
    public class BoolEvent : EventBase<IBoolResponse>
    {
        public void Raise(bool response)
        {
            for (int i = listeners.Count - 1; i >= 0; i--)
                listeners[i].OnBoolResponse(response);
        }
    }
}