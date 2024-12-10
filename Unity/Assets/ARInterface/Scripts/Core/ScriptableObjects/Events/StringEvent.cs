using UnityEngine;

namespace ARInterface.Core.Events
{
    [CreateAssetMenu(menuName = "Event/StringEvent")]
    public class StringEvent : EventBase<IStringResponse>
    {
        public void Raise(string response)
        {
            for (int i = listeners.Count - 1; i >= 0; i--)
                listeners[i].OnStringResponse(response);
        }
    }
}