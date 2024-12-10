using UnityEngine;

namespace ARInterface.Core.Events
{
    [CreateAssetMenu(menuName = "Event/StringArrayEvent")]
    public class StringArrayEvent : EventBase<IStringArrayResponse>
    {
        public void Raise(string[] response)
        {
            for (int i = listeners.Count - 1; i >= 0; i--)
                listeners[i].OnStringArrayResponse(response);
        }
    }
}