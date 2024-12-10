using ARInterface.Core.Events;
using UnityEngine;

namespace ARInterface.TAP
{
    public class OnTriggerDelete : MonoBehaviour
    {
        [SerializeField] private StringEvent _deleteTriggerEvent;
        [SerializeField] private StringEvent _loggingEvent; 
        public void TriggerEvent()
        {
            _deleteTriggerEvent.Raise(name);
            _loggingEvent.Raise(name);
        }
    }

}
