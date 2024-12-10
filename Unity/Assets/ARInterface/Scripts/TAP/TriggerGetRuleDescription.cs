using ARInterface.Core.Events;
using UnityEngine;

namespace ARInterface.TAP
{
    public class TriggerGetRuleDescription : MonoBehaviour
    {
        [SerializeField] private StringEvent _getRuleDescriptionEvent;

        public void TriggerEvent()
        {
            _getRuleDescriptionEvent.Raise(name);
        }
    }

}
