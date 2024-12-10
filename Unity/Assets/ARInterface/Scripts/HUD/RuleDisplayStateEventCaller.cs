using UnityEngine;
using TMPro;
using ARInterface.Core.Variables;
using ARInterface.Core;
using ARInterface.Core.Events; 
using MixedReality.Toolkit.UX;

namespace ARInterface.HUD
{
    public class RuleDisplayStateEventCaller : MonoBehaviour
    {
        [SerializeField] private RuleDisplayStateEvent _ruleDisplayStateEvent;
        [SerializeField] private RuleDisplayState _callerType;

        public void InvokeEvent()
        {
            _ruleDisplayStateEvent.Raise(_callerType);
        }
    }
}
