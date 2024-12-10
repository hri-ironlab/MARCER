using TMPro;
using ARInterface.Core.Events;
using UnityEngine;

namespace ARInterface.HUD {
    public class RaiseUpdateRule : MonoBehaviour
    {
        [SerializeField] private StringEvent _updateRuleEvent;

        public void TriggerEvent()
        {
            foreach (var text in gameObject.GetComponentsInChildren<TextMeshProUGUI>())
            {
                if (text.gameObject.name == "Text")
                {
                    _updateRuleEvent.Raise(text.text);
                }
            }
        }
    }
}


