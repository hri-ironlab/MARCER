using UnityEngine;
using TMPro;
using ARInterface.Core.Variables;
using MixedReality.Toolkit.UX;
using ARInterface.Core.Events;

namespace ARInterface.HUD
{
    public class RuleInfo : MonoBehaviour
    {
        [SerializeField] private StringVariable _ruleDescription;
        private TextMeshPro _text;


        /// <summary>
        /// A Unity event function that is called on the frame when a script is enabled just before any of the update methods are called the first time.
        /// </summary> 
        private void Awake()
        {
            _text = FindChildTextByName("RuleInfoText");
        }

        private void Update()
        {
            if (_ruleDescription.Value != null)
            {
                _text.text = _ruleDescription.Value; 
            }
        }

        private TextMeshPro FindChildTextByName(string name)
        {
            foreach (var text in GetComponentsInChildren<TextMeshPro>())
            {
                if (text.gameObject.name == name)
                {
                    return text;
                }
            }
            return null;
        }

        void OnDisable()
        {
            _ruleDescription.Clear(); 
        }
    }
}
