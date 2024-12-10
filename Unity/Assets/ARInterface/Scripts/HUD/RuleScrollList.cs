using UnityEngine;
using MixedReality.Toolkit.UX.Experimental;
using ARInterface.Core.Variables;

namespace ARInterface.HUD
{
    public class RuleScrollList : MonoBehaviour
    {
        [SerializeField] private ActiveRulesVariable _activeRules;
        private VirtualizedScrollRectList list;

        /// <summary>
        /// A Unity event function that is called on the frame when a script is enabled just before any of the update methods are called the first time.
        /// </summary> 
        private void Start()
        {
            list = GetComponent<VirtualizedScrollRectList>();
            list.OnVisible = (go, i) =>
            {
                go.GetComponent<RuleButtonDisplay>().SetRuleIndex(i);
            };
        }

        private void Update()
        {
            if (_activeRules.ActiveRules != null)
            {
                SetItemCount(_activeRules.ActiveRules.rules.Length);
            }
        }

        private void SetItemCount(int count)
        {
            // Display the number of rules available
            if (list.ItemCount != count)
            {
                list.SetItemCount(count);
            }
        }

        private void OnApplicationQuit()
        {
            _activeRules.Clear();
        }
    }
}