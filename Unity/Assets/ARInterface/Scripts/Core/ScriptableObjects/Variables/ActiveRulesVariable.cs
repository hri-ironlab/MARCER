using UnityEngine;
using RosMessageTypes.RuleMonitor;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/ActiveRulesVariable")]
    public class ActiveRulesVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private ActiveRulesMsg _activeRules;

        public ActiveRulesMsg ActiveRules
        {
            get => _activeRules;
            set => _activeRules = value;
        }

        public void SetValue(ActiveRulesMsg activeRules)
        {
            ActiveRules = activeRules;
        }

        public void Clear() => _activeRules = null;
    }
}