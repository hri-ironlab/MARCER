using UnityEngine;
using RosMessageTypes.RuleGenerator;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/RuleGeneratorStateVariable")]
    public class RuleGeneratorStateVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private RuleGeneratorStateMsg _state;

        public RuleGeneratorStateMsg State
        {
            get => _state;
            set => _state = value;
        }

        public void SetValue(RuleGeneratorStateMsg state)
        {
            State = state;
        }

        public void Clear() => _state = null;
    }
}