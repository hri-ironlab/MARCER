using UnityEngine;
using RosMessageTypes.RuleGenerator;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/RuleGeneratorFeedbackVariable")]
    public class RuleGeneratorFeedbackVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private UserFeedbackMsg _feedback;

        public UserFeedbackMsg Feedback
        {
            get => _feedback;
            set => _feedback = value;
        }

        public void SetValue(UserFeedbackMsg feedback)
        {
            Feedback = feedback;
        }

        public void Clear() => _feedback = null;
    }
}