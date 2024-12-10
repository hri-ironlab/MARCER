using UnityEngine;
using RosMessageTypes.ActionDispatcher;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/DispatchActionsActionFeedbackVariable")]
    public class DispatchActionsActionFeedbackVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private DispatchActionsActionFeedback _feedback;

        public DispatchActionsActionFeedback Feedback
        {
            get => _feedback;
            set => _feedback = value;
        }

        public void Clear() => _feedback = null;
    }
}