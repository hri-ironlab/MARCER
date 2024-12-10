using UnityEngine;
using RosMessageTypes.ActionDispatcher;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/ActionDispatcherStateVariable")]
    public class ActionDispatcherStateVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private ActionDispatcherStateMsg _state;

        public ActionDispatcherStateMsg State
        {
            get => _state;
            set => _state = value;
        }

        public void SetValue(ActionDispatcherStateMsg state)
        {
            State = state;
        }

        public void Clear() => _state = null;
    }
}