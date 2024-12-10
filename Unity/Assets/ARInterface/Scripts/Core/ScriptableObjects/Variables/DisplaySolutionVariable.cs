using UnityEngine;
using RosMessageTypes.ActionDispatcher;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/DisplaySolutionVariable")]
    public class DisplaySolutionVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private DisplaySolutionMsg _solution;

        public DisplaySolutionMsg Solution
        {
            get => _solution;
            set => _solution = value;
        }

        public void SetValue(DisplaySolutionMsg solution)
        {
            Solution = solution;
        }

        public void Clear() => _solution = null;
    }
}