using UnityEngine;
using RosMessageTypes.MoveitTaskConstructor;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/SolutionVariable")]
    public class SolutionVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private SolutionMsg _solution;

        public SolutionMsg Solution
        {
            get => _solution;
            set => _solution = value;
        }

        public void SetValue(SolutionMsg solution)
        {
            Solution = solution;
        }

        public void Clear() => _solution = null;
    }
}