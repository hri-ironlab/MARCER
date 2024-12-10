using UnityEngine;
using RosMessageTypes.MoveitTaskConstructor;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/ExecuteTaskSolutionGoalVariable")]
    public class ExecuteTaskSolutionGoalVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private ExecuteTaskSolutionActionGoal _goal;

        public ExecuteTaskSolutionActionGoal Goal
        {
            get => _goal;
            set => _goal = value;
        }

        public void SetValue(ExecuteTaskSolutionActionGoal goal)
        {
            Goal = goal;
        }

        public void Clear() => _goal = null;
    }
}