using UnityEngine;
using RosMessageTypes.Moveit;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/PlanningSceneWorldVariable")]
    public class PlanningSceneWorldVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private PlanningSceneWorldMsg _planningSceneWorld;

        public PlanningSceneWorldMsg PlanningSceneWorld
        {
            get => _planningSceneWorld;
            set => _planningSceneWorld = value;
        }

        public void Clear() => _planningSceneWorld = null;
    }
}
