using UnityEngine;
using RosMessageTypes.Manipulation;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/ManipulationPlanRequestVariable")]
    public class ManipulationPlanRequestVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private ManipulationPlanRequestMsg _request;

        public ManipulationPlanRequestMsg Request
        {
            get => _request;
            set => _request = value;
        }

        public void Clear() => _request = null;
    }
}