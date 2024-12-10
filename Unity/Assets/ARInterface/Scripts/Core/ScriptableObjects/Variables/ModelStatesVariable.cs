using UnityEngine;
using RosMessageTypes.Gazebo;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/ModelStatesVariable")]
    public class ModelStatesVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private string _topicName;
        [SerializeField] private ModelStatesMsg _modelStates;

        public string TopicName
        {
            get => _topicName;
            set => _topicName = value;
        }

        public ModelStatesMsg ModelStates
        {
            get => _modelStates;
            set => _modelStates = value;
        }

        public void Clear() => _modelStates = null;
    }
}