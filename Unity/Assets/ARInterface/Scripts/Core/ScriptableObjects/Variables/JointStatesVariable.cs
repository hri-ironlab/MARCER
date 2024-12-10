using UnityEngine;
using RosMessageTypes.Sensor;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/JointStatesVariable")]
    public class JointStatesVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private string _topicName;
        [SerializeField] private JointStateMsg _jointStates;

        public string TopicName
        {
            get => _topicName;
            set => _topicName = value;
        }

        public JointStateMsg JointStates
        {
            get => _jointStates;
            set => _jointStates = value;
        }

        public void Clear() => _jointStates = null;
    }
}