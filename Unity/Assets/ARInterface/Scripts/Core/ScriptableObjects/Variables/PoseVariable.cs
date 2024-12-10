using UnityEngine;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/PoseVariable")]
    public class PoseVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private Vector3 _position;
        [SerializeField] private Quaternion _rotation;

        public Vector3 Position
        {
            get => _position;
            set => _position = value;
        }

        public Quaternion Rotation
        {
            get => _rotation;
            set => _rotation = value;
        }

        public void Clear()
        {
            _position = Vector3.zero;
            _rotation = Quaternion.identity;
        }
    }
}