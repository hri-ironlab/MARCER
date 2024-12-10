using UnityEngine;
using RosMessageTypes.Vision;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/Detection3DArrayVariable")]
    public class Detection3DArrayVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private Detection3DArrayMsg _value;

        public Detection3DArrayMsg Value
        {
            get => _value;
            set => _value = value;
        }

        public void SetValue(Detection3DArrayMsg value)
        {
            Value = value;
        }

        public void SetValue(Detection3DArrayVariable detection3DArrayVariable)
        {
            Value = detection3DArrayVariable.Value;
        }

        public void Clear() => _value = null;
    }
}