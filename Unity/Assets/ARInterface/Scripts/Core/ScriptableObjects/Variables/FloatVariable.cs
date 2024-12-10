using UnityEngine;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/FloatVariable")]
    public class FloatVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private float _value;

        public float Value
        {
            get => _value;
            set => _value = value;
        }

        public void SetValue(float value)
        {
            Value = value;
        }

        public void SetValue(FloatVariable floatVariable)
        {
            Value = floatVariable.Value;
        }
        public void Clear() => _value = 0f;
    }
}