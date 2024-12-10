using UnityEngine;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/FloatRangeVariable")]
    public class FloatRangeVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField, Range(0f, 1.0f)] private float _value;

        public float Value
        {
            get => _value;
            set => _value = Mathf.Clamp01(value);
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