using UnityEngine;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/BoolVariable")]
    public class BoolVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private bool _value;

        public bool Value
        {
            get => _value;
            set => _value = value;
        }

        public void SetValue(bool value)
        {
            Value = value;
        }

        public void SetValue(BoolVariable boolVariable)
        {
            Value = boolVariable.Value;
        }

        public void FlipValue() => Value = !Value;

        public void Clear() => _value = false;

    }
}
