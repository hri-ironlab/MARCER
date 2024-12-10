using UnityEngine;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/IntegerVariable")]
    public class IntegerVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private int _value;

        public int Value
        {
            get => _value;
            set => _value = value;
        }

        public void SetValue(int value)
        {
            Value = value;
        }

        public void SetValue(IntegerVariable integerVariable)
        {
            Value = integerVariable.Value;
        }

        public void Clear() => _value = 0; 
    }
}
