using UnityEngine;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/StringVariable")]
    public class StringVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private string _value;

        public string Value
        {
            get => _value;
            set => _value = value;
        }

        public void SetValue(string value)
        {
            Value = value;
        }

        public void SetValue(StringVariable stringVariable)
        {
            Value = stringVariable.Value;
        }

        public void Clear() => _value = null;
    }
}