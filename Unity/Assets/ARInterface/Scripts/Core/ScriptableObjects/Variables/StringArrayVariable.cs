using System;
using UnityEngine;

namespace ARInterface.Core.Variables
{
    [CreateAssetMenu(menuName = "Variables/StringArrayVariable")]
    public class StringArrayVariable : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] private string[] _values;

        public string[] Values
        {
            get => _values;
            set => _values = value;
        }

        public void SetValues(string[] values)
        {
            Values = values;
        }

        public void SetValues(StringArrayVariable stringArrayVariable)
        {
            Values = stringArrayVariable.Values;
        }

        public void Clear() => _values = null;
    }
}