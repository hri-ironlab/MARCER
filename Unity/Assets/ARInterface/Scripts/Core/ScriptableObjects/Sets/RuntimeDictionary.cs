using System.Collections.Generic;
using UnityEngine;

namespace ARInterface.Core
{
    [System.Serializable]
    public abstract class RuntimeDictionary<T1, T2> : ScriptableObject
    {
        public Dictionary<T1, T2> Items = new();

        public void Add(T1 key, T2 value)
        {
            Items[key] = value;
        }

        public void Remove(T1 key)
        {
            Items.Remove(key);
        }
    }
}