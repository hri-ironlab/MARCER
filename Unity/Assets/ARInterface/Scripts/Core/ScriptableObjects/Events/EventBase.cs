using UnityEngine;
using System;
using System.Collections.Generic;

namespace ARInterface.Core.Events
{
    public abstract class EventBase : ScriptableObject
    {
    }
    public abstract class EventBase<T> : EventBase, IClearListeners
    {

        public List<T> listeners = new List<T>();

        public virtual void ClearListeners()
        {
            listeners.Clear();
        }

        public virtual void RegisterListener(T listener) => listeners.Add(listener);

        public virtual void UnregisterListener(T listener) => listeners.Remove(listener);
    }
}