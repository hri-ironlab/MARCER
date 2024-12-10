using System.Collections.Generic;
using UnityEngine;

namespace ARInterface.TAP
{
    [CreateAssetMenu(menuName = "Sets/TriggerList")]
    public class TriggerList : ScriptableObject
    {
#if UNITY_EDITOR
        [Multiline] public string DeveloperDescription = "";
#endif

        [SerializeField] public List<TriggerBase> _triggers = new();

        public TriggerBase GetTriggerByBaseType(int type)
        {
            foreach (var trigger in _triggers)
            {
                if (trigger.TriggerType == type)
                {
                    return trigger;
                }
            }
            return null;
        }

        public TriggerBase GetTriggerByBaseName(string name)
        {
            foreach (var trigger in _triggers)
            {
                if (trigger.TriggerName == name)
                {
                    return trigger;
                }
            }
            return null;
        }

        public TriggerBase GetTriggerByParentName(string name)
        {
            foreach (var trigger in _triggers)
            {
                if (trigger.gameObject.name == name)
                {
                    return trigger;
                }
            }
            return null;
        }

        public List<TriggerBase> GetTriggersByBaseType(int type)
        {
            List<TriggerBase> triggers = new List<TriggerBase>();

            foreach (var trigger in _triggers)
            {
                if (trigger.TriggerType == type)
                {
                    triggers.Add(trigger);
                }
            }
            return triggers;
        }

        public List<TriggerBase> GetTriggersByBaseName(string name)
        {
            List<TriggerBase> triggers = new List<TriggerBase>();

            foreach (var trigger in _triggers)
            {
                if (trigger.TriggerName == name)
                {
                    triggers.Add(trigger);
                }
            }
            return triggers;
        }

        public void AddTrigger(TriggerBase trigger)
        {
            _triggers.Add(trigger);
        }

        public void DestroyTriggerByName(string name)
        {
            TriggerBase trigger = GetTriggerByParentName(name);
            if(trigger != null)
            {
                _triggers.Remove(trigger);
                Destroy(trigger.gameObject);
            }
        }

        public void ClearTriggers()
        {
            for (int i = 0; i < _triggers.Count; i++)
            {
                var temp = _triggers[i];
                _triggers.Remove(temp);
                Destroy(temp); 
            }
            _triggers.Clear(); 
        }
    }
}
