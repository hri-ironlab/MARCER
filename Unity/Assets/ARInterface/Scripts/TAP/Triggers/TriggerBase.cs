using UnityEngine;

namespace ARInterface.TAP
{
    public abstract class TriggerBase : MonoBehaviour
    {
        public int UniqueId { get; protected set; }

        [SerializeField] protected int triggerType;
        public int TriggerType { get { return triggerType; } }

        [SerializeField] protected string triggerName;
        public string TriggerName { get { return triggerName; } }

        public string Action { get; protected set; }

        public void SetUniqueId(int uniqueId)
        {
            UniqueId = uniqueId;
        }

        public void SetAction(string action)
        {
            Action = action;
        }

        public void SetTriggerType(int triggerType)
        {
            this.triggerType = triggerType;
        }

        public void SetTriggerName(string triggerName)
        {
            this.triggerName = triggerName;
        }
    }
}