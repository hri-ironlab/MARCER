using UnityEngine;
using ARInterface.Core.Events;

namespace ARInterface.TAP
{
    public class TriggerListener : MonoBehaviour, ITriggerAction
    {
        [SerializeField] private TriggerEvent _triggerEvent;
        private string _currentAction;
        private int _id; 

        public void OnTriggerActionEvent(int id, string action)
        {
            Debug.Log(id + " " + action);
        }
        private void OnEnable()
        {
            _triggerEvent.RegisterListener(this);
        }

        private void OnDisable()
        {
            _triggerEvent.UnregisterListener(this);
        }


    }

}
