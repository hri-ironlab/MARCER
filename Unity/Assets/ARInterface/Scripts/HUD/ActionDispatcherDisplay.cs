using ARInterface.Core.Variables;
using TMPro;
using UnityEngine;
using RosMessageTypes.ActionDispatcher;
using RosMessageTypes.RuleGenerator;
using RosMessageTypes.Manipulation;
using ARInterface.Core.Events;

namespace ARInterface.HUD
{
    public class ActionDispatcherDisplay : MonoBehaviour
    {
        [SerializeField] private TextMeshPro _actionDispatcherDisplay;
        [SerializeField] private DispatchActionsActionFeedbackVariable _dispatchActionsActionFeedbackVariable;
        [SerializeField] private ManipulationPlanRequestVariable _manipulationPlanRequestVariable; 
        [SerializeField] private ActionDispatcherStateVariable _actionDispatcherStateVariable;
        private ManipulationPlanRequestMsg req = new ManipulationPlanRequestMsg();

        private ActionDispatcherStateMsg _lastSentLoggingState = new ActionDispatcherStateMsg();
        [SerializeField] private StringEvent _loggingEvent; 
        // Start is called before the first frame update
        void Start()
        {
        }


        // Update is called once per frame
        void Update()
        {
            if(_manipulationPlanRequestVariable.Request != null)
            {
                req = _manipulationPlanRequestVariable.Request;

            }
            switch (_actionDispatcherStateVariable.State.state)
            {
                case ActionDispatcherStateMsg.IDLE:
                    _actionDispatcherDisplay.text = "Waiting...";
                    break;
                case ActionDispatcherStateMsg.PLANNING:
                    _actionDispatcherDisplay.text = "Planning action " + req.task_name + "(" + req.target_object_name + ")";
                    break;
                case ActionDispatcherStateMsg.RECOVERY:
                    _actionDispatcherDisplay.text = "Executing action " + req.task_name + "(" + req.target_object_name + ")";
                    break;
                case ActionDispatcherStateMsg.MONITOR_EXECUTION:
                    _actionDispatcherDisplay.text = "Executing action " + req.task_name + "(" + req.target_object_name + ")";
                    break;
                case ActionDispatcherStateMsg.SEND_PLAN:
                    _actionDispatcherDisplay.text = "Planning action " + req.task_name + "(" + req.target_object_name + ")";
                    break;
                case RuleGeneratorStateMsg.DONE:
                    _actionDispatcherDisplay.text = "Done";
                    break;
            }
            if (_lastSentLoggingState.state != _actionDispatcherStateVariable.State.state)
            {
                _loggingEvent.Raise(_actionDispatcherDisplay.text);
                _lastSentLoggingState.state = _actionDispatcherStateVariable.State.state; 
            }
        }

        private void OnApplicationQuit()
        {
            _manipulationPlanRequestVariable.Clear(); 
            _dispatchActionsActionFeedbackVariable.Clear();
            _actionDispatcherStateVariable.Clear();
        }
    }
}