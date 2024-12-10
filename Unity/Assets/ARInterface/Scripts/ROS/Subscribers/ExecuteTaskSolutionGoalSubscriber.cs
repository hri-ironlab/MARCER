using UnityEngine;
using ARInterface.Core.Variables;
using RosMessageTypes.MoveitTaskConstructor;

namespace ARInterface.ROS
{
    public class ExecuteTaskSolutionGoalSubscriber : ROSBase
    {
        [SerializeField] private string _topicName = "/execute_task_solution/goal";
        [SerializeField] private ExecuteTaskSolutionGoalVariable _executeTaskSolutionVariable;

        void Start()
        {
            _ROS.Subscribe<ExecuteTaskSolutionActionGoal>(_topicName, HandleMessage);
        }

        void HandleMessage(ExecuteTaskSolutionActionGoal msg)
        {
            _executeTaskSolutionVariable.Goal = msg;
        }

        private void OnApplicationQuit()
        {
            _executeTaskSolutionVariable.Goal = null;
        }
    }
}